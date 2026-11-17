#!/usr/bin/env python3
"""Prepare a geometry-free OpenRAVE chain and reference FK samples; no ROS nodes."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tarfile
import urllib.request
import xml.etree.ElementTree as ET
import numpy as np

WS = Path('/data/Projects/fr5_ros2')


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def origin(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rotation(rpy)
    t[:3, 3] = xyz
    return t


def build_model(urdf, srdf):
    root, semantic = ET.fromstring(urdf), ET.fromstring(srdf)
    group = semantic.find("group[@name='fairino5_v6_group']")
    chain = None if group is None else group.find('chain')
    if chain is None or chain.attrib != {'base_link': 'base_link', 'tip_link': 'wrist3_link'}:
        raise RuntimeError('Unexpected planning chain; refusing to guess frames')
    names = [f'j{i}' for i in range(1, 7)]
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    robot = ET.Element('Robot', name=root.attrib['name'])
    body = ET.SubElement(robot, 'KinBody')
    ET.SubElement(body, 'Body', name='base_link', type='static')
    t = np.eye(4)
    original_origins, limits, adjustments = [], [], []
    parent = 'base_link'
    for name in names:
        j = joints[name]
        if j.attrib['type'] != 'revolute' or j.find('parent').attrib['link'] != parent:
            raise RuntimeError(f'{name}: unexpected chain topology')
        if [float(v) for v in j.find('axis').attrib['xyz'].split()] != [0, 0, 1]:
            raise RuntimeError(f'{name}: this generator expects a local Z axis')
        o = j.find('origin')
        xyz = list(map(float, o.attrib.get('xyz', '0 0 0').split()))
        rpy = list(map(float, o.attrib.get('rpy', '0 0 0').split()))
        original_origins.append(origin(xyz, rpy))
        # The supplied model uses 1.5708 for pi/2. Snap only near exact quarter-turns
        # in the generation copy, documenting every adjustment and verifying FK.
        snapped = []
        for angle in rpy:
            nearest = round(angle / (math.pi / 2)) * (math.pi / 2)
            snapped.append(nearest if abs(angle - nearest) < 5e-6 else angle)
        if snapped != rpy:
            adjustments.append({'joint': name, 'original_rpy': rpy, 'generation_rpy': snapped})
        t = t @ origin(xyz, snapped)
        child = j.find('child').attrib['link']
        link = ET.SubElement(body, 'Body', name=child, type='dynamic')
        ET.SubElement(link, 'Translation').text = ' '.join(format(v, '.15g') for v in t[:3, 3])
        matrix = t[:3, :3].copy()
        matrix[np.abs(matrix) < 1e-12] = 0
        ET.SubElement(link, 'RotationMat').text = ' '.join(format(v, '.15g') for v in matrix.ravel())
        joint = ET.SubElement(body, 'Joint', name=name, type='hinge')
        ET.SubElement(joint, 'body').text = parent
        ET.SubElement(joint, 'body').text = child
        ET.SubElement(joint, 'offsetfrom').text = child
        ET.SubElement(joint, 'anchor').text = '0 0 0'
        ET.SubElement(joint, 'axis').text = '0 0 1'
        lim = j.find('limit')
        lower, upper = float(lim.attrib['lower']), float(lim.attrib['upper'])
        if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
            raise RuntimeError(f'{name}: invalid limits')
        limits.append([lower, upper])
        ET.SubElement(joint, 'limitsdeg').text = f'{math.degrees(lower):.15g} {math.degrees(upper):.15g}'
        parent = child
    if parent != 'wrist3_link':
        raise RuntimeError('Unexpected end-effector')
    manip = ET.SubElement(robot, 'Manipulator', name='fairino5_v6_group')
    ET.SubElement(manip, 'base').text = 'base_link'
    ET.SubElement(manip, 'effector').text = 'wrist3_link'
    rng = np.random.RandomState(613)
    qs = [np.zeros(6)]
    bounds = np.asarray(limits)
    qs += [rng.uniform(bounds[:, 0]+0.05, bounds[:, 1]-0.05) for _ in range(1000)]
    references = []
    for q in qs:
        transform = np.eye(4)
        for fixed, angle in zip(original_origins, q):
            transform = transform @ fixed @ origin([0, 0, 0], [0, 0, angle])
        references.append({'q': q.tolist(), 'transform': transform.tolist()})
    return ET.tostring(robot, encoding='unicode'), {'joints': names, 'limits': limits,
        'adjustments': adjustments, 'samples': references, 'position_tolerance_m': 2e-5,
        'rotation_tolerance_rad': 2e-5}


def main():
    work = Path(sys.argv[1]).resolve()
    work.mkdir(parents=True, exist_ok=True)
    config = WS / 'src/fairino5_v6_moveit2_config/config'
    urdf = (WS / 'src/fairino_description/urdf/fairino5_v6.urdf').read_text()
    srdf = (config / 'fairino5_v6_robot.srdf').read_text()
    xml, reference = build_model(urdf, srdf)
    (work / 'robot.xml').write_text(xml)
    (work / 'reference.json').write_text(json.dumps(reference))
    (work / 'original.urdf').write_text(urdf)
    (work / 'original.srdf').write_text(srdf)
    provenance = {'urdf_sha256': hashlib.sha256(urdf.encode()).hexdigest(),
                  'srdf_sha256': hashlib.sha256(srdf.encode()).hexdigest(),
                  'kinematics_before': (config / 'kinematics.yaml').read_text(),
                  'adjustments': reference['adjustments']}
    (work / 'provenance.json').write_text(json.dumps(provenance, indent=2))
    # Use the SymPy version recommended by the installed MoveIt generator,
    # confined to the code-generation container via PYTHONPATH.
    deps = work / 'deps'
    deps.mkdir(exist_ok=True)
    # Reuse our previously prepared, isolated dependency. It was hash-checked
    # on download; this retry needs neither PyPI nor a Docker registry.
    candidates = sorted((WS / 'outputs').glob('ikfast_generation_*/deps/sympy-0.7.1'),
                        key=lambda p: p.stat().st_mtime, reverse=True)
    for cached in candidates:
        if cached.resolve() == (deps / 'sympy-0.7.1').resolve():
            continue
        if (cached / 'sympy/__init__.py').is_file():
            shutil.copytree(cached, deps / 'sympy-0.7.1')
            print('Prepared model; reused isolated SymPy from: ' + str(cached))
            return
    with urllib.request.urlopen('https://pypi.org/pypi/sympy/0.7.1/json', timeout=30) as response:
        metadata = json.load(response)
    distribution = next(x for x in metadata['urls'] if x['filename'] == 'sympy-0.7.1.tar.gz')
    archive = deps / distribution['filename']
    with urllib.request.urlopen(distribution['url'], timeout=90) as response, archive.open('wb') as output:
        shutil.copyfileobj(response, output)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != distribution['digests']['sha256']:
        raise RuntimeError('SymPy download hash mismatch')
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            destination = (deps / member.name).resolve()
            destination.relative_to(deps.resolve())
            if member.isdir():
                destination.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                destination.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(member) as source, destination.open('wb') as target:
                    shutil.copyfileobj(source, target)
            else:
                raise RuntimeError('Unexpected archive entry: ' + member.name)
    print('Prepared generation-only model; original URDF and KDL configuration unchanged.')


if __name__ == '__main__':
    main()

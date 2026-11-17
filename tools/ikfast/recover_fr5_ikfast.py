#!/usr/bin/env python3
"""Recover generated C++, compile and validate on Ubuntu without robot access."""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from prepare_model import origin

WS = Path('/data/Projects/fr5_ros2')
TOOLS = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def residual(left, right):
    if not np.all(np.isfinite(left)) or not np.all(np.isfinite(right)):
        return float('inf'), float('inf')
    dp = float(np.linalg.norm(left[:3, 3] - right[:3, 3]))
    dr = math.acos(float(np.clip((np.trace(left[:3, :3].T @ right[:3, :3])-1)/2, -1, 1)))
    return dp, dr


def main():
    generation = Path(sys.argv[1]).resolve()
    generation.relative_to((WS / 'outputs').resolve())
    current_urdf = WS / 'src/fairino_description/urdf/fairino5_v6.urdf'
    current_srdf = WS / 'src/fairino5_v6_moveit2_config/config/fairino5_v6_robot.srdf'
    for saved, current in [('original.urdf', current_urdf), ('original.srdf', current_srdf)]:
        if digest(generation / saved) != digest(current):
            raise RuntimeError('Robot model changed after generation: ' + saved)
    candidates = []
    for path in (generation / '.openrave').rglob('*.cpp'):
        contents = path.read_text(errors='replace')
        if 'ComputeIk' in contents and 'ComputeFk' in contents and 'GetNumJoints' in contents:
            candidates.append(path)
    # A failed OpenRAVE compile can leave a second copy with another filename.
    by_hash = {}
    for path in candidates:
        by_hash.setdefault(digest(path), path)
    if len(by_hash) != 1:
        raise RuntimeError('Expected one distinct generated solver, found: ' + repr([str(p) for p in candidates]))
    source = next(iter(by_hash.values()))
    work = generation / ('host_validation_' + time.strftime('%Y%m%d_%H%M%S'))
    work.mkdir(exist_ok=False)
    shutil.copy2(source, work / 'fr5_ikfast_solver.cpp')
    header = source.parent / 'ikfast.h'
    if not header.is_file():
        header = Path('/opt/ros/humble/share/moveit_kinematics/ikfast_kinematics_plugin/templates/ikfast.h')
    shutil.copy2(header, work / 'ikfast.h')
    shutil.copy2(TOOLS / 'host_ikfast_api.cpp', work / 'host_ikfast_api.cpp')
    shutil.copy2(Path(__file__), work / 'validation_script.py')
    print('Recovered solver:', source, flush=True)
    print('Host validation:', work, flush=True)
    compiler = shutil.which('g++')
    if not compiler:
        raise RuntimeError('Ubuntu g++ is missing; install build-essential')
    command = [compiler, '-std=c++14', '-O2', '-fPIC', '-shared',
               'host_ikfast_api.cpp', '-o', 'libfr5_ikfast_test.so', '-llapack', '-lblas']
    print('Compiling existing solver on Ubuntu; symbolic generation will NOT run.', flush=True)
    result = subprocess.run(command, cwd=work, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=900)
    (work / 'compile.log').write_text(result.stdout)
    if result.returncode:
        print(result.stdout[-12000:])
        raise RuntimeError('Host compiler failed; see compile.log')
    api = ctypes.CDLL(str(work / 'libfr5_ikfast_test.so'))
    pointer = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    api.fr5_fk.argtypes = [pointer, pointer, pointer]
    api.fr5_fk.restype = ctypes.c_int
    api.fr5_ik.argtypes = [pointer, pointer, pointer, ctypes.c_int]
    api.fr5_ik.restype = ctypes.c_int
    if api.fr5_joint_count() != 6 or api.fr5_free_count() != 0:
        raise RuntimeError('Expected six joints with no required free parameter')
    reference = json.loads((generation / 'reference.json').read_text())
    limits = np.array(reference['limits'])
    root = ET.parse(generation / 'original.urdf').getroot()
    fixed = []
    for name in reference['joints']:
        entry = root.find("joint[@name='%s']/origin" % name)
        fixed.append(origin(list(map(float, entry.get('xyz', '0 0 0').split())),
                            list(map(float, entry.get('rpy', '0 0 0').split()))))

    def original_fk(q):
        transform = np.eye(4)
        for joint, angle in zip(fixed, q):
            transform = transform @ joint @ origin([0, 0, 0], [0, 0, angle])
        return transform

    def solver_fk(q):
        translation, rotation = np.zeros(3), np.zeros(9)
        if api.fr5_fk(np.ascontiguousarray(q, dtype=np.float64), translation, rotation) != 0:
            raise RuntimeError('Generated FK raised an exception')
        transform = np.eye(4)
        transform[:3, 3], transform[:3, :3] = translation, rotation.reshape(3, 3)
        return transform

    stats = {name: {'tested': 0, 'solved_in_limits': 0, 'returned_solutions': 0,
                    'invalid_solutions': 0, 'api_errors': 0,
                    'max_position_error_m': 0.0, 'max_rotation_error_rad': 0.0}
             for name in ('generated_targets', 'original_urdf_targets', 'wrist_zero_targets')}
    examples = []
    def check(target, name):
        s = stats[name]
        s['tested'] += 1
        values = np.zeros((64, 6))
        count = api.fr5_ik(np.ascontiguousarray(target[:3, 3]),
                           np.ascontiguousarray(target[:3, :3].ravel()), values, 64)
        if count < 0:
            s['api_errors'] += 1
            if len(examples) < 10:
                examples.append({'suite': name, 'case': s['tested'], 'api_return': count})
            return
        solved = False
        for q in values[:count]:
            s['returned_solutions'] += 1
            if not np.all(np.isfinite(q)):
                s['invalid_solutions'] += 1
                continue
            dp, dr = residual(solver_fk(q), target)
            s['max_position_error_m'] = max(s['max_position_error_m'], dp)
            s['max_rotation_error_rad'] = max(s['max_rotation_error_rad'], dr)
            if dp > 1e-5 or dr > 1e-5:
                s['invalid_solutions'] += 1
                continue
            # Select an equivalent revolution inside each URDF joint interval.
            wrapped = q + 2*np.pi*np.ceil((limits[:, 0]-q-1e-10)/(2*np.pi))
            if np.any(wrapped < limits[:, 0]-1e-9) or np.any(wrapped > limits[:, 1]+1e-9):
                continue
            dp_original, dr_original = residual(original_fk(wrapped), target)
            if dp_original <= reference['position_tolerance_m'] and dr_original <= reference['rotation_tolerance_rad']:
                solved = True
        s['solved_in_limits'] += int(solved)
        if not solved and len(examples) < 10:
            examples.append({'suite': name, 'case': s['tested'], 'returned': count})

    max_fk = np.zeros(2)
    print('Checking FK and IK against generated and original URDF targets...', flush=True)
    for sample in reference['samples']:
        q = np.array(sample['q'])
        generated = solver_fk(q)
        original = np.array(sample['transform'])
        max_fk = np.maximum(max_fk, residual(generated, original))
        check(generated, 'generated_targets')
        check(original, 'original_urdf_targets')
    # Explicitly include wrist-singular poses; do not hide degeneracy failures.
    if not limits[4, 0] <= 0 <= limits[4, 1]:
        raise RuntimeError('Unexpected joint-five limits')
    for sample in reference['samples'][:101]:
        q = np.array(sample['q'])
        q[4] = 0.0
        check(solver_fk(q), 'wrist_zero_targets')
    for s in stats.values():
        s['success_rate'] = s['solved_in_limits'] / s['tested']
    passed = (max_fk[0] <= reference['position_tolerance_m'] and
              max_fk[1] <= reference['rotation_tolerance_rad'] and
              all(s['success_rate'] >= 0.99 and s['invalid_solutions'] == 0 and s['api_errors'] == 0
                  for s in stats.values()))
    report = {'status': 'passed' if passed else 'failed', 'validation': 'standalone_host_not_openrave',
              'solver_source': str(source), 'solver_sha256': digest(source),
              'header_sha256': digest(work / 'ikfast.h'), 'compile_command': command,
              'original_urdf_sha256': digest(current_urdf),
              'original_srdf_sha256': digest(current_srdf),
              'max_model_position_error_m': float(max_fk[0]),
              'max_model_rotation_error_rad': float(max_fk[1]),
              'suites': stats, 'failure_examples': examples,
              'moveit_configuration_changed': False, 'robot_connected': False}
    (work / 'host_validation.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2), flush=True)
    if not passed:
        raise RuntimeError('Standalone validation failed; KDL remains unchanged')
    (WS / 'outputs/ikfast_host_validated_latest.txt').write_text(str(work) + '\n')
    print('HOST VALIDATION PASSED. Solver recovered; MoveIt plugin installation still pending.')


if __name__ == '__main__':
    main()

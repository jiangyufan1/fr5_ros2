#!/usr/bin/env python3
"""Collect the exact FR5 kinematic model and generator versions; no robot access."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

WS = Path('/data/Projects/fr5_ros2')
CONFIG = WS / 'src/fairino5_v6_moveit2_config'
report = {'workspace': str(WS), 'files': {}, 'commands': {}, 'errors': []}


def read(path):
    if path.is_file() and path.stat().st_size < 1500000:
        data = path.read_bytes()
        report['files'][str(path)] = {'sha256': hashlib.sha256(data).hexdigest(),
                                     'text': data.decode('utf-8')}


def command(name, argv, timeout=10):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        report['commands'][name] = {'returncode': result.returncode,
                                    'stdout': result.stdout[-20000:], 'stderr': result.stderr[-6000:]}
    except (OSError, subprocess.SubprocessError) as exc:
        report['commands'][name] = {'error': str(exc)}


for path in [WS / 'src/fairino_description/urdf/fairino5_v6.urdf',
             WS / 'src/fairino_description/package.xml', CONFIG / 'package.xml', CONFIG / 'CMakeLists.txt']:
    read(path)
for path in sorted((CONFIG / 'config').iterdir()):
    if path.suffix in {'.srdf', '.xacro', '.yaml'}:
        read(path)

try:
    from moveit_configs_utils import MoveItConfigsBuilder
    config = MoveItConfigsBuilder('fairino5_v6_robot', package_name='fairino5_v6_moveit2_config').to_moveit_configs()
    report['expanded_urdf'] = config.robot_description['robot_description']
    report['expanded_srdf'] = config.robot_description_semantic['robot_description_semantic']
    report['kinematics'] = config.robot_description_kinematics
    report['joint_limits'] = config.joint_limits
    root = ET.fromstring(report['expanded_urdf'])
    report['robot_name'] = root.attrib.get('name')
    report['joints'] = [{'name': j.attrib['name'], 'type': j.attrib['type'],
                         'parent': j.find('parent').attrib, 'child': j.find('child').attrib,
                         'origin': {} if j.find('origin') is None else j.find('origin').attrib,
                         'axis': {} if j.find('axis') is None else j.find('axis').attrib}
                        for j in root.findall('joint')]
except Exception as exc:
    report['errors'].append('Model expansion: ' + str(exc))

for root in [Path('/opt/ros/humble/lib/moveit_kinematics'), Path('/opt/ros/humble/share/moveit_kinematics')]:
    if root.is_dir():
        for path in root.rglob('*'):
            if path.is_file() and ('ikfast' in path.name.lower() or
                                  ('templates' in path.parts and path.suffix in {'.cpp', '.h', '.xml', '.txt', '.in'})):
                try: read(path)
                except UnicodeError: pass

report['executables'] = {name: shutil.which(name) for name in ['docker', 'podman', 'openrave', 'openrave0.9.py', 'check_urdf', 'c++', 'colcon']}
report['python'] = sys.version
report['python_modules'] = {name: importlib.util.find_spec(name) is not None for name in ['openravepy', 'sympy', 'numpy']}
command('moveit_version', ['dpkg-query', '-W', '-f=${Package} ${Version}\n', 'ros-humble-moveit-kinematics', 'ros-humble-moveit-core'])
if shutil.which('docker'):
    command('docker_server', ['docker', 'info', '--format', '{{.ServerVersion}}'])
    command('openrave_image', ['docker', 'images', 'personalrobotics/ros-openrave', '--format', '{{.Repository}}:{{.Tag}} {{.ID}}'])
    command('fixed_openrave_image', ['docker', 'images', 'fixed-openrave', '--format', '{{.Repository}}:{{.Tag}} {{.ID}}'])
print(json.dumps(report, ensure_ascii=True, indent=2))

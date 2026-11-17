#!/usr/bin/env python3
"""Build and offline-test the authorized IKFast/KDL combination before activation."""
import ctypes
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from prepare_model import origin

WS = Path('/data/Projects/fr5_ros2')
TOOLS = Path(__file__).resolve().parent
PACKAGE = 'fairino5_ikfast_plugin'
GROUP = 'fairino5_v6_group'
PLUGIN = 'fr5_kinematics/IKFastWithKDL'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_write(path, data):
    # Resolve installed symlinks so their targets are preserved, not replaced.
    path = path.resolve()
    mode = path.stat().st_mode & 0o777
    fd, name = tempfile.mkstemp(dir=str(path.parent), prefix='.ikfast-')
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
        os.chmod(name, mode)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def run(command, **kwargs):
    print('+', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, cwd=WS, **kwargs)


def run_ros(command):
    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'
    run(['bash', '-c', 'set -e; source "$1"; source "$2"; shift 2; exec "$@"',
         '_', '/opt/ros/humble/setup.bash', WS / 'install/setup.bash'] + list(command), env=env)


def make_cases(validation, generation, run_dir):
    reference = json.loads((generation / 'reference.json').read_text())
    bounds = np.array(reference['limits'])
    root = ET.parse(generation / 'original.urdf').getroot()
    transforms = []
    for name in reference['joints']:
        o = root.find("joint[@name='%s']/origin" % name)
        transforms.append(origin(list(map(float, o.get('xyz', '0 0 0').split())),
                                 list(map(float, o.get('rpy', '0 0 0').split()))))
    def fk(q):
        value = np.eye(4)
        for fixed, angle in zip(transforms, q):
            value = value @ fixed @ origin([0, 0, 0], [0, 0, angle])
        return value
    # Load the previously compiled local solver to reproduce its exact
    # wrist-zero targets. The combined plugin must solve the formerly failing set.
    lib = ctypes.CDLL(str(validation / 'libfr5_ikfast_test.so'))
    pointer = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    lib.fr5_fk.argtypes = [pointer, pointer, pointer]
    lib.fr5_fk.restype = ctypes.c_int
    with (run_dir / 'cases.txt').open('w') as stream:
        def emit(suite, q, target, perturb):
            seed = np.clip(q + perturb*np.array([1,-1,1,-1,1,-1]), bounds[:, 0], bounds[:, 1])
            values = np.r_[seed, target[:3, 3], target[:3, :3].ravel()]
            stream.write(suite + ' ' + ' '.join(format(x, '.17g') for x in values) + '\n')
        for sample in reference['samples']:
            emit('regular', np.array(sample['q']), np.array(sample['transform']), 0.03)
        for sample in reference['samples'][:101]:
            q = np.array(sample['q']); q[4] = 0.0
            t, r = np.zeros(3), np.zeros(9)
            if lib.fr5_fk(np.ascontiguousarray(q), t, r) != 0:
                raise RuntimeError('Saved solver FK failed')
            target = np.eye(4); target[:3, 3] = t; target[:3, :3] = r.reshape(3, 3)
            emit('wrist_zero', q, target, 0.001)
        for i, sample in enumerate(reference['samples'][1:109]):
            q = np.array(sample['q'])
            joint, side = i % 6, (i // 6) % 2
            q[joint] = bounds[joint, side] + (1e-4 if side == 0 else -1e-4)
            emit('near_limits', q, fk(q), 0.01)


def create_staged_package(generator, args, stage):
    # ament accepts package names, not filesystem paths. Force only this target
    # to be treated as a new package, even if an older version is installed.
    # Other lookups (e.g. MoveIt's templates) retain the real ament behavior.
    stage = stage.resolve()
    if stage.name != PACKAGE or args.ikfast_plugin_pkg != PACKAGE:
        raise RuntimeError('Unexpected staging package name')
    old_directory = Path.cwd()
    original_lookup = generator.get_package_share_directory
    def staging_lookup(name):
        if name == PACKAGE:
            raise generator.PackageNotFoundError(name)
        return original_lookup(name)
    try:
        os.chdir(stage.parent)
        generator.get_package_share_directory = staging_lookup
        generator.create_ikfast_package(args)
        if Path(args.ikfast_plugin_pkg_path).resolve() != stage:
            raise RuntimeError('Generator selected a path outside staging')
    finally:
        generator.get_package_share_directory = original_lookup
        os.chdir(old_directory)


def make_package(validation, stage):
    generator_path = Path('/opt/ros/humble/lib/moveit_kinematics/create_ikfast_moveit_plugin.py')
    spec = importlib.util.spec_from_file_location('fr5_official_ikfast_generator', generator_path)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    args = generator.create_parser().parse_args([
        'fairino5_v6_robot', GROUP, PACKAGE, 'base_link', 'wrist3_link',
        str(validation / 'fr5_ikfast_solver.cpp')])
    generator.populate_optional(args)
    generator.validate_openrave_version(args)
    create_staged_package(generator, args, stage)
    generator.update_ikfast_package(args)
    # Deliberately do not call update_moveit_package: switch only after testing.
    for filename, subdirectory in [('hybrid_plugin.hpp', 'include'),
                                   ('hybrid_plugin.cpp', 'src'), ('validate_hybrid.cpp', 'src')]:
        shutil.copy2(TOOLS / filename, stage / subdirectory / filename)
    (stage / '.fr5_hybrid_managed').write_text('fr5-ikfast-kdl-v1\n')
    (stage / 'fr5_hybrid_plugin.xml').write_text('''<library path="fr5_hybrid_kinematics">
  <class name="fr5_kinematics/IKFastWithKDL" type="fr5_kinematics::IKFastWithKDL" base_class_type="kinematics::KinematicsBase">
    <description>FR5 IKFast with KDL fallback and original-model validation.</description>
  </class>
</library>
''')
    cmake = stage / 'CMakeLists.txt'
    addition = '''
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
add_library(fr5_hybrid_kinematics SHARED src/hybrid_plugin.cpp)
ament_target_dependencies(fr5_hybrid_kinematics moveit_core pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_core fr5_hybrid_plugin.xml)
install(TARGETS fr5_hybrid_kinematics LIBRARY DESTINATION lib)
add_executable(fr5_validate_hybrid src/validate_hybrid.cpp)
ament_target_dependencies(fr5_validate_hybrid moveit_core pluginlib rclcpp urdf srdfdom)
add_dependencies(fr5_validate_hybrid fr5_hybrid_kinematics)
install(TARGETS fr5_validate_hybrid RUNTIME DESTINATION lib/${PROJECT_NAME})
'''
    text = cmake.read_text()
    if text.count('ament_package()') != 1:
        raise RuntimeError('Unexpected official CMake template')
    cmake.write_text(text.replace('ament_package()', addition + '\nament_package()'))
    manifest_path = stage / 'package.xml'
    tree = ET.parse(manifest_path)
    for name in ['urdf', 'srdfdom']:
        ET.SubElement(tree.getroot(), 'depend').text = name
    ET.SubElement(tree.getroot(), 'exec_depend').text = 'moveit_kinematics'
    tree.write(manifest_path, encoding='utf-8', xml_declaration=True)


def main():
    if len(sys.argv) != 2:
        raise RuntimeError('Pass the host_validation directory printed by the recovery script')
    validation = Path(sys.argv[1]).resolve()
    validation.relative_to((WS / 'outputs').resolve())
    generation = validation.parent
    previous = json.loads((validation / 'host_validation.json').read_text())
    for suite in ('generated_targets', 'original_urdf_targets'):
        s = previous['suites'][suite]
        if s['tested'] < 1000 or s['success_rate'] < 0.99 or s['invalid_solutions'] or s['api_errors']:
            raise RuntimeError('Ordinary IKFast validation must pass before combination')
    if digest(validation / 'fr5_ikfast_solver.cpp') != previous['solver_sha256']:
        raise RuntimeError('Solver changed since validation')
    for filename, key, current in [
        ('original.urdf', 'original_urdf_sha256', WS / 'src/fairino_description/urdf/fairino5_v6.urdf'),
        ('original.srdf', 'original_srdf_sha256', WS / 'src/fairino5_v6_moveit2_config/config/fairino5_v6_robot.srdf')]:
        if digest(current) != previous[key] or digest(generation / filename) != previous[key]:
            raise RuntimeError('Model changed since validation')
    config = WS / 'src/fairino5_v6_moveit2_config/config/kinematics.yaml'
    before = config.read_bytes()
    settings = yaml.safe_load(before)
    if settings[GROUP]['kinematics_solver'] not in ('kdl_kinematics_plugin/KDLKinematicsPlugin', PLUGIN):
        raise RuntimeError('Unexpected active solver; refusing to overwrite configuration')
    run_dir = WS / 'outputs' / ('ikfast_hybrid_' + time.strftime('%Y%m%d_%H%M%S'))
    run_dir.mkdir(exist_ok=False)
    (run_dir / 'COLCON_IGNORE').touch()
    (run_dir / 'kinematics.source.before.yaml').write_bytes(before)
    from ament_index_python.packages import get_package_share_directory
    installed = Path(get_package_share_directory('fairino5_v6_moveit2_config')) / 'config/kinematics.yaml'
    installed_before = installed.read_bytes()
    (run_dir / 'kinematics.installed.before.yaml').write_bytes(installed_before)
    print('Installation record:', run_dir, flush=True)
    make_cases(validation, generation, run_dir)
    stage = run_dir / PACKAGE
    make_package(validation, stage)
    destination = WS / 'src' / PACKAGE
    if destination.exists():
        if destination.is_symlink() or not (destination / '.fr5_hybrid_managed').is_file():
            raise RuntimeError('Existing unmanaged plugin package; refusing overwrite')
        shutil.move(str(destination), str(run_dir / 'previous_plugin'))
    shutil.move(str(stage), str(destination))
    run(['colcon', 'build', '--base-paths', 'src', '--packages-select', PACKAGE,
         '--cmake-args', '-DCMAKE_BUILD_TYPE=Release'])
    report_path = run_dir / 'hybrid_validation.json'
    run_ros(['ros2', 'run', PACKAGE, 'fr5_validate_hybrid',
             generation / 'original.urdf', generation / 'original.srdf', run_dir / 'cases.txt', report_path])
    report = json.loads(report_path.read_text())
    if report.get('passed') is not True:
        raise RuntimeError('Combined plugin failed validation; KDL is unchanged')
    if config.read_bytes() != before or installed.read_bytes() != installed_before:
        raise RuntimeError('Kinematics configuration changed during build; refusing to overwrite')
    settings[GROUP]['kinematics_solver'] = PLUGIN
    settings[GROUP]['kinematics_solver_timeout'] = 0.05
    after = yaml.safe_dump(settings, sort_keys=False).encode()
    try:
        atomic_write(config, after)
        run(['colcon', 'build', '--base-paths', 'src', '--packages-select', 'fairino5_v6_moveit2_config'])
        verify = """from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml
group='fairino5_v6_group'; expected='fr5_kinematics/IKFastWithKDL'
config=MoveItConfigsBuilder('fairino5_v6_robot', package_name='fairino5_v6_moveit2_config').to_moveit_configs()
assert config.robot_description_kinematics['robot_description_kinematics'][group]['kinematics_solver']==expected
path=Path(get_package_share_directory('fairino5_v6_moveit2_config'))/'config/kinematics.yaml'
assert yaml.safe_load(path.read_text())[group]['kinematics_solver']==expected
print('Installed MoveIt configuration verified: '+expected)
"""
        run_ros(['/usr/bin/python3', '-c', verify])
    except BaseException:
        atomic_write(config, before)
        if installed.resolve() != config.resolve():
            atomic_write(installed, installed_before)
        print('Configuration rolled back to the previous solver.', flush=True)
        raise
    record = {'status': 'complete', 'plugin': PLUGIN, 'package': str(destination),
              'solver_sha256': previous['solver_sha256'], 'source_validation': str(validation),
              'hybrid_validation': str(report_path), 'configuration': str(config),
              'previous_config': str(run_dir / 'kinematics.source.before.yaml'),
              'installed_previous_config': str(run_dir / 'kinematics.installed.before.yaml'),
              'timeout_sec': 0.05, 'robot_connected': False,
              'configuration_sha256': hashlib.sha256(after).hexdigest()}
    (run_dir / 'installation.json').write_text(json.dumps(record, indent=2))
    (WS / 'outputs/ikfast_hybrid_latest.txt').write_text(str(run_dir) + '\n')
    doc = WS / 'docs/IKFAST.md'
    doc.write_text('''# FR5 inverse kinematics

MoveIt uses `fr5_kinematics/IKFastWithKDL` from `fairino5_ikfast_plugin`.
IKFast is tried first. When no candidate passes bounds, consistency, original
URDF residual checks and the caller callback, KDL receives the remaining query
time. The configured query budget is 0.05 seconds total. No approximate solutions
are accepted. Collision rejection callbacks apply to both backends.

The raw IKFast solver has no solutions for the validated wrist-zero target set;
KDL fallback is intentional. The generation-only model snaps near-quarter-turn
angles; the original URDF remains unchanged. Both backends are checked against
the original model with 20 micrometre / 20 microradian residual thresholds.

Validation loads the actual installed plugin and tests 1001 ordinary targets,
101 wrist-zero targets and 108 near-limit targets with perturbed seeds. It also
checks callback rejection, consistency, invalid inputs and unreachable targets.
These tests do not establish full workspace coverage or trajectory continuity.
No hardware connection or motion test was performed during installation.

Run records and the previous kinematics configurations: ''' + str(run_dir) + '\n')
    architecture = WS / 'docs/ARCHITECTURE.md'
    if architecture.is_file() and '[IKFAST.md](IKFAST.md)' not in architecture.read_text():
        with architecture.open('a') as stream:
            stream.write('\n## Inverse kinematics\n\nIKFast is primary with KDL fallback. '
                         'See [IKFAST.md](IKFAST.md) for validation, limits and installation records.\n')
    print('IKFAST + KDL INSTALLED AND OFFLINE VALIDATED. No robot connection was attempted.', flush=True)
    print('Record:', run_dir / 'installation.json', flush=True)


if __name__ == '__main__':
    with (WS / 'outputs/.ikfast_hybrid_install.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        main()

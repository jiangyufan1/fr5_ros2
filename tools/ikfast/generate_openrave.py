# Python 2/3 compatible for the documented personalrobotics OpenRAVE container.
from __future__ import print_function
import json
import logging
import math
import shutil
import threading
import time
import numpy as np
import openravepy as rave
import openravepy.databases.inversekinematics as inversekinematics
import sympy
from ikfast_compat import install_and_test

logging.basicConfig(filename='/work/ikfast_details.log', level=logging.INFO,
                    format='%(asctime)s %(name)s %(levelname)s %(message)s')
console = logging.StreamHandler()
console.setLevel(logging.WARNING)
logging.getLogger().addHandler(console)

print('OpenRAVE:', rave.__version__, 'SymPy:', sympy.__version__)
if sympy.__version__ != '0.7.1':
    raise RuntimeError('Expected isolated SymPy 0.7.1')
reference = json.load(open('/work/reference.json'))
environment = rave.Environment()
if not environment.Load('/work/robot.xml'):
    raise RuntimeError('OpenRAVE model load failed')
robot = environment.GetRobots()[0]
manip = robot.SetActiveManipulator('fairino5_v6_group')
indices = list(manip.GetArmIndices())
names = [robot.GetJointFromDOFIndex(int(i)).GetName() for i in indices]
if names != reference['joints'] or robot.GetDOF() != 6:
    raise RuntimeError('Unexpected OpenRAVE joint order: ' + repr(names))
low, high = robot.GetDOFLimits(indices)
if not np.allclose(np.array(reference['limits']), np.array([low, high]).T, atol=1e-9, rtol=0):
    raise RuntimeError('OpenRAVE joint limits differ from URDF')
max_position, max_rotation = 0.0, 0.0
for sample in reference['samples']:
    robot.SetDOFValues(sample['q'], indices)
    actual = np.dot(np.linalg.inv(manip.GetBase().GetTransform()), manip.GetEndEffectorTransform())
    expected = np.array(sample['transform'])
    position = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    rotation = math.acos(float(np.clip((np.trace(np.dot(actual[:3, :3].T, expected[:3, :3]))-1)/2, -1, 1)))
    max_position, max_rotation = max(max_position, position), max(max_rotation, rotation)
if max_position > reference['position_tolerance_m'] or max_rotation > reference['rotation_tolerance_rad']:
    raise RuntimeError('Model conversion differs from URDF: position=%g rotation=%g' % (max_position, max_rotation))
print('Model FK validation passed:', max_position, 'm;', max_rotation, 'rad')
robot.SetDOFValues([0]*6, indices)
model = inversekinematics.InverseKinematicsModel(robot, iktype=rave.IkParameterization.Type.Transform6D)
compatibility = install_and_test(model.ikfast, sympy)
with open('/work/compatibility_report.json', 'w') as stream:
    json.dump(compatibility, stream, indent=2)
print('Compatibility regression passed:', compatibility['regression_cases'], 'cases; original installed libraries unchanged.')

finished = threading.Event()
started = time.time()
def heartbeat():
    while not finished.wait(60):
        print('IKFast working: %d seconds; details: /work/ikfast_details.log' % (time.time()-started))
monitor = threading.Thread(target=heartbeat)
monitor.daemon = True
monitor.start()
if not model.load():
    print('Generating analytical IK solver...')
    model.autogenerate()
    model.save()
print('Running 1000 IK self-tests...')
success_rate, wrong_rate = model.testik('1000')
finished.set()
report = {'sample_count': len(reference['samples']), 'max_model_position_error_m': max_position,
          'max_model_rotation_error_rad': max_rotation, 'ikfast_success_rate': float(success_rate),
          'ikfast_wrong_rate': float(wrong_rate), 'compatibility': compatibility}
with open('/work/generation_result.json', 'w') as stream:
    json.dump(report, stream, indent=2)
if wrong_rate != 0 or success_rate < 0.99:
    raise RuntimeError('IKFast self-test did not pass; KDL remains unchanged')
shutil.copyfile(model.getsourcefilename(), '/work/fr5_ikfast_solver.cpp')
print('SUCCESS: fr5_ikfast_solver.cpp generated and self-tested. KDL remains unchanged.')

"""Opt-in real FR5 MoveIt launch. Keeps the installed mock demo unchanged."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def live_description(xml, planning_limits, robot_ip):
    root = ET.fromstring(xml)
    systems = root.findall('ros2_control')
    if len(systems) != 1:
        raise RuntimeError('Expected exactly one ros2_control system')
    old = systems[0]
    if old.findtext('hardware/plugin') != 'mock_components/GenericSystem':
        raise RuntimeError('Expected original mock model; refusing to replace an unknown hardware plugin')
    root.remove(old)
    system = ET.SubElement(root, 'ros2_control', name='FR5MoveItLive', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'fairino_hardware/FairinoMoveItHardware'
    ET.SubElement(hardware, 'param', name='robot_ip').text = robot_ip
    ET.SubElement(hardware, 'param', name='enable_motion').text = 'true'
    model_joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    for name in ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']:
        model = model_joints[name]
        if model.attrib.get('type') != 'revolute':
            raise RuntimeError(f'{name}: expected a bounded revolute joint')
        limit = model.find('limit')
        lower, upper, velocity = (float(limit.attrib[k]) for k in ['lower', 'upper', 'velocity'])
        override = planning_limits.get(name, {})
        if override.get('has_velocity_limits', False):
            value = float(override['max_velocity'])
            if not math.isfinite(value) or value <= 0:
                raise RuntimeError(f'{name}: invalid MoveIt velocity limit')
            velocity = min(velocity, value)
        if not all(math.isfinite(v) for v in [lower, upper, velocity]) or lower >= upper or velocity <= 0:
            raise RuntimeError(f'{name}: invalid model limits')
        joint = ET.SubElement(system, 'joint', name=name)
        for key, value in [('min_position', lower), ('max_position', upper), ('max_velocity', velocity)]:
            ET.SubElement(joint, 'param', name=key).text = repr(value)
        command = ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(command, 'param', name='min').text = repr(lower)
        ET.SubElement(command, 'param', name='max').text = repr(upper)
        ET.SubElement(joint, 'state_interface', name='position')
    return ET.tostring(root, encoding='unicode')


def setup(context):
    if LaunchConfiguration('enable_motion').perform(context).lower() != 'true':
        raise RuntimeError('Real hardware requires explicit enable_motion:=true. Use the original demo for simulation.')
    ip = LaunchConfiguration('robot_ip').perform(context)
    config = MoveItConfigsBuilder('fairino5_v6_robot', package_name='fairino5_v6_moveit2_config').to_moveit_configs()
    xml = live_description(config.robot_description['robot_description'],
                           config.joint_limits.get('robot_description_planning', {}).get('joint_limits', {}), ip)
    config.robot_description = {'robot_description': xml}
    # Use an unambiguous controller configuration independent of duplicate YAML keys.
    config.trajectory_execution = {
        'moveit_manage_controllers': False,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['fairino5_controller'],
            'fairino5_controller': {
                'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': [f'j{i}' for i in range(1, 7)]}},
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 3.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    description = {'robot_description': ParameterValue(xml, value_type=str), 'use_sim_time': False}
    control_file = Path(__file__).with_name('fr5_moveit_live_controllers.yaml')
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   output='screen', parameters=[str(control_file), description])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen', parameters=[description])
    broadcaster = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                                  '--controller-manager-timeout', '60'])
    trajectory = Node(package='controller_manager', executable='spawner', output='screen',
                      arguments=['fairino5_controller', '-c', '/controller_manager',
                                 '--controller-manager-timeout', '60'])
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[config.to_dict(), {'use_sim_time': False,
                        'publish_robot_description_semantic': True,
                        'publish_planning_scene': True,
                        'publish_geometry_updates': True,
                        'publish_state_updates': True,
                        'publish_transforms_updates': True}])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', str(config.package_path / 'config/moveit.rviz')],
                parameters=[description, config.robot_description_semantic,
                            config.robot_description_kinematics, config.planning_pipelines,
                            config.joint_limits])
    transforms = []
    srdf = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    for joint in srdf.findall('virtual_joint'):
        if joint.attrib['type'] != 'fixed':
            raise RuntimeError('Only fixed-base virtual joints are supported')
        transforms.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='fr5_live_virtual_joint_' + str(len(transforms)),
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', joint.attrib['parent_frame'], '--child-frame-id', joint.attrib['child_link']]))

    def after_success(actions, stage):
        def callback(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=f'{stage} failed; inspect terminal errors'))]
            return actions
        return callback

    return [
        LogInfo(msg=f'REAL FR5 at {ip}. Stop other drivers/demos before launching. Targets come from MoveIt.'),
        RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='Hardware controller manager exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
            on_exit=after_success([trajectory], 'joint_state_broadcaster'))),
        RegisterEventHandler(OnProcessExit(target_action=trajectory,
            on_exit=after_success([move_group, rviz], 'fairino5_controller'))),
        rsp, *transforms, manager, broadcaster,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.57.2'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        OpaqueFunction(function=setup),
    ])

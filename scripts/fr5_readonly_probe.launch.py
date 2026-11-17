"""Read-only SDK/ros2_control probe; no motion controller or command interfaces."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    ip = LaunchConfiguration('robot_ip').perform(context)
    model = Path(get_package_share_directory('fairino_description')) / 'urdf/fairino5_v6.urdf'
    root = ET.parse(model).getroot()
    if root.findall('ros2_control'):
        raise RuntimeError('Base URDF already contains ros2_control; inspect before proceeding')
    names = {j.attrib.get('name') for j in root.findall('joint')}
    if not {f'j{i}' for i in range(1, 7)}.issubset(names):
        raise RuntimeError('Expected joints j1 through j6 in the base URDF')
    system = ET.SubElement(root, 'ros2_control', name='FR5ReadOnlyProbe', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'fairino_hardware/FairinoReadOnlyProbe'
    ET.SubElement(hardware, 'param', name='robot_ip').text = ip
    for i in range(1, 7):
        joint = ET.SubElement(system, 'joint', name=f'j{i}')
        ET.SubElement(joint, 'state_interface', name='position')
    description = ParameterValue(ET.tostring(root, encoding='unicode'), value_type=str)
    return [
        Node(package='controller_manager', executable='ros2_control_node',
             namespace='fr5_probe', name='controller_manager', output='screen',
             parameters=[{'robot_description': description, 'update_rate': 100,
                          'use_sim_time': False,
                          'joint_state_broadcaster.type':
                          'joint_state_broadcaster/JointStateBroadcaster'}]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', '--controller-manager',
                        '/fr5_probe/controller_manager', '--controller-manager-timeout', '30']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.57.2'),
        OpaqueFunction(function=setup),
    ])

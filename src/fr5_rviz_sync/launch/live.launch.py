"""Read-only model display; the already-running official driver supplies state."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from fr5_rviz_sync.model import load_live_description


def generate_launch_description():
    package = Path(get_package_share_directory('fr5_rviz_sync'))
    description = Path(get_package_share_directory('fairino_description'))
    robot_description = load_live_description(description / 'urdf/fairino5_v6.urdf')
    # Separate TF topics prevent an offline simulation publisher from replacing
    # the live transforms in this viewer. Frame names stay native to the URDF.
    tf_remaps = [('/tf', '/fr5_live/tf'), ('/tf_static', '/fr5_live/tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('input_topic', default_value='/nonrt_state_data'),
        Node(package='fr5_rviz_sync', executable='joint_state_bridge',
             output='screen', parameters=[{
                 'use_sim_time': False,
                 'input_topic': LaunchConfiguration('input_topic'),
                 'publish_rate_hz': 50.0, 'stale_timeout_sec': 2.0,
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace='fr5_live', output='screen',
             remappings=tf_remaps,
             parameters=[{
                 'robot_description': ParameterValue(robot_description, value_type=str),
                 'publish_frequency': 50.0, 'use_sim_time': False,
             }]),
        Node(package='rviz2', executable='rviz2', name='rviz2', namespace='fr5_live',
             output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
             remappings=tf_remaps,
             parameters=[{'use_sim_time': False}],
             arguments=['-d', str(package / 'config/live.rviz')]),
    ])

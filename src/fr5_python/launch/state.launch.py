from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = str(Path(get_package_share_directory('fr5_python')) / 'config/state.yaml')
    return LaunchDescription([
        Node(package='fr5_python', executable='state_echo', output='screen', parameters=[config]),
    ])

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('wheel_human_commander'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', description='Same feedforward motor YAML used by hardware'),
        Node(package='wheel_human_commander', executable='commander_node',
             name='wheel_human_commander', output='screen',
             parameters=[str(share / 'config/commander.yaml'),
                         {'config_file': LaunchConfiguration('config_file')}]),
    ])

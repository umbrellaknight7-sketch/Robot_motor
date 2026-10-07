"""Convenience preset for the installed CAD; queries still require explicit states."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    commander = Path(get_package_share_directory('wheel_human_commander'))
    description = Path(get_package_share_directory('wheel_human_description'))
    return LaunchDescription([
        DeclareLaunchArgument('kinematics_config', default_value=str(commander / 'config/kinematics.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(commander / 'launch/kinematics.launch.py')),
            launch_arguments={
                'model_file': str(description / 'urdf/kinematics_fixed.urdf'),
                'kinematics_config': LaunchConfiguration('kinematics_config'),
            }.items()),
    ])

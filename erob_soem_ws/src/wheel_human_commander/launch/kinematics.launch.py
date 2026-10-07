"""Explicit-model FK/IK queries only; no controllers, SDK, or state sources."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def nodes(context):
    model = Path(LaunchConfiguration('model_file').perform(context)).resolve(strict=True)
    description = xacro.process_file(str(model)).toxml()
    return [Node(
        package='wheel_human_commander', executable='kinematics_node',
        name='wheel_human_kinematics', output='screen',
        parameters=[LaunchConfiguration('kinematics_config').perform(context),
                    {'robot_description': ParameterValue(description, value_type=str)}],
    )]


def generate_launch_description():
    share = Path(get_package_share_directory('wheel_human_commander'))
    return LaunchDescription([
        DeclareLaunchArgument('model_file', description='Explicit geometry URDF/Xacro with both hand TCPs'),
        DeclareLaunchArgument('kinematics_config', default_value=str(share / 'config/kinematics.yaml')),
        OpaqueFunction(function=nodes),
    ])

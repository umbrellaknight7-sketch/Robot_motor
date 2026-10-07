"""Packaged CAD display and explicit initial pose; no controllers or SDK."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wheel_human_description.poses import display_description, load_pose, zero_pose


def nodes(context):
    share = Path(get_package_share_directory('wheel_human_description'))
    description = (share / 'urdf/kinematics_fixed.urdf').read_text()
    initial = LaunchConfiguration('initial_pose').perform(context)
    positions = (load_pose(share / 'config/poses/upright.yaml', description)
                 if initial == 'upright' else zero_pose(description))

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(display_description(description), value_type=str)}],
             output='screen'),
        Node(package='wheel_human_description' if gui else 'joint_state_publisher',
             executable='cad_joint_state_publisher_gui.py' if gui else 'joint_state_publisher',
             parameters=[{'rate': 20, **{'zeros.' + name: value for name, value in positions.items()}}],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/cad.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('initial_pose', default_value='upright', choices=['upright', 'zero']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])

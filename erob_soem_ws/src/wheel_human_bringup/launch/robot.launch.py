from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wheel_human_bringup.model import build_description


def launch_nodes(context):
    share = Path(get_package_share_directory('wheel_human_bringup'))
    description_share = Path(get_package_share_directory('wheel_human_description'))
    backend = LaunchConfiguration('backend').perform(context)

    # Validate the composed model and motor mapping before constructing any ROS nodes.
    description = build_description(
        share, description_share, backend=backend,
        config_file=LaunchConfiguration('config_file').perform(context),
        model_file=LaunchConfiguration('model_file').perform(context),
        enable_mock_fault_injection=IfCondition(
            LaunchConfiguration('enable_mock_fault_injection')).evaluate(context),
    )
    initial = {'hardware_components_initial_state': {'inactive': ['ErobSystem']}} if backend == 'soem' else {}
    nodes = [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': description}, str(share / 'config/controllers.yaml'), initial],
             output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(description_share / 'rviz/display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    if backend == 'mock':
        nodes += [
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['position_controller', '--inactive', '--controller-manager-timeout', '30']),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mock', choices=['mock', 'soem']),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('model_file', default_value=''),
        DeclareLaunchArgument('enable_mock_fault_injection', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])

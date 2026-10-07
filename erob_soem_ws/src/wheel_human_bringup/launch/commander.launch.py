from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from wheel_human_bringup.model import build_description


def launch_nodes(context):
    share = Path(get_package_share_directory('wheel_human_bringup'))
    description_share = Path(get_package_share_directory('wheel_human_description'))
    backend = LaunchConfiguration('backend').perform(context)
    wrist_mode = LaunchConfiguration('wrist_mode').perform(context)
    wrist_config = LaunchConfiguration('wrist_config').perform(context)
    if wrist_mode == 'real' and not wrist_config:
        raise ValueError('Real wrists require an explicit calibrated wrist_config')
    description = build_description(
        share, description_share, backend=backend, wrists_enabled=wrist_mode != 'disabled',
        config_file=LaunchConfiguration('config_file').perform(context),
        model_file=LaunchConfiguration('model_file').perform(context),
        controllers_file=str(share / 'config/controllers_feedforward.yaml'),
        enable_mock_fault_injection=IfCondition(
            LaunchConfiguration('enable_mock_fault_injection')).evaluate(context),
    )
    if wrist_mode == 'real' and (not LaunchConfiguration('model_file').perform(context) or 'SOFTWARE-ONLY PLACEHOLDER MODEL' in description):
        raise ValueError('Real wrists require an explicit calibrated model_file')

    # Real EtherCAT hardware starts inactive; activation is a separate operation.
    initial = {'hardware_components_initial_state': {'inactive': ['ErobSystem']}} if backend == 'soem' else {}
    commander_share = Path(get_package_share_directory('wheel_human_commander'))
    nodes = [
        Node(package='wheel_human_commander', executable='commander_node',
             parameters=[str(commander_share / 'config/commander.yaml'),
                         {'config_file': LaunchConfiguration('config_file').perform(context), 'wrists_enabled': wrist_mode != 'disabled'}],
             output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': description}, ParameterFile(str(share / 'config/controllers_feedforward.yaml'), allow_substs=True), initial],
             output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(description_share / 'rviz/display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]

    if wrist_mode != 'disabled':
        adapter_share = Path(get_package_share_directory('wheel_human_wrist_adapter'))
        nodes.append(Node(package='wheel_human_wrist_adapter', executable='wrist_adapter',
                          parameters=[wrist_config or str(adapter_share / 'config/mock.yaml'),
                                      {'mode': wrist_mode}], output='screen'))
        # Real SDK bus owners are started separately with commissioned hand/bus
        # settings and explicit hand activation. Never auto-open or enable CAN.
        if wrist_mode == 'mock':
            for side in ('left', 'right'):
                nodes.append(Node(package='linker_hand_l30_v6', executable='wrist_bridge',
                                  name='linker_' + side + '_bridge',
                                  parameters=[{'mock': True, 'hand_type': side}], output='screen'))

    # Spawn the example controllers only for the mock backend.
    if backend == 'mock':
        nodes += [
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['csp_controller', '--inactive', '--controller-manager-timeout', '30']),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wrist_mode', default_value='disabled', choices=['disabled', 'mock', 'real']),
        DeclareLaunchArgument('wrist_config', default_value=''),
        DeclareLaunchArgument('backend', default_value='mock', choices=['mock', 'soem']),
        DeclareLaunchArgument('config_file', default_value=str(Path(get_package_share_directory('wheel_human_bringup')) / 'config/robot_mock_feedforward.yaml')),
        DeclareLaunchArgument('model_file', default_value=''),
        DeclareLaunchArgument('enable_mock_fault_injection', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])

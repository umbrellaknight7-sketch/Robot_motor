from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_nodes(context):
    share = Path(get_package_share_directory('erob_bringup'))
    backend = LaunchConfiguration('backend').perform(context)
    config = LaunchConfiguration('config_file').perform(context)
    if backend not in ('mock', 'soem'):
        raise RuntimeError('backend must be mock or soem')
    if not config:
        if backend == 'soem':
            raise RuntimeError('Real hardware requires an explicit verified config_file')
        config = str(share / 'config/robot_mock.yaml')
    description = xacro.process_file(str(share / 'urdf/erob_15.urdf.xacro'),
                                    mappings={'backend': backend, 'config_file': config,
                                              'enable_mock_fault_injection': LaunchConfiguration('enable_mock_fault_injection').perform(context)}).toxml()
    initial = {'hardware_components_initial_state': {'inactive': ['ErobSystem']}} if backend == 'soem' else {}
    nodes = [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': description}, str(share / 'config/controllers.yaml'), initial],
             output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
    ]
    if backend == 'mock':
        nodes += [Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
                  Node(package='controller_manager', executable='spawner',
                       arguments=['position_controller',
                                  '--inactive', '--controller-manager-timeout', '30'])]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mock'),
        DeclareLaunchArgument('enable_mock_fault_injection', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=''),
        OpaqueFunction(function=launch_nodes),
    ])

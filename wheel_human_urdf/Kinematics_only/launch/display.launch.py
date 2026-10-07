"""Display only: robot state, simulated joint sliders and RViz; no controllers."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = Path(get_package_share_directory('kinematics_only'))
    model = Path(LaunchConfiguration('model').perform(context))
    robot = ET.fromstring(model.read_text())
    preview = LaunchConfiguration('preview_limits').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    notices = []
    if preview:
        for joint in robot.findall('joint'):
            if joint.get('type') != 'revolute':
                continue
            limit = joint.find('limit')
            if limit is not None and float(limit.get('lower')) == float(limit.get('upper')):
                extent = math.pi if 'Wheel' in joint.get('name') else math.pi/2
                limit.set('lower', str(-extent))
                limit.set('upper', str(extent))
        notices.append(LogInfo(msg='VISUALIZATION ONLY: zero-range joint limits replaced in memory by +/-90 degrees (wheels +/-180). These are NOT hardware limits.'))
    # KDL cannot represent root-link inertia. Omit it only from the display copy.
    children = {j.find('child').get('link') for j in robot.findall('joint')}
    for link in robot.findall('link'):
        if link.get('name') not in children:
            for inertia in link.findall('inertial'):
                link.remove(inertia)
    description = ET.tostring(robot, encoding='unicode')
    return notices + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui' if gui else 'joint_state_publisher',
             executable='joint_state_publisher_gui' if gui else 'joint_state_publisher',
             parameters=[{'robot_description': description, 'rate': 20}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share/'rviz/model.rviz')], output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('kinematics_only'))
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(share/'urdf/Kinematics_only.urdf')),
        DeclareLaunchArgument('gui', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('preview_limits', default_value='true', choices=['true','false']),
        OpaqueFunction(function=nodes),
    ])

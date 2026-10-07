#!/usr/bin/env python3
"""Installed CAD/reference-pose acceptance in isolated ROS domain 189."""

import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from wheel_human_description.poses import load_pose, zero_pose
from wheel_human_interfaces.srv import ComputeFK
from workspace_paths import test_log_directory


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)


class Observer(Node):
    def __init__(self):
        super().__init__('cad_description_acceptance')
        self.positions = {}
        self.description = ''
        self.stamp = 0
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(String, '/robot_description', self.robot,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = self.create_client(ComputeFK, '/wheel_human_kinematics/compute_fk')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def joints(self, message):
        self.positions = dict(zip(message.name, message.position))
        self.stamp = Time.from_msg(message.header.stamp).nanoseconds

    def robot(self, message):
        self.description = message.data

    def wait(self, condition):
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if condition():
                return
        raise AssertionError('Timed out waiting for CAD display/TF')

    def fk(self, positions, tip):
        assert self.client.wait_for_service(timeout_sec=20)
        request = ComputeFK.Request(configuration=JointState(name=list(positions), position=list(positions.values())), target_frame=tip)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        assert future.done()
        result = future.result()
        assert result.error_code == 0, result.message
        return result.pose.pose


def main():
    logs = test_log_directory('cad_description')
    os.environ.update(ROS_DOMAIN_ID='189', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs / 'ros'))
    share = Path(get_package_share_directory('wheel_human_description'))
    assert 'wheel_human_urdf' not in str(share)
    model = share / 'urdf/kinematics_fixed.urdf'
    description = model.read_text()
    manifest = json.loads((share / 'config/kinematics_fixed.provenance.json').read_text())
    assert hashlib.sha256(model.read_bytes()).hexdigest() == manifest['packaged_urdf_sha256']
    for relative, digest in manifest['meshes_sha256'].items():
        path = share / relative
        assert 'wheel_human_urdf' not in str(path.resolve())
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
    for mesh in ET.fromstring(description).findall('.//mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://wheel_human_description/')
        assert (share / uri.removeprefix('package://wheel_human_description/')).is_file()
    print('PASS: installed CAD/meshes match provenance and resolve entirely inside the workspace install', flush=True)
    processes, handles = [], []
    node = None

    def launch(package, filename, label, *args):
        handle = (logs / (label + '.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(['ros2', 'launch', package, filename, *args], stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    try:
        launch('wheel_human_commander', 'cad_kinematics.launch.py', 'kinematics')
        rclpy.init()
        node = Observer()
        for pose in ('upright', 'zero'):
            expected = load_pose(share / 'config/poses/upright.yaml', description) if pose == 'upright' else zero_pose(description)
            node.positions.clear()
            node.description = ''
            after = node.get_clock().now().nanoseconds
            display = launch('wheel_human_description', 'cad_display.launch.py', pose,
                             'initial_pose:=' + pose, 'gui:=false', 'rviz:=false')
            node.wait(lambda: node.stamp >= after and set(node.positions) == set(expected) and node.description)
            assert node.positions == expected, (pose, node.positions)
            display_xml = ET.fromstring(node.description)
            assert display_xml.get('name') == 'Kinematics_fixed'
            assert display_xml.find("link[@name='base_link']/inertial") is None
            assert ET.fromstring(description).find("link[@name='base_link']/inertial") is not None
            for tip in ('Left_Hand_TCP', 'Right_Hand_TCP'):
                node.wait(lambda: node.buffer.can_transform('base_link', tip, Time()) and
                          Time.from_msg(node.buffer.lookup_transform('base_link', tip, Time()).header.stamp).nanoseconds >= after)
                tf = node.buffer.lookup_transform('base_link', tip, Time()).transform
                calculated = node.fk(expected, tip)
                assert math.dist([getattr(calculated.position, k) for k in 'xyz'],
                                 [getattr(tf.translation, k) for k in 'xyz']) < 1e-8
                dot = sum(getattr(calculated.orientation, k) * getattr(tf.rotation, k) for k in ('x', 'y', 'z', 'w'))
                assert 2 * math.acos(min(1.0, abs(dot))) < 1e-6
            assert not set(node.get_node_names()) & {'controller_manager', 'wheel_human_commander', 'wheel_human_wrist_adapter'}
            print(f'PASS: {pose} publishes exact 19-joint configuration; both TCP FK poses match fresh TF', flush=True)
            if pose == 'upright':
                with (logs / 'reference_client.log').open('w') as client_log:
                    subprocess.run(['ros2', 'run', 'wheel_human_commander', 'reference_pose_example.py'],
                                   stdout=client_log, stderr=subprocess.STDOUT, check=True, timeout=35)
                print('PASS: installed reference-pose client queried both TCPs', flush=True)
            stop(display)
        assert hashlib.sha256(model.read_bytes()).hexdigest() == manifest['packaged_urdf_sha256']
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            stop(process)
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    main()

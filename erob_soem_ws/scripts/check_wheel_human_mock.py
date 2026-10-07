#!/usr/bin/env python3
"""Launch a private mock instance; verify controller startup, all joints and TF."""

import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from workspace_paths import test_log_directory


class Acceptance(Node):
    def __init__(self):
        super().__init__('wheel_human_mock_acceptance')
        self.positions = {}
        self.received = 0.0
        self.frames = set()
        self.create_subscription(JointState, '/joint_states', self.feedback, 10)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)
        self.create_subscription(TFMessage, '/tf_static', self.transforms,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.params = self.create_client(GetParameters, '/controller_manager/get_parameters')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.commands = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

    def feedback(self, msg):
        self.positions = dict(zip(msg.name, msg.position))
        self.received = time.monotonic()

    def transforms(self, msg):
        self.frames.update(transform.child_frame_id for transform in msg.transforms)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('Missing service: ' + client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15)
        if not future.done() or future.exception():
            raise RuntimeError('Service failed: ' + client.srv_name)
        return future.result()

    def wait(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for mock feedback or TF')

    def run(self):
        request = GetParameters.Request()
        request.names = ['robot_description']
        robot = ET.fromstring(self.call(self.params, request).values[0].string_value)
        assert robot.find("ros2_control/hardware/param[@name='backend']").text == 'mock'
        assert len(robot.findall("joint[@type='revolute']")) == 19
        names = [joint.attrib['name'] for joint in robot.findall('ros2_control/joint')]
        assert len(names) == 15 and 'Feet_Joint' in names
        # Body-only bringup supplies no wrist or wheel states. Those movable
        # joints have no TF until a separate state source supplies their angles.
        expected_frames = {joint.find('child').attrib['link'] for joint in robot.findall('joint')
                           if joint.attrib['name'] in names}
        assert len(expected_frames) == 15
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            states = {c.name: c.state for c in self.call(self.controllers, ListControllers.Request()).controller}
            if states.get('joint_state_broadcaster') == 'active' and states.get('position_controller') == 'inactive':
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            raise AssertionError('Expected active broadcaster and inactive position controller')
        self.wait(lambda: set(self.positions) == set(names) and expected_frames <= self.frames)
        print('PASS: 19-joint model, 15 body joint states and 15 body child TF frames', flush=True)
        request = SwitchController.Request()
        request.activate_controllers = ['position_controller']
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        assert self.call(self.switch, request).ok
        self.wait(lambda: self.commands.get_subscription_count() > 0)
        # Distinct signed targets detect swapped joint mappings in the full ROS path.
        targets = {name: (-1 if index % 2 else 1) * (0.02 + index * 0.003)
                   for index, name in enumerate(names)}
        message = JointTrajectory()
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [targets[name] for name in names]
        point.time_from_start.sec = 2
        message.points = [point]
        self.commands.publish(message)
        self.wait(lambda: time.monotonic() - self.received < 0.5 and
                  all(abs(self.positions.get(name, 999) - target) < 0.002
                      for name, target in targets.items()))
        request.activate_controllers = []
        request.deactivate_controllers = ['position_controller']
        assert self.call(self.switch, request).ok
        print('PASS: all 15 distinct trajectory targets and explicit controller activation/deactivation', flush=True)


def main():
    logs = test_log_directory('wheel_human')
    # Keep the acceptance instance separate from the existing generic mock checks.
    os.environ['ROS_DOMAIN_ID'] = '177'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(logs)
    with (logs / 'launch.log').open('w') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'wheel_human_bringup', 'robot.launch.py', 'backend:=mock'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = Acceptance()
            node.run()
        except Exception:
            print((logs / 'launch.log').read_text()[-16000:], file=sys.stderr)
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


if __name__ == '__main__':
    main()

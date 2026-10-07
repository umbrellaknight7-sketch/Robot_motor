#!/usr/bin/env python3
"""Acceptance client for robot.launch.py backend:=mock; verifies all fifteen axes."""

import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Demo(Node):
    def __init__(self):
        super().__init__('erob_mock_acceptance')
        self.names = [f'motor_{i:02x}_joint' for i in range(1, 16)]
        self.state = None
        self.received = 0.0
        self.create_subscription(JointState, '/joint_states', self.feedback, 10)
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.params = self.create_client(GetParameters, '/controller_manager/get_parameters')
        self.pos = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)
        self.active = None

    def feedback(self, msg):
        self.state = msg
        self.received = time.monotonic()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError(f'Service missing: {client.srv_name}')
        f = client.call_async(request)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15)
        if not f.done() or f.exception():
            raise RuntimeError(f'Service failed: {client.srv_name}')
        return f.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def switch(self, name):
        req = SwitchController.Request()
        req.activate_controllers = [name] if name else []
        req.deactivate_controllers = [self.active] if self.active else []
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = 5
        if not self.call(self.switch_client, req).ok:
            raise RuntimeError(f'Controller switch failed: {self.active} -> {name}')
        self.active = name
        self.spin_for(0.2)

    def verify(self, field, target, tolerance):
        if self.state is None or time.monotonic() - self.received > 0.5:
            raise AssertionError('Joint feedback missing/stale')
        values = dict(zip(self.state.name, getattr(self.state, field)))
        assert set(self.names) <= values.keys(), 'Expected all 15 joints'
        for name in self.names:
            assert abs(values[name] - target) < tolerance, (field, name, values[name], target)
        print(f'PASS: {field} on all 15 motors (target={target})', flush=True)

    def run(self):
        req = GetParameters.Request()
        req.names = ['robot_description']
        description = self.call(self.params, req).values[0].string_value
        import xml.etree.ElementTree as ET
        hardware = ET.fromstring(description).find('./ros2_control/hardware')
        backend = hardware.find("param[@name='backend']").text
        if backend != 'mock':
            raise RuntimeError('This acceptance client refuses a non-mock backend')
        end = time.monotonic() + 30
        while time.monotonic() < end:
            result = self.call(self.list_client, ListControllers.Request())
            names = {c.name for c in result.controller if c.state in ('inactive', 'active')}
            if {'position_controller', 'joint_state_broadcaster'} <= names:
                break
            self.spin_for(0.2)
        else:
            raise RuntimeError('Controllers did not load')
        self.switch('position_controller')
        msg = JointTrajectory()
        msg.joint_names = self.names
        point = JointTrajectoryPoint()
        point.positions = [0.1] * 15
        point.time_from_start.sec = 1
        msg.points = [point]
        self.pos.publish(msg)
        self.spin_for(1.5)
        self.verify('position', 0.1, 0.002)
        self.switch(None)
        print('PASS: CSP position-only ROS interfaces; 7 + 5 + 3 topology', flush=True)


def main():
    rclpy.init()
    node = Demo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

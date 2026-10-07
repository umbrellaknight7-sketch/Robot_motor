#!/usr/bin/env python3
"""Mock-only end-to-end timeout, latched hold, controller reset and resume test."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController, ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from workspace_paths import test_log_directory

workspace = Path(__file__).resolve().parents[1]
logs = test_log_directory('hold')
os.environ.update(ROS_DOMAIN_ID='179', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs/'ros'))


class Check(Node):
    def __init__(self):
        super().__init__('erob_hold_acceptance')
        self.state = {}
        self.diag = {}
        self.names = [f'motor_{i:02x}_joint' for i in range(1, 16)]
        self.create_subscription(JointState, '/joint_states', lambda m: self.state.update(zip(m.name, m.position)), 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
        self.actions = ActionClient(self, FollowJointTrajectory, '/position_controller/follow_joint_trajectory')

    def diagnostics(self, msg):
        for s in msg.status:
            if s.name == 'ErobSystem/motor_01_joint':
                self.diag = {v.key: v.value for v in s.values}

    def call(self, typ, path, request):
        client = self.create_client(typ, path)
        assert client.wait_for_service(timeout_sec=20), path
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20)
        assert future.done() and not future.exception(), path
        result = future.result()
        self.destroy_client(client)
        return result

    def wait(self, predicate, seconds=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.01)
            if predicate(): return
        raise AssertionError(('Timed out', self.diag, self.state))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end: rclpy.spin_once(self, timeout_sec=.01)

    def goal(self, target, duration):
        assert self.actions.wait_for_server(timeout_sec=15)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.names
        p = JointTrajectoryPoint()
        p.positions = [target] * 15
        p.velocities = [0.] * 15
        p.time_from_start.sec = duration
        goal.trajectory.points = [p]
        future = self.actions.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        assert future.done() and future.result().accepted
        return future.result()

    def run(self):
        req = GetParameters.Request(); req.names = ['robot_description']
        description = self.call(GetParameters, '/controller_manager/get_parameters', req).values[0].string_value
        import xml.etree.ElementTree as ET
        robot = ET.fromstring(description)
        assert robot.find("./ros2_control/hardware/param[@name='backend']").text == 'mock'
        assert len(robot.findall("./ros2_control/joint/command_interface")) == 15
        assert all(c.attrib['name']=='position' for c in robot.findall("./ros2_control/joint/command_interface"))
        for _ in range(100):
            controllers = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller
            if any(c.name == 'position_controller' and c.state == 'inactive' for c in controllers): break
            self.spin(.1)
        else: raise AssertionError('Controller not loaded')
        req = SwitchController.Request(); req.activate_controllers=['position_controller']; req.strictness=2; req.timeout.sec=5
        assert self.call(SwitchController, '/controller_manager/switch_controller', req).ok
        old_goal = self.goal(.3, 8)
        self.wait(lambda: len(self.state)==15 and self.state[self.names[0]] > .003)
        assert self.call(Trigger, '/ErobSystem/test_expire_commands', Trigger.Request()).success
        self.wait(lambda: self.diag.get('phase_name') == 'holding')
        held = dict(self.state)
        # Continued commands from the old trajectory do not clear the hold or disable drives.
        self.spin(.3)
        assert self.diag['phase_name']=='holding' and int(self.diag['status_word']) & 0x6f == 0x27
        assert all(abs(self.state[n]-held[n])<.0001 for n in self.names)
        assert not self.call(Trigger, '/ErobSystem/resume', Trigger.Request()).success
        # Helper failure is non-destructive: target an absent manager.
        failed = subprocess.run(['ros2','run','erob_bringup','resume_control.py',
                                 '--controller-manager','/absent','--timeout','0.5'], timeout=10)
        assert failed.returncode != 0
        self.spin(.1); assert self.diag['phase_name']=='holding'
        subprocess.run(['ros2','run','erob_bringup','resume_control.py'],check=True,timeout=30)
        self.wait(lambda: self.diag.get('phase_name')=='active')
        result = old_goal.get_result_async()
        rclpy.spin_until_future_complete(self,result,timeout_sec=5)
        assert result.done() and result.result().status == GoalStatus.STATUS_ABORTED
        self.spin(.2)
        assert all(abs(self.state[n]-held[n])<.001 for n in self.names)
        target = self.state[self.names[0]]+.02
        new_goal = self.goal(target, 2)
        result = new_goal.get_result_async()
        rclpy.spin_until_future_complete(self,result,timeout_sec=5)
        assert result.done() and result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.wait(lambda: all(abs(self.state[n]-target)<.0001 for n in self.names))
        print('PASS: mock 15-axis timeout hold, rejection before reset, helper failure, explicit resume, old goal aborted, new trajectory',flush=True)


if __name__ == '__main__':
    # Isolate host scheduling jitter from deterministic watchdog boundary tests.
    # These overrides apply only to this mock test, never the robot profile.
    profile = yaml.safe_load((workspace/'src/erob_bringup/config/robot_mock.yaml').read_text())
    assert profile['bus']['backend'] == 'mock'
    profile['bus']['command_timeout_ms'] = 200
    profile['bus']['lateness_us'] = 100000
    test_config = logs/'profile.yaml'
    test_config.write_text(yaml.safe_dump(profile))
    with (logs/'launch.log').open('w') as log:
        process = subprocess.Popen(['ros2','launch','erob_bringup','robot.launch.py','backend:=mock',
                                    'enable_mock_fault_injection:=true', 'config_file:='+str(test_config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init(); node=Check()
        try:
            node.run()
        finally:
            node.destroy_node(); rclpy.shutdown()
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=10)

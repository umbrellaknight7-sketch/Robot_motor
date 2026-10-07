#!/usr/bin/env python3
"""Private mock acceptance. Offsets are inspected; no gravity physics is simulated."""

import os
import signal
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from wheel_human_interfaces.action import MoveJoints, MoveToPose
from wheel_human_interfaces.msg import CommanderStatus
from wheel_human_interfaces.srv import AcquireControl, RenewControl, ReleaseControl
from workspace_paths import test_log_directory


class Acceptance(Node):
    def __init__(self):
        super().__init__('commander_acceptance')
        prefix = '/wheel_human_commander/'
        self.acquire = self.create_client(AcquireControl, prefix + 'acquire_control')
        self.renew = self.create_client(RenewControl, prefix + 'renew_control')
        self.release = self.create_client(ReleaseControl, prefix + 'release_control')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.parameters = self.create_client(GetParameters, '/controller_manager/get_parameters')
        self.moves = ActionClient(self, MoveJoints, prefix + 'move_joints')
        self.poses = ActionClient(self, MoveToPose, prefix + 'move_to_pose')
        self.controller_status = {}
        self.controller_phase = ""
        self.status = None
        self.positions = {}
        self.diagnostics = {}
        self.token = ''
        self.progress = []
        self.nonzero_velocity = False
        self.nonzero_torque = False
        self.create_subscription(CommanderStatus, prefix + 'status', self.on_status, 10)
        self.create_subscription(DiagnosticArray, '/csp_controller/status', self.on_controller, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        # Example ownership renewals run at 5 Hz, separately from 10 Hz application activity.
        self.create_timer(.2, self.renew_owner)
        self.application_ticks = 0
        self.create_timer(.1, self.application_tick)

    def application_tick(self):
        self.application_ticks += 1

    def on_status(self, message):
        self.status = message

    def on_controller(self, message):
        if message.status:
            self.controller_phase = message.status[0].message
            self.controller_status = {v.key: v.value for v in message.status[0].values}

    def on_joints(self, message):
        self.positions = dict(zip(message.name, message.position))

    def on_diagnostics(self, message):
        self.diagnostics = {s.name: {v.key: v.value for v in s.values}
                            for s in message.status if s.name.startswith('ErobSystem/')}
        for row in self.diagnostics.values():
            self.nonzero_velocity |= abs(float(row.get('commanded_velocity_offset', 0))) > .005
            self.nonzero_torque |= float(row.get('commanded_torque_offset', 0)) < -.02

    def renew_owner(self):
        if self.token:
            self.renew.call_async(RenewControl.Request(token=self.token))

    def wait(self, predicate, timeout=15, reason='condition'):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(f'Timeout: {reason}; commander={self.status}; hardware={self.diagnostics}')

    def future(self, future, timeout=15):
        self.wait(future.done, timeout, 'future')
        assert future.exception() is None, future.exception()
        return future.result()

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=30), client.srv_name
        return self.future(client.call_async(request))

    def ownership(self, source, takeover=False):
        response = self.call(self.acquire, AcquireControl.Request(source_id=source, takeover=takeover))
        assert response.success, response.message
        self.token = response.token
        return response.token

    def goal(self, target=.1, duration=2, token=None, gravity=False, torque=True):
        goal = MoveJoints.Goal()
        goal.token = self.token if token is None else token
        goal.joint_names = ['Feet_Joint']
        goal.positions = [target]
        goal.duration.sec = duration
        goal.gravity_compensation = gravity
        if torque:
            goal.torque_joint_names = ['Feet_Joint']
            goal.torque_offsets = [-.15]
        return goal

    def send(self, goal):
        assert self.moves.wait_for_server(timeout_sec=10)
        return self.future(self.moves.send_goal_async(goal, feedback_callback=lambda f: self.progress.append(f.feedback.progress)))

    def neutral(self):
        return bool(self.diagnostics) and all(
            abs(float(row['commanded_velocity_offset'])) < 1e-8 and
            abs(float(row['commanded_torque_offset'])) < 1e-8
            for row in self.diagnostics.values())

    def run(self, logs, process):
        until = time.monotonic() + 30
        while time.monotonic() < until:
            states = {c.name: c.state for c in self.call(self.controllers, ListControllers.Request()).controller}
            if states.get('csp_controller') == 'inactive' and states.get('joint_state_broadcaster') == 'active':
                break
        else:
            raise AssertionError('Controller did not start inactive')
        self.wait(lambda: self.status is not None and len(self.positions) == 15)
        assert not self.status.ready
        assert not self.call(self.acquire, AcquireControl.Request(source_id='early')).success
        request = GetParameters.Request(names=['update_rate'])
        assert self.call(self.parameters, request).values[0].integer_value == 1000
        request = SwitchController.Request(activate_controllers=['csp_controller'], strictness=2)
        request.timeout.sec = 5
        assert self.call(self.switch, request).ok
        self.wait(lambda: self.status.ready, reason='active commander')
        original = dict(self.positions)
        owner_a = self.ownership('synthetic_vla')
        assert not self.call(self.acquire, AcquireControl.Request(source_id='synthetic_teleop')).success
        assert not self.send(self.goal(token='stale-or-forged')).accepted
        handle = self.send(self.goal())
        assert handle.accepted
        assert not self.send(self.goal(target=.2)).accepted
        result = self.future(handle.get_result_async())
        assert result.result.error_code == 0, result.result
        self.wait(lambda: self.status.ready)
        assert abs(self.positions['Feet_Joint'] - .1) < .002
        assert all(abs(self.positions[k] - v) < .002 for k, v in original.items() if k != 'Feet_Joint')
        assert self.nonzero_velocity and self.nonzero_torque
        assert max(self.progress) > .9
        assert self.application_ticks > 15
        print('PASS: inactive startup, competing sources, busy rejection, subset goal, signed offsets, 10 Hz client / configured 1 kHz cycles', flush=True)

        todo = self.send(self.goal(gravity=True))
        assert todo.accepted and self.future(todo.get_result_async()).result.error_code == 1
        pose = MoveToPose.Goal(token=self.token, planning_group='arm', end_effector_link='hand')
        assert self.poses.wait_for_server(timeout_sec=5)
        todo = self.future(self.poses.send_goal_async(pose))
        assert todo.accepted and self.future(todo.get_result_async()).result.error_code == 1
        print('PASS: gravity and Cartesian TODOs report NOT_IMPLEMENTED', flush=True)

        moving = self.send(self.goal(target=.3, duration=3))
        assert moving.accepted
        self.wait(lambda: self.status.state == 'executing')
        owner_b = self.ownership('synthetic_teleop', takeover=True)
        assert owner_b != owner_a and self.neutral()
        assert self.future(moving.get_result_async()).result.error_code != 0
        assert not self.call(self.renew, RenewControl.Request(token=owner_a)).success
        assert not self.send(self.goal(token=owner_a)).accepted
        print('PASS: takeover waits for stationary neutral hold and revokes old token', flush=True)

        moving = self.send(self.goal(target=-.1, duration=3))
        assert moving.accepted
        self.wait(lambda: self.status.state == 'executing')
        assert self.future(moving.cancel_goal_async()).goals_canceling
        result = self.future(moving.get_result_async())
        assert result.status == 5 and result.result.error_code == 7
        self.wait(self.neutral)
        print('PASS: explicit cancellation completes only after neutral stop', flush=True)

        moving = self.send(self.goal(target=.25, duration=5))
        assert moving.accepted
        self.token = ''  # Simulate loss of the active application, commander remains alive.
        self.wait(lambda: not self.status.owner and self.status.state == 'ready', reason='lease expiry hold')
        assert self.neutral()
        assert self.future(moving.get_result_async()).result.error_code != 0
        assert not self.call(self.renew, RenewControl.Request(token=owner_b)).success
        self.ownership('synthetic_vla_return')
        response = self.call(self.release, ReleaseControl.Request(token=self.token))
        assert response.success, response.message
        self.token = ''
        self.wait(lambda: not self.status.owner and self.status.ready)
        assert self.neutral()
        print('PASS: lease expiry, reacquisition and release preserve stationary neutral hold', flush=True)

        owner_before_fault = self.ownership('handover_failure_source')
        request = SwitchController.Request(deactivate_controllers=['csp_controller'], strictness=2)
        request.timeout.sec = 5
        assert self.call(self.switch, request).ok
        self.wait(lambda: self.status.state == 'fault', reason='failed stop acknowledgement fault')
        assert not self.call(self.acquire, AcquireControl.Request(source_id='blocked_takeover', takeover=True)).success
        assert not self.call(self.renew, RenewControl.Request(token=owner_before_fault)).success
        self.token = ''
        request = SwitchController.Request(activate_controllers=['csp_controller'], strictness=2)
        request.timeout.sec = 5
        assert self.call(self.switch, request).ok
        self.wait(lambda: self.controller_phase == 'holding' and self.neutral())
        reset = self.create_client(Trigger, '/wheel_human_commander/reset_fault')
        self.wait(lambda: self.controller_status.get('heartbeat_valid') == 'true')
        assert self.call(reset, Trigger.Request()).success
        self.wait(lambda: self.status.ready)
        print('PASS: unavailable controller makes stopping fault visible, blocks takeover and requires explicit reset', flush=True)

        expire = self.create_client(Trigger, '/ErobSystem/test_expire_commands')
        assert self.call(expire, Trigger.Request()).success
        self.wait(lambda: self.diagnostics and all(row['phase_name'] == 'holding' for row in self.diagnostics.values()),
                  reason='powered hardware hold')
        assert not self.call(self.acquire, AcquireControl.Request(source_id='while_faulted')).success
        with (logs / 'recovery.log').open('w') as output:
            recovery = subprocess.Popen(['ros2', 'run', 'erob_bringup', 'resume_control.py'],
                                        stdout=output, stderr=subprocess.STDOUT)
            try:
                self.wait(lambda: recovery.poll() is not None, timeout=25, reason='explicit recovery helper')
                assert recovery.returncode == 0, (logs / 'recovery.log').read_text()
            finally:
                if recovery.poll() is None:
                    recovery.terminate()
                    recovery.wait(timeout=10)
        self.wait(lambda: self.status.ready and self.neutral(), reason='recovered readiness')
        print('PASS: hardware hold prevents acquisition; recovery helper resets CSP controller with neutral offsets', flush=True)

        self.ownership('process_loss_client')
        moving = self.send(self.goal(target=.35, duration=5))
        assert moving.accepted
        self.wait(lambda: any(float(row.get('commanded_torque_offset', 0)) < -.02
                              for row in self.diagnostics.values()), reason='applied torque before process loss')
        # Kill only the commander child belonging to this test's private launch session.
        match = re.search(r'\[commander_node-\d+\]: process started with pid \[(\d+)\]',
                          (logs / 'launch.log').read_text())
        assert match, 'Missing commander child PID'
        pid = int(match.group(1))
        assert os.getsid(pid) == process.pid
        self.token = ''
        os.kill(pid, signal.SIGKILL)
        self.wait(lambda: self.controller_phase == 'holding' and
                  self.controller_status.get('heartbeat_valid') == 'false' and self.neutral(),
                  reason='controller execution watchdog after commander loss')
        print('PASS: commander process loss stops execution and withdraws torque despite continuing hardware cycles', flush=True)


def main():
    logs = test_log_directory('commander')
    os.environ.update(ROS_DOMAIN_ID='180', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs))
    profile = Path(get_package_share_directory('wheel_human_bringup')) / 'config/robot_mock_feedforward.yaml'
    config = yaml.safe_load(profile.read_text())
    # Shared-host software acceptance tolerates scheduler stalls. Deterministic
    # unit tests exercise the production watchdog limits; cycles remain 1 ms.
    config['bus'].update(command_timeout_ms=200, feedback_timeout_ms=50, lateness_us=100000)
    test_profile = logs / 'synthetic_profile.yaml'
    test_profile.write_text(yaml.safe_dump(config))
    with (logs / 'launch.log').open('w') as output:
        process = subprocess.Popen(['ros2', 'launch', 'wheel_human_bringup', 'commander.launch.py',
                                    'config_file:=' + str(test_profile), 'enable_mock_fault_injection:=true'], stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = Acceptance()
            node.run(logs, process)
        except Exception:
            print((logs / 'launch.log').read_text()[-16000:], file=sys.stderr)
            raise
        finally:
            if node:
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

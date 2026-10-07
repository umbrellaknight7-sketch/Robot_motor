#!/usr/bin/env python3
"""Mock acceptance of wrist/body fault isolation and SDK command ownership."""

import os
import json
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from wheel_human_interfaces.srv import AcquireControl, RenewControl
from check_commander import Acceptance
from workspace_paths import test_log_directory


class Wrists(Acceptance):
    def __init__(self):
        super().__init__()
        self.tx = {}
        self.sdk_status = {}
        self.create_subscription(String, '/cb_left_wrist/status', self.on_sdk, 10)
        self.create_subscription(JointState, '/cb_left_hand_transmitted', self.transmitted, 10)
        self.fingers = self.create_publisher(JointState, '/cb_left_hand_control_cmd', 10)
        self.enable_feedback = self.create_client(SetBool, '/cb_left_hand/mock_feedback')
        self.reset_left = self.create_client(Trigger, '/wheel_human_wrist_adapter/left/reset_fault')

    def on_sdk(self, message):
        self.sdk_status = json.loads(message.data)

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

    def transmitted(self, message):
        self.tx = list(message.position)

    def wrist_goal(self, side='Left', target=.2, duration=2):
        goal = self.goal(target=target, duration=duration, torque=False)
        goal.joint_names = [side + '_Wrist_Joint']
        return goal

    def wrist_ready(self, side=0):
        return self.status and len(self.status.wrist_ready) == 2 and self.status.wrist_ready[side]

    def move_ok(self, request):
        self.wait(lambda: self.status.ready)
        handle = self.send(request)
        assert handle.accepted, self.status
        result = self.future(handle.get_result_async())
        assert result.result.error_code == 0, result.result
        return result

    def run(self, logs, process):
        self.wait(lambda: self.controllers.service_is_ready(), 30)
        self.wait(lambda: {c.name: c.state for c in self.call(self.controllers, ListControllers.Request()).controller}.get('csp_controller') == 'inactive', 30)
        assert self.call(self.switch, SwitchController.Request(activate_controllers=['csp_controller'], strictness=2)).ok
        self.wait(lambda: self.status and self.status.ready and all(self.status.wrist_ready), 30)
        token = self.ownership('wrist_vla')
        assert not self.call(self.acquire, AcquireControl.Request(source_id='competing_fingers')).success
        initial_right = self.positions['Right_Wrist_Joint']
        # A legacy 17-position finger command must not change wrist entry 16.
        for _ in range(5):
            self.fingers.publish(JointState(position=[12.] * 16 + [800.]))
            rclpy.spin_once(self, timeout_sec=.05)
        self.wait(lambda: self.tx and self.tx[:16] == [12.] * 16)
        assert abs(self.tx[16]) < 1.
        move = self.send(self.wrist_goal())
        assert move.accepted, self.status
        assert not self.send(self.goal(torque=False)).accepted
        mixed = self.wrist_goal()
        mixed.joint_names += ['Feet_Joint']
        mixed.positions.append(0.)
        assert self.future(move.get_result_async()).result.error_code == 0
        self.wait(lambda: self.status.ready)
        assert not self.send(mixed).accepted
        self.wait(lambda: self.wrist_ready())
        assert abs(self.positions['Left_Wrist_Joint']-.2) < .02
        assert abs(self.positions['Right_Wrist_Joint']-initial_right) < .001
        assert self.tx[:16] == [12.] * 16 and abs(self.tx[16]-100.) < 10.
        print('PASS: wrist SI conversion, measured goal, finger preservation, subset and busy/mixed rejection', flush=True)

        move = self.send(self.wrist_goal(target=-.2, duration=3))
        assert move.accepted, self.status
        self.wait(lambda: len(self.progress)>5)
        assert self.future(move.cancel_goal_async()).goals_canceling
        assert self.future(move.get_result_async()).result.error_code == 7
        self.wait(lambda: self.status.ready and self.wrist_ready())
        move = self.send(self.wrist_goal(target=.25, duration=3))
        assert move.accepted, self.status
        replacement = self.ownership('wrist_teleop', takeover=True)
        assert replacement != token
        assert not self.call(self.renew, RenewControl.Request(token=token)).success
        assert self.future(move.get_result_async()).result.error_code == 7
        self.wait(lambda: self.status.ready and self.wrist_ready())
        print('PASS: wrist cancellation and takeover wait for measured hold; old token rejected', flush=True)

        wrist_start = self.positions['Left_Wrist_Joint']
        move = self.send(self.wrist_goal(target=-.25, duration=3))
        assert move.accepted, self.status
        self.wait(lambda: abs(self.positions['Left_Wrist_Joint']-wrist_start) > .002, reason='moving wrist before feedback loss')
        assert self.call(self.enable_feedback, SetBool.Request(data=False)).success
        result = self.future(move.get_result_async(), timeout=8)
        assert result.result.error_code == 6, result.result
        self.wait(lambda: self.status.ready)
        assert self.status.owner == 'wrist_teleop' and self.status.state != 'fault'
        assert not self.wrist_ready()
        assert not self.send(self.wrist_goal()).accepted
        self.move_ok(self.goal(target=.08, torque=False))
        self.wait(lambda: self.wrist_ready(1))
        self.move_ok(self.wrist_goal(side='Right', target=.1))
        assert self.call(self.enable_feedback, SetBool.Request(data=True)).success
        # Restored feedback alone must not resume/re-arm the failed wrist.
        end = time.monotonic()+.8
        while time.monotonic()<end:
            rclpy.spin_once(self, timeout_sec=.02)
        assert not self.wrist_ready()
        assert self.call(self.reset_left, Trigger.Request()).success
        self.wait(lambda: self.wrist_ready())
        self.move_ok(self.wrist_goal(target=.1))
        print('PASS: wrist timeout isolated; same owner executes body and other wrist; explicit reset required', flush=True)

        # Losing idle wrist feedback during a body trajectory must not interrupt it.
        move = self.send(self.goal(target=.02, duration=2, torque=False))
        assert move.accepted, self.status
        assert self.call(self.enable_feedback, SetBool.Request(data=False)).success
        assert self.future(move.get_result_async()).result.error_code == 0
        assert self.call(self.enable_feedback, SetBool.Request(data=True)).success
        self.wait(lambda: self.status.ready and self.wrist_ready())
        move = self.send(self.wrist_goal(target=-.2, duration=4))
        assert move.accepted, self.status
        self.token = ''
        assert self.future(move.get_result_async(), timeout=8).result.error_code == 7
        self.wait(lambda: self.status.ready and not self.status.owner)
        self.ownership('after_wrist_lease_loss')
        self.move_ok(self.goal(target=0., torque=False))
        print('PASS: body goal survives wrist feedback loss; wrist lease expiry stops and allows new ownership', flush=True)
        # Exercise the SDK's independent watchdog and delayed-command fence.
        arm = self.create_client(Trigger, '/cb_left_wrist/arm')
        stop = self.create_client(Trigger, '/cb_left_wrist/stop')
        raw = self.create_publisher(JointState, '/cb_left_wrist_control_cmd', 1)
        self.wait(lambda: raw.get_subscription_count() > 0, reason='SDK raw wrist command discovery')
        self.wait(lambda: self.wrist_ready())
        session = self.call(arm, Trigger.Request())
        assert session.success
        command = JointState(position=[50.])
        command.header.frame_id = session.message
        command.header.stamp = self.get_clock().now().to_msg()
        raw.publish(command)
        self.wait(lambda: self.tx and abs(self.tx[16]-50.) < 1.)
        assert self.call(stop, Trigger.Request()).success
        command.position = [100.]
        command.header.stamp = self.get_clock().now().to_msg()
        raw.publish(command)
        end = time.monotonic()+.3
        while time.monotonic()<end:
            rclpy.spin_once(self, timeout_sec=.02)
        assert abs(self.tx[16]-50.) < 1.
        session = self.call(arm, Trigger.Request())
        assert session.success
        # No wrist commands follow arm: local SDK command timeout must latch.
        end = time.monotonic()+.8
        while time.monotonic()<end:
            rclpy.spin_once(self, timeout_sec=.02)
        assert not self.wrist_ready() and self.status.ready
        assert not self.call(arm, Trigger.Request()).success
        assert self.call(self.reset_left, Trigger.Request()).success
        self.wait(lambda: self.wrist_ready())
        print('PASS: SDK local command watchdog and delayed wrist command fencing', flush=True)
        start = self.positions['Left_Wrist_Joint']
        move = self.send(self.wrist_goal(target=-.3, duration=4))
        assert move.accepted, self.status
        self.wait(lambda: abs(self.positions['Left_Wrist_Joint']-start) > .005)
        match = re.search(r'\[wrist_adapter-\d+\]: process started with pid \[(\d+)\]',
                          (logs / 'launch.log').read_text())
        assert match, 'Missing private wrist adapter PID'
        pid = int(match.group(1))
        assert os.getsid(pid) == process.pid
        os.kill(pid, signal.SIGKILL)
        assert self.future(move.get_result_async(), timeout=8).result.error_code == 6
        self.wait(lambda: self.sdk_status.get('fault') and not self.sdk_status.get('armed'))
        self.wait(lambda: self.status.ready)
        assert self.status.state != 'fault' and self.status.owner == 'after_wrist_lease_loss'
        self.move_ok(self.goal(target=.03, torque=False))
        print('PASS: adapter process loss triggers SDK hold and preserves body control', flush=True)


def main():
    logs = test_log_directory('wrists')
    os.environ.update(ROS_DOMAIN_ID='181', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs))
    profile = Path(get_package_share_directory('wheel_human_bringup')) / 'config/robot_mock_feedforward.yaml'
    config = yaml.safe_load(profile.read_text())
    config['bus'].update(command_timeout_ms=200, feedback_timeout_ms=50, lateness_us=100000)
    path = logs / 'synthetic_profile.yaml'
    path.write_text(yaml.safe_dump(config))
    with (logs / 'launch.log').open('w') as output:
        process = subprocess.Popen(['ros2', 'launch', 'wheel_human_bringup', 'commander.launch.py',
                                    'wrist_mode:=mock', 'config_file:=' + str(path)],
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = Wrists()
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

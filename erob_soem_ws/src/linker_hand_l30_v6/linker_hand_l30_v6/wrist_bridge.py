"""Optional replacement SDK ROS node, with one serialized CAN owner per hand.

Finger positions retain their SDK units and topic. Only the adapter uses the
session-fenced wrist endpoint. Mock mode exercises the same command mux.
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from .wrist_mux import WristMux


class WristBridge(Node):
    def __init__(self):
        super().__init__('linker_wrist_bridge')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.side = param('hand_type', 'left')
        if self.side not in ('left', 'right'):
            raise ValueError('hand_type must be left or right')
        self.mock = param('mock', False)
        verified = param('commissioned', False)
        if not self.mock and not verified:
            raise ValueError('Real SDK bridge requires commissioned:=true; see wrist_integration.md')
        self.mux = WristMux(param('raw_lower', -900.0), param('raw_upper', 900.0),
                            param('timeout', 0.5))
        self.api = None
        self.active = self.mock
        self.enabled_feedback = True
        self.running = True
        self.io_error = ''
        self.activate_requested = False
        self.velocity_settings = None
        self.torque_settings = None
        self.wrist_speed = param('wrist_speed_raw', 50)
        self.wrist_torque = param('wrist_torque_limit_raw', 100)
        if not self.mock:
            from .LinkerHandL30API.linker_hand_l30_api import LinkerHandL30API
            self.api = LinkerHandL30API(
                hand_type=self.side, device_id=param('device_id', 6),
                canfd_id=param('canfd_id', 0), comm_type=param('comm_type', 'socketcan'),
                channel=param('channel', 'can0'), auto_setup=False, enable_on_connect=False)
        base = '/cb_' + self.side
        self.state_pub = self.create_publisher(JointState, base + '_hand_state', 10)
        self.status_pub = self.create_publisher(String, base + '_wrist/status', 10)
        self.tx_pub = self.create_publisher(JointState, base + '_hand_transmitted', 10) if self.mock else None
        self.create_subscription(JointState, base + '_hand_control_cmd', self.fingers, 10)
        self.create_subscription(JointState, base + '_wrist_control_cmd', self.wrist, 1)
        self.create_subscription(String, '/cb_hand_setting_cmd', self.settings, 10)
        self.create_service(Trigger, base + '_wrist/arm', self.arm)
        self.create_service(Trigger, base + '_wrist/stop', self.stop)
        self.create_service(Trigger, base + '_wrist/reset', self.reset)
        self.create_service(Trigger, base + '_hand/activate', self.activate)
        if self.mock:
            self.create_service(SetBool, base + '_hand/mock_feedback', self.set_feedback)
        self.create_timer(0.05, self.status)
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()

    def fingers(self, msg):
        if not self.active or not self.mux.fingers(list(msg.position)):
            self.get_logger().warning('Rejected finger command: invalid vector or no measured initialization')

    def wrist(self, msg):
        age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        if len(msg.position) == 1 and -0.05 <= age <= self.mux.timeout:
            self.mux.wrist(msg.header.frame_id, msg.position[0], time.monotonic())

    def settings(self, msg):
        # The shared settings topic must explicitly address this hand. Wrist
        # settings are commissioned parameters, never changed by finger clients.
        try:
            data = json.loads(msg.data)
            if data.get('hand_type') != self.side:
                return
            kind = data['setting_cmd']
            key = 'speed' if kind == 'set_speed' else 'torque'
            values = [int(x) for x in data['params'][key]]
            if len(values) not in (16, 17):
                return
            with self.mux.lock:
                if kind == 'set_speed':
                    self.velocity_settings = values[:16] + [self.wrist_speed]
                elif kind == 'set_max_torque_limits':
                    self.torque_settings = values[:16] + [self.wrist_torque]
        except (KeyError, TypeError, ValueError, OverflowError):
            self.get_logger().warning('Invalid finger settings')

    def arm(self, _, response):
        response.message = self.mux.arm(time.monotonic()) if self.active else ''
        response.success = bool(response.message)
        return response

    def stop(self, _, response):
        self.mux.stop()
        response.success = True
        response.message = 'Session revoked; measured hold requested, not a physical stop acknowledgement'
        return response

    def reset(self, _, response):
        response.success = self.mux.reset(time.monotonic())
        response.message = 'Reset; new session required' if response.success else 'Fresh feedback required'
        return response

    def activate(self, _, response):
        with self.mux.lock:
            if self.active:
                response.success, response.message = True, 'Hand already active'
                return response
            response.success = self.mux.fresh(time.monotonic()) and not self.mux.fault
            if response.success:
                self.mux.stop()
                self.mux.target = list(self.mux.measured)
                self.activate_requested = True
        response.message = 'SDK all-hand enable requested; observe status for completion'
        return response

    def set_feedback(self, request, response):
        self.enabled_feedback = request.data
        response.success = True
        return response

    def status(self):
        self.mux.check(time.monotonic())
        with self.mux.lock:
            self.status_pub.publish(String(data=json.dumps(dict(
                active=self.active, fresh=self.mux.fresh(time.monotonic()),
                fault=self.mux.fault or self.io_error, armed=bool(self.mux.token)))))

    def io_loop(self):
        mock_position = [0.0] * 17
        while self.running and rclpy.ok():
            try:
                # Hold the mux lock through transmission: stop acknowledgement
                # cannot be overtaken by an already-copied old wrist command.
                with self.mux.lock:
                    target = self.mux.snapshot(time.monotonic())
                    if self.api:
                        if self.velocity_settings is not None:
                            self.api.set_velocities(self.velocity_settings)
                            self.velocity_settings = None
                        if self.torque_settings is not None:
                            self.api.set_joint_torques(self.torque_settings)
                            self.torque_settings = None
                        if target is not None and (self.active or self.activate_requested):
                            sent = self.api.hand_controller.set_positions([round(x) for x in target])
                            if not sent:
                                raise RuntimeError('SDK position transmission failed')
                        if self.activate_requested:
                            self.api.set_velocities([self.wrist_speed] * 17)
                            self.api.set_joint_torques([self.wrist_torque] * 17)
                            self.active = bool(self.api.hand_controller.protocol.enable_all_joints())
                            self.activate_requested = False
                    elif target is not None:
                        mock_position = target
                        self.tx_pub.publish(JointState(position=[float(x) for x in target]))
                if self.enabled_feedback:
                    positions = self.api.get_joint_state() if self.api else mock_position
                    if not self.mux.feedback(positions, time.monotonic()):
                        raise ValueError('Invalid measured positions')
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = ['sdk_joint_' + str(i) for i in range(16)] + ['wrist_pitch']
                    msg.position = [float(x) for x in positions]
                    # Raw SDK speed units are not assumed to be radians/sec.
                    self.state_pub.publish(msg)
                    self.io_error = ''
            except Exception as error:
                self.io_error = str(error)
                self.mux.check(time.monotonic())
            time.sleep(0.02)

    def close(self):
        self.mux.stop()
        self.running = False
        self.worker.join(timeout=3)
        if self.api and not self.worker.is_alive():
            self.api.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = WristBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

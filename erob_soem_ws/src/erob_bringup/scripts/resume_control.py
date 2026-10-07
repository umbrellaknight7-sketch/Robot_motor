#!/usr/bin/env python3
"""Explicit CSP recovery. Keep upstream trajectory publishers paused until success."""

import argparse
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger


class Resume(Node):
    def __init__(self, hardware='ErobSystem', manager='/controller_manager', timeout=15.0):
        super().__init__('erob_resume_control')
        self.hardware = hardware.strip('/')
        self.timeout = timeout
        self.states = {}
        self.received = 0.0
        self.create_subscription(DiagnosticArray, '/diagnostics', self.feedback, 10)
        self.list_client = self.create_client(ListControllers, manager.rstrip('/') + '/list_controllers')
        self.switch_client = self.create_client(SwitchController, manager.rstrip('/') + '/switch_controller')
        self.resume_client = self.create_client(Trigger, '/' + self.hardware + '/resume')
        self.controller_ns = manager.rstrip('/').rsplit('/', 1)[0]

    def feedback(self, msg):
        rows = {s.name.split('/', 1)[1]: dict((v.key, v.value) for v in s.values)
                for s in msg.status if s.name.startswith(self.hardware + '/')}
        if rows:
            self.states = rows
            self.received = time.monotonic()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError('Missing service: ' + client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done() or future.exception():
            raise RuntimeError('Service failed: ' + client.srv_name)
        return future.result()

    def wait(self, predicate, message):
        until = time.monotonic() + self.timeout
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.states and time.monotonic() - self.received < 0.5 and predicate():
                return
        raise RuntimeError(message)

    def switch(self, start=(), stop=()):
        req = SwitchController.Request()
        req.activate_controllers = list(start)
        req.deactivate_controllers = list(stop)
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = int(self.timeout)
        if not self.call(self.switch_client, req).ok:
            raise RuntimeError('Controller switch failed; driver hold has not been released')

    def run(self):
        self.wait(lambda: True, 'No fresh hardware diagnostics')
        first = next(iter(self.states.values()))
        if first.get('phase_name') != 'holding':
            raise RuntimeError('Requires confirmed holding; current phase: ' + first.get('phase_name', 'unknown'))

        # Keep this hold generation fixed throughout controller restart and resume.
        generation = first['hold_generation']
        controllers = self.call(self.list_client, ListControllers.Request()).controller
        keys = {name + '/position' for name in self.states}
        affected = [c for c in controllers if keys.intersection(c.claimed_interfaces)]
        for c in affected:
            if c.type not in ('joint_trajectory_controller/JointTrajectoryController',
                              'erob_controllers/CspTrajectoryController'):
                raise RuntimeError('Unsupported claiming controller: ' + c.name)
            client = self.create_client(GetParameters, self.controller_ns + '/' + c.name + '/get_parameters')
            req = GetParameters.Request()
            req.names = ['open_loop_control', 'set_last_command_interface_value_as_state_on_activation']
            values = self.call(client, req).values
            self.destroy_client(client)
            if any(v.type != 1 or v.bool_value for v in values):
                raise RuntimeError(c.name + ' must initialize from feedback (both recovery parameters false)')

        # Release claims, then reactivate controllers so their targets start from feedback.
        names = [c.name for c in affected]
        if names:
            self.switch(stop=names)
        self.wait(lambda: all(s.get('claims_released') == '1' for s in self.states.values()),
                  'Worker did not confirm released interfaces; hold retained')
        if names:
            self.switch(start=names)
        self.wait(lambda: all(s.get('resume_ready') == '1' and s.get('hold_generation') == generation
                              and float(s.get('commanded_velocity_offset', 'nan')) == 0.0
                              and float(s.get('commanded_torque_offset', 'nan')) == 0.0
                              for s in self.states.values()), 'Hold not ready for explicit resume')

        # Resume only after every joint confirms the same hold and zero feedforward.
        response = self.call(self.resume_client, Trigger.Request())
        if not response.success:
            raise RuntimeError(response.message)
        print(response.message, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', default='ErobSystem')
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()
    rclpy.init()
    node = Resume(args.hardware, args.controller_manager, args.timeout)
    try:
        node.run()
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

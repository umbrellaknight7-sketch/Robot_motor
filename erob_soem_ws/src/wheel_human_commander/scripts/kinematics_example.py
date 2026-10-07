#!/usr/bin/env python3
"""Call calculation-only services with explicit positions; never command motion."""

import argparse
from copy import deepcopy
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from wheel_human_interfaces.srv import ComputeFK, ComputeIK
import xacro


def configuration(model_file):
    robot = ET.fromstring(xacro.process_file(model_file).toxml())
    state = JointState()
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        limits = joint.find('limit')
        lower, upper = float(limits.get('lower')), float(limits.get('upper'))
        state.name.append(joint.get('name'))
        state.position.append(max(lower, min(upper, 0.0)))
    return state


def arm_names(side):
    return [side + '_Shoulder_Joint'] + [side + '_Arm_Joint' + str(i) for i in range(1, 5)] + [side + '_Wrist_Joint']


class Queries(Node):
    def __init__(self):
        super().__init__('kinematics_example')
        self.fk_client = self.create_client(ComputeFK, '/wheel_human_kinematics/compute_fk')
        self.ik_client = self.create_client(ComputeIK, '/wheel_human_kinematics/compute_ik')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('Missing service: ' + client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or future.exception():
            raise RuntimeError('Service call failed: ' + client.srv_name)
        return future.result()

    def fk(self, state, frame):
        return self.call(self.fk_client, ComputeFK.Request(configuration=state, target_frame=frame))

    def ik(self, state, group, pose, mode):
        return self.call(self.ik_client, ComputeIK.Request(seed=state, planning_group=group, target=pose, target_mode=mode))


def verify_round_trips(node, state):
    # Deliberately avoid the CAD zero-pose near singularity.
    for side in ('Left', 'Right'):
        for name, angle in zip(arm_names(side), (.2, -.4, .3, .6, -.2, .1)):
            state.position[state.name.index(name)] = angle
    for side in ('Left', 'Right'):
        group, tip = side.lower() + '_arm', side + '_Hand_TCP'
        desired = node.fk(state, tip)
        assert desired.error_code == 0, desired.message
        for mode in (ComputeIK.Request.POSITION_ONLY, ComputeIK.Request.FULL_POSE):
            seed = deepcopy(state)
            for name in arm_names(side):
                seed.position[seed.name.index(name)] += .04
            result = node.ik(seed, group, desired.pose, mode)
            assert result.error_code == 0, result.message
            actual = node.fk(result.solution, tip)
            assert actual.error_code == 0, actual.message
            a, b = actual.pose.pose, desired.pose.pose
            position_error = math.dist((a.position.x, a.position.y, a.position.z), (b.position.x, b.position.y, b.position.z))
            assert position_error <= 1e-4, position_error
            if mode == ComputeIK.Request.FULL_POSE:
                dot = sum(getattr(a.orientation, k)*getattr(b.orientation, k) for k in ('x', 'y', 'z', 'w'))
                angle_error = 2*math.acos(min(1.0, abs(dot)))
                assert angle_error <= .001, angle_error
            solved = dict(zip(result.solution.name, result.solution.position))
            assert all(solved[n] == q for n, q in zip(seed.name, seed.position) if n not in arm_names(side))
            print(f'PASS: {group} mode={mode}, position_error={position_error:.3g} m, iterations={result.iterations}', flush=True)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-file', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Queries()
    try:
        verify_round_trips(node, configuration(args.model_file))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

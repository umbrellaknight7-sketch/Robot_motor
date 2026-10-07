#!/usr/bin/env python3
"""Isolated, calculation-only CAD acceptance. Never launches a controller or SDK."""

import argparse
from copy import deepcopy
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from wheel_human_interfaces.srv import ComputeIK
import xacro
import yaml
from workspace_paths import test_log_directory


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-file', required=True)
    args = parser.parse_args()
    model_file = str(Path(args.model_file).resolve(strict=True))
    logs = test_log_directory('kinematics')
    os.environ.update(ROS_DOMAIN_ID='188', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs / 'ros'))
    path = Path(get_package_prefix('wheel_human_commander')) / 'lib/wheel_human_commander/kinematics_example.py'
    spec = importlib.util.spec_from_file_location('kinematics_example', path)
    example = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(example)
    processes = []
    node = None
    with (logs / 'launch.log').open('w') as launch_log, (logs / 'rsp.log').open('w') as rsp_log:
        try:
            processes.append(subprocess.Popen(
                ['ros2', 'launch', 'wheel_human_commander', 'kinematics.launch.py', 'model_file:=' + model_file],
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True))
            rclpy.init()
            node = example.Queries()
            state = example.verify_round_trips(node, example.configuration(model_file))
            expected_nodes = {'wheel_human_kinematics', 'kinematics_example'}
            assert set(node.get_node_names()) == expected_nodes, node.get_node_names()
            publishers = node.get_publisher_names_and_types_by_node('wheel_human_kinematics', '/')
            assert all(name in ('/rosout', '/parameter_events') for name, _ in publishers), publishers
            subscribers = node.get_subscriber_names_and_types_by_node('wheel_human_kinematics', '/')
            assert all(name == '/parameter_events' for name, _ in subscribers), subscribers
            print('PASS: standalone query graph has no controllers, SDK, state subscriptions, or command publishers', flush=True)
            target = node.fk(state, 'Left_Hand_TCP').pose
            for change in ('missing', 'duplicate', 'unknown', 'nan', 'limits'):
                bad = deepcopy(state)
                if change == 'missing':
                    bad.name.pop(); bad.position.pop()
                elif change == 'duplicate':
                    bad.name[0] = bad.name[1]
                elif change == 'unknown':
                    bad.name[0] = 'unknown'
                elif change == 'nan':
                    bad.position[0] = float('nan')
                else:
                    bad.position[0] = 100
                assert node.fk(bad, 'Left_Hand_TCP').error_code == 1
                result = node.ik(bad, 'left_arm', target, ComputeIK.Request.FULL_POSE)
                assert result.error_code == 1 and not result.solution.name
            assert node.fk(state, 'unknown').error_code == 1
            bad = deepcopy(target); bad.header.frame_id = 'world'
            assert node.ik(state, 'left_arm', bad, ComputeIK.Request.FULL_POSE).error_code == 1
            bad = deepcopy(target); bad.pose.orientation.x = bad.pose.orientation.y = bad.pose.orientation.z = bad.pose.orientation.w = 0.0
            assert node.ik(state, 'left_arm', bad, ComputeIK.Request.FULL_POSE).error_code == 1
            assert node.ik(state, 'left_arm', bad, ComputeIK.Request.POSITION_ONLY).error_code == 0
            bad = deepcopy(target); bad.pose.position.x = 100.0
            result = node.ik(state, 'left_arm', bad, ComputeIK.Request.FULL_POSE)
            assert result.error_code in (2, 3) and not result.solution.name
            # Near-singular CAD zero pose: exact FK target succeeds without an invented solution.
            zero = example.configuration(model_file)
            exact = node.fk(zero, 'Left_Hand_TCP')
            assert node.ik(zero, 'left_arm', exact.pose, ComputeIK.Request.FULL_POSE).error_code == 0
            print('PASS: malformed, unreachable, wrong-frame and invalid-quaternion requests; zero-pose target', flush=True)

            # RSP is a separate reference process, not part of the query launch.
            description = xacro.process_file(model_file).toxml()
            parameters = logs / 'rsp.yaml'
            parameters.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': description}}}))
            processes.append(subprocess.Popen(
                ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(parameters)],
                stdout=rsp_log, stderr=subprocess.STDOUT, start_new_session=True))
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            publisher = node.create_publisher(JointState, '/joint_states', 10)
            for wrist_angle in (.1, .7):
                for side in ('Left', 'Right'):
                    state.position[state.name.index(side + '_Wrist_Joint')] = wrist_angle
                sent_at = node.get_clock().now()
                deadline = time.monotonic() + 15
                transforms = {}
                while time.monotonic() < deadline:
                    state.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(state)
                    rclpy.spin_once(node, timeout_sec=.05)
                    for side in ('Left', 'Right'):
                        tip = side + '_Hand_TCP'
                        if buffer.can_transform('base_link', tip, Time()):
                            tf = buffer.lookup_transform('base_link', tip, Time())
                            if Time.from_msg(tf.header.stamp).nanoseconds >= sent_at.nanoseconds:
                                transforms[tip] = tf
                    if len(transforms) == 2:
                        break
                assert len(transforms) == 2, 'Missing fresh TCP transforms'
                for tip, tf in transforms.items():
                    pose = node.fk(state, tip).pose.pose
                    translation, rotation = tf.transform.translation, tf.transform.rotation
                    error = math.dist((pose.position.x, pose.position.y, pose.position.z), (translation.x, translation.y, translation.z))
                    dot = sum(getattr(pose.orientation, k)*getattr(rotation, k) for k in ('x', 'y', 'z', 'w'))
                    assert error < 1e-8, (tip, error)
                    assert 2*math.acos(min(1.0, abs(dot))) < 1e-6
            del listener
            print('PASS: both CAD TCP poses match robot_state_publisher at two wrist angles', flush=True)
        except Exception:
            for path in (logs / 'launch.log', logs / 'rsp.log'):
                print(path.read_text()[-6000:])
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for process in reversed(processes):
                stop(process)


if __name__ == '__main__':
    main()

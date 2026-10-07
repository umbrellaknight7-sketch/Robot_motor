#!/usr/bin/env python3
"""Query both TCPs using an explicit saved pose. Publishes no joint commands."""

import argparse
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from wheel_human_interfaces.srv import ComputeFK
from wheel_human_description.poses import load_pose


def main():
    share = Path(get_package_share_directory('wheel_human_description'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-file', default=str(share / 'urdf/kinematics_fixed.urdf'))
    parser.add_argument('--pose-file', default=str(share / 'config/poses/upright.yaml'))
    args = parser.parse_args()
    positions = load_pose(args.pose_file, Path(args.model_file).read_text())
    configuration = JointState(name=list(positions), position=list(positions.values()))
    rclpy.init()
    node = Node('reference_pose_example')
    try:
        client = node.create_client(ComputeFK, '/wheel_human_kinematics/compute_fk')
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('FK service unavailable; launch cad_kinematics.launch.py')
        for tip in ('Left_Hand_TCP', 'Right_Hand_TCP'):
            future = client.call_async(ComputeFK.Request(configuration=configuration, target_frame=tip))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError('FK query timed out')
            result = future.result()
            if result.error_code:
                raise RuntimeError(result.message)
            p, q = result.pose.pose.position, result.pose.pose.orientation
            print(json.dumps({'tip': tip, 'frame': result.pose.header.frame_id,
                              'position_m': [p.x, p.y, p.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

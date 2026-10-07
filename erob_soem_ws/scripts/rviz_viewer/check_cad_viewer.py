#!/usr/bin/env python3
"""Verify the workspace CAD viewer, optionally exercise Center, and save a screen capture."""

import argparse
import ctypes
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from wheel_human_description.poses import load_pose, zero_pose
from network_check import check_network, LOGS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-pose', choices=('upright', 'zero'), default='upright')
    parser.add_argument('--test-center', action='store_true', help='Uses the standard 1600x1000 startup window layout')
    args = parser.parse_args()
    share = Path(get_package_share_directory('wheel_human_description'))
    description = (share / 'urdf/kinematics_fixed.urdf').read_text()
    expected = load_pose(share / 'config/poses/upright.yaml', description) if args.initial_pose == 'upright' else zero_pose(description)
    result = check_network()
    rclpy.init()
    node = Node('cad_viewer_acceptance')
    values, received = {}, {}
    node.create_subscription(JointState, '/joint_states', lambda msg: values.update(zip(msg.name, msg.position)), 10)
    node.create_subscription(String, '/robot_description', lambda msg: received.update(description=msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def wait(condition, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if condition():
                return
        raise AssertionError(('Viewer condition timed out', values))

    def matches():
        return set(values) == set(expected) and all(abs(values[n] - q) <= 6.28 / 10000 + 1e-12 for n, q in expected.items())

    try:
        wait(lambda: matches() and 'description' in received)
        robot = ET.fromstring(received['description'])
        assert robot.get('name') == 'Kinematics_fixed'
        assert len(robot.findall('link')) == 22
        assert robot.find("link[@name='base_link']/inertial") is None
        for mesh in robot.findall('.//mesh'):
            assert mesh.get('filename').startswith('package://wheel_human_description/')
        if args.test_center:
            x11 = ctypes.CDLL('libX11.so.6')
            xtest = ctypes.CDLL('libXtst.so.6')
            x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
            x11.XOpenDisplay.restype = ctypes.c_void_p
            x11.XFlush.argtypes = [ctypes.c_void_p]
            x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
            xtest.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
            xtest.XTestFakeButtonEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
            display = x11.XOpenDisplay(None)
            assert display

            def motion(x, y):
                xtest.XTestFakeMotionEvent(display, -1, x, y, 0)
                x11.XFlush(display)

            def button(pressed):
                xtest.XTestFakeButtonEvent(display, 1, pressed, 0)
                x11.XFlush(display)

            try:
                motion(1248, 261)
                button(1)
                time.sleep(.1)
                motion(1280, 261)
                time.sleep(.1)
                button(0)
                wait(lambda: abs(values['Feet_Joint']) > .1)
                result['test_moved_feet_rad'] = values['Feet_Joint']
                motion(1350, 52)
                button(1)
                button(0)
                wait(matches)
                result['center_restored_reference'] = True
                motion(1105, 970)
            finally:
                x11.XCloseDisplay(display)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert matches()
        nodes = sorted(node.get_node_names())
        assert all(
            n in {'cad_viewer_acceptance', 'rviz', 'rviz2', 'robot_state_publisher',
                  'joint_state_publisher', 'transform_listener_impl'}
            or n.startswith('transform_listener_impl_') for n in nodes), nodes
        app = QApplication.instance() or QApplication([])
        screenshot = LOGS / ('cad-' + args.initial_pose + '.png')
        assert app.primaryScreen().grabWindow(0).save(str(screenshot), 'PNG')
        result.update(initial_pose=args.initial_pose, joint_positions=values,
                      maximum_error_rad=max(abs(values[n] - q) for n, q in expected.items()),
                      nodes=nodes, snapshot=str(screenshot), installed_description=str(share))
        (LOGS / ('cad-' + args.initial_pose + '-acceptance.json')).write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

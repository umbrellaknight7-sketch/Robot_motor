#!/usr/bin/env python3
"""Standard joint GUI with stable nearest-step rounding for saved CAD poses."""

import signal
import sys
import threading

from joint_state_publisher.joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui.joint_state_publisher_gui import JointStatePublisherGui, RANGE
from python_qt_binding.QtWidgets import QApplication
import rclpy


class ReferencePoseGui(JointStatePublisherGui):
    def valueToSlider(self, value, joint):
        # Upstream int() truncation can drop another step when a slider value is
        # converted back through floating point during updateSliders(). Rounding
        # makes repeated Center/refresh operations stable without changing limits.
        scaled = (value - joint['min']) * RANGE / (joint['max'] - joint['min'])
        return max(0, min(RANGE, round(scaled)))


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    publisher = JointStatePublisher(None)
    window = ReferencePoseGui('Joint State Publisher', publisher)
    window.show()
    worker = threading.Thread(target=window.loop, daemon=True)
    worker.start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    try:
        app.exec_()
    finally:
        window.running = False
        worker.join()
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

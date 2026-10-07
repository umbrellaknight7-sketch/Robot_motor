#!/usr/bin/env python3
"""Launch and validate only the mock robot, then reap the entire launch group."""

import os
import signal
import subprocess
import sys
from workspace_paths import test_log_directory

logs = test_log_directory('mock')
env = dict(os.environ)
env.setdefault("ROS_DOMAIN_ID", "176")
env.setdefault("ROS_LOCALHOST_ONLY", "1")
env["ROS_LOG_DIR"] = str(logs)
with (logs / "launch.log").open("w") as output:
    process = subprocess.Popen(
        ["ros2", "launch", "erob_bringup", "robot.launch.py", "backend:=mock"],
        env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        result = subprocess.run(
            ["ros2", "run", "erob_bringup", "mock_demo.py"], env=env, timeout=90)
        status = result.returncode
    except subprocess.TimeoutExpired:
        print("Mock acceptance timed out; see", logs / "launch.log", file=sys.stderr)
        status = 1
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
if status:
    print((logs / "launch.log").read_text()[-16000:], file=sys.stderr)
sys.exit(status)

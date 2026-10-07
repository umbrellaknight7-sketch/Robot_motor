#!/usr/bin/env python3
"""Mock-only regression for the real-guide lifecycle and one-joint CSP action."""

import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from workspace_paths import test_log_directory

workspace=Path(__file__).resolve().parents[1]
logs=test_log_directory('csp_startup')
env=dict(os.environ, ROS_DOMAIN_ID="178", ROS_LOCALHOST_ONLY="1",
         ROS_LOG_DIR=str(logs/"ros"))
def run(args, timeout=40):
    r=subprocess.run(args, env=env, text=True, stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT, timeout=timeout)
    print(r.stdout,flush=True)
    if r.returncode: raise RuntimeError(args)
    return r.stdout
with (logs/"launch.log").open("w") as log:
    process=subprocess.Popen(["ros2","launch","erob_bringup","robot.launch.py","backend:=mock"],
                             env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        run(["ros2","run","controller_manager","spawner","commissioning_position_controller",
             "-t","joint_trajectory_controller/JointTrajectoryController",
             "-p",str(workspace/"doc/commissioning_position_controller.yaml"),
             "--inactive","--controller-manager-timeout","30"])
        description=run(["ros2","param","get","/controller_manager","robot_description"])
        import xml.etree.ElementTree as ET
        robot=ET.fromstring(description[description.index("<"):])
        assert robot.find("./ros2_control/hardware/param[@name='backend']").text=="mock"
        run(["ros2","control","set_hardware_component_state","ErobSystem","inactive"])
        run(["ros2","topic","echo","--once","/joint_states","--qos-reliability","best_effort"],timeout=15)
        run(["ros2","control","set_hardware_component_state","ErobSystem","active"])
        run(["ros2","control","set_controller_state","commissioning_position_controller","active"])
        goal="{trajectory: {joint_names: [motor_01_joint], points: [{positions: [0.02], velocities: [0.0], time_from_start: {sec: 3}}]}}"
        output=run(["ros2","action","send_goal","/commissioning_position_controller/follow_joint_trajectory",
                    "control_msgs/action/FollowJointTrajectory",goal],timeout=20)
        assert "error_code: 0" in output and "SUCCEEDED" in output,output
        output=run(["ros2","topic","echo","--once","/joint_states","--qos-reliability","best_effort"],timeout=15)
        state=next(yaml.safe_load_all("header:"+output.split("header:",1)[1]))
        values=dict(zip(state["name"],state["position"]))
        assert len(values)==15 and abs(values["motor_01_joint"]-0.02)<0.0001
        assert all(abs(v)<0.0001 for k,v in values.items() if k!="motor_01_joint")
        run(["ros2","control","set_controller_state","commissioning_position_controller","inactive"])
        run(["ros2","control","set_hardware_component_state","ErobSystem","inactive"])
        print("PASS: guide one-joint CSP action and normal shutdown; other 14 joints held",flush=True)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            process.wait(timeout=10)

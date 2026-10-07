# Wheel human robot packages

`erob_soem_ws` remains the single colcon workspace. `wheel_human_description`
owns the robot geometry; `wheel_human_bringup` combines it with the existing
`erob_hardware/ErobSystem` plugin. The generic `erob_bringup` example remains
available independently.

The original `../wheel_human_urdf/dummy_bot.urdf` is retained as a migration
reference. Its geometry is packaged as the `wheel_human_dummy_body` Xacro macro
in `wheel_human_description/urdf/dummy_body.xacro`. Runtime launch uses installed
package resources, without depending on that sibling directory. Preserve the
original as a reference; edit the packaged model for subsequent development.

## Build and display

Use the workspace's [native Humble guide](native_humble_install.md) and
`bash scripts/test_software.sh` to build and test on the robot. Install the new
runtime dependencies using rosdep from the workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
source install/setup.bash
ros2 launch wheel_human_description display.launch.py
```

The display starts RViz, robot_state_publisher and joint_state_publisher_gui.
Use `gui:=false` for joint_state_publisher without sliders and `rviz:=false`
to omit RViz. The fixed frame is `base_link`. Display mode does not start
controller_manager or any EtherCAT transport. Its synthetic joint states cover
all 19 movable joints and produce all 19 child TF frames.

## Mock robot control

```bash
ros2 launch wheel_human_bringup robot.launch.py rviz:=true
```

This runs one controller manager and one ErobSystem with the existing fake
EtherCAT transport. The joint-state broadcaster starts active; the position
trajectory controller starts inactive. Do not run the standalone display launch
in the same ROS domain: body joint states here come from the hardware interface.
Body-only bringup publishes 15 body joint states and their 15 child transforms.
The revolute wrists and wheels have no state source in this mode, so RViz cannot
place their hand/wheel links. Optional wrist integration supplies wrist states;
wheels remain description-only. Use the standalone display in a separate ROS
domain to preview the complete model with synthetic joint angles.

To explicitly activate the controller:

```bash
ros2 control switch_controllers --activate position_controller --strict
```

Commands use `/position_controller/follow_joint_trajectory` or
`/position_controller/joint_trajectory`. Supply all 15 joint names from
`config/controllers.yaml`, with position targets in radians. Position and
velocity state feedback are available; effort state is conditional on the
per-motor/profile `effort_verified` setting. Commands are position-only.

Run the isolated automated acceptance check from the workspace in a sourced
Humble environment:

```bash
python3 scripts/check_wheel_human_mock.py
```

It uses localhost ROS domain 177, launches and cleans up its own mock process
group, verifies the 19-joint model, 15 body joint states and 15 body child TF
frames, activates the controller,
checks 15 distinct signed position targets, and deactivates the controller.
Logs are under `log/<profile>/tests/wheel_human`. The workspace test script includes this
check after the existing generic driver acceptance checks.

## Mapping and model contract

The initial logical IDs 0x01–0x0F follow the configured body-joint order:
Feet, Knee, Hip, Neck, Face, left shoulder and four left arm joints, then right
shoulder and four right arm joints. This is **synthetic test data**, not physical
wiring. Branch A/B/C assignments, identities, paths and calibration are inherited
from the generic mock profile and do not describe anatomical groups.

`config/robot_mock.yaml` owns the ordered motor mapping. The control Xacro emits
joint declarations in that order. `config/controllers.yaml` owns the trajectory
controller's joint list. Launch requires exact coverage of the 15 body joints,
excluding the explicitly named SDK wrists and uncontrolled wheels. It rejects
missing/duplicate body mappings, wrist/wheel motor mappings, unrecognized movable
joints, incompatible interfaces, driver limits outside model limits and
mismatched controller joints before starting ROS nodes.

The dummy model has 20 links and 19 revolute joints rooted at `base_link`:
15 body joints, two SDK wrists, and two uncontrolled wheels. Wrists remain
revolute even when their backend is disabled. Neither wrists nor wheels enter
the body motor mapping or ros2_control interfaces; there is no mobile-base
controller. The dummy model and motor assignments are software placeholders,
not a calibrated control model. Mock operation does not validate physical
dynamics or commissioning.

## Real hardware integration

Real launch requires both an explicit hardware configuration and an explicit
calibrated geometry model:

```bash
ros2 launch wheel_human_bringup robot.launch.py backend:=soem \
  config_file:=/absolute/path/verified_robot.yaml \
  model_file:=/absolute/path/calibrated_robot.urdf.xacro
```

The model must expand to a complete geometry-only URDF containing the same 15
body revolute joints and no existing ros2_control block. The explicitly named
wrists and wheels are excluded from body motor coverage; other unmapped movable
joints are rejected. Bringup adds the body control block. The
packaged dummy model carries a software-only marker and is rejected for SOEM.
Supplying a file does not verify its calibration: complete the
[commissioning procedure](real_robot_first_test.md), replace the synthetic
mapping in `robot_real.template.yaml`, and retain the driver's verification
gates. The requested backend must match `bus.backend` in the supplied YAML.

Real hardware starts inactive and no controllers are automatically spawned.
The existing hold/recovery behavior remains in the eRob driver; use
`ros2 run erob_bringup resume_control.py` for explicit recovery according to the
[hold contract](hold_recovery.md). All 15 motors remain one hardware fault/hold
group, even if separate application controllers are introduced later.

For development on this server, activate the existing `ros_humble` conda
environment and run `bash scripts/test_software.sh --current`; source
`artifacts/dev/install/setup.bash`. This is separate from native robot deployment.

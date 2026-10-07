# First real-robot commissioning: installation through one CSP move

This procedure describes the current implementation. **Only software/mock validation has been completed.** Installation and disabled discovery can be done now; motor enabling depends on a completed, verified real profile. Do not turn the verification flags on merely to bypass an error.

Run the EtherCAT master on the computer physically connected to the junction. The remote development server cannot control this bus through a normal routed network. The robot uses **Ubuntu 22.04 x86-64 with system-wide ROS 2 Humble at /opt/ros/humble**. Commands use the example workspace path /home/xurui/downloads/Robot_motor/erob_soem_ws; adjust it to your robot account. The conda environment is for development-server tests only.

## 1. Immediately after main power is on

Support the robot and its gravity-loaded links so drive enable, brake release or drive disable cannot cause a fall. Have the independent hardware stop available and verify its behavior for this robot. The ROS controller stop commands below are not that hardware stop.

Confirm the computer connects to the intended hub upstream port and that A/B/C connect to motors 0x01–0x07, 0x08–0x0C and 0x0D–0x0F. The hub still needs identification as an EtherCAT junction, with its port map and ESI. An ordinary Ethernet switch is not a verified substitute for this topology.

Stop other EtherCAT masters, vendor applications and old demonstration programs. Do not unplug a branch as an initial test. The current robot profile requires all 15 motors to match. **Activating ErobSystem enables all 15 drives; a one-joint controller does not isolate drive power to one motor.**

## 2. Install/build as the ordinary user

Follow [native_humble_install.md](native_humble_install.md) to install official Humble apt packages, the control stack and build dependencies. No conda installation is needed on the robot.

Open a fresh Bash terminal with no conda/virtualenv or other ROS distribution sourced:

```bash
source /opt/ros/humble/setup.bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
bash scripts/test_software.sh
source install/setup.bash
```

The script builds into `build/` and `install/`, runs the software tests and mock
acceptance checks, and never opens the real EtherCAT bus. Reserve ROS domains
176–179 for those tests. Do not reuse or source the development server's
`artifacts/dev/` outputs; build from source on the robot. End all mock launches
before starting the real session.

See [validation.md](validation.md) for the current development-environment result. Native acceptance must come from the robot's actual run. The mock demo is not a real-robot commissioning program.

## 3. Prepare an unverified real profile and identify the NIC

In the ordinary-user terminal, create a new local configuration once. Preserve an existing commissioned file instead of overwriting it:

```bash
mkdir -p config
cp -n src/erob_bringup/config/robot_real.template.yaml config/robot_real.yaml
ip -br link
```

Edit `config/robot_real.yaml`: set `bus.interface` to the dedicated EtherCAT NIC, for example `enp3s0`. Keep backend `soem` and all verification flags false for initial discovery. The template's counts, identities, serials, port paths, limits and calibration are placeholders.

Do not select or reconfigure the NIC carrying your SSH connection. EtherCAT uses raw Ethernet; this driver does not require an IP address for the EtherCAT link. If the dedicated interface is down, bring only that verified interface up:

```bash
sudo ip link set dev enp3s0 up
```

Replace `enp3s0` with the real dedicated interface name.

## 4. Open the privileged bus terminal and discover

SOEM needs raw Ethernet access; the real worker also requires FIFO scheduling and locked memory. For this supervised first session, use a root terminal only for the bus tools/master, keeping ROS client commands in an ordinary-user terminal. A production service should use appropriately scoped capabilities and real-time limits instead. Humble's scheduling and memory-lock requirements are described in the [controller-manager documentation](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html).

In Terminal A:

```bash
sudo /bin/bash --noprofile --norc
source /opt/ros/humble/setup.bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
source install/setup.bash
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/erob-real-ros-log
ulimit -l unlimited
ulimit -r 80
ros2 run erob_tools erob_inspect config/robot_real.yaml discover
```

If privilege/limit setup fails, resolve it before attempting real activation. Do not set `require_realtime: false` just to suppress the failure. Permissions alone do not establish that this computer can meet a 1 ms cycle.

Discovery opens the network exclusively, changes EtherCAT states, reads identities and derives topology. It does not enable drive motion. On exit it returns the network to INIT. It is not a passive network sniffer.

Save the printed slave index/address, physical path, vendor/product/revision/serial and firmware for every motor and infrastructure device. The first run can print useful discovery data and then exit with code 2 / `Profile match unavailable`, because the expected entries are still synthetic. An earlier SDO/topology error requires diagnosis before proceeding; do not assume it is the same expected mismatch.

Update all 15 motor entries in the user-owned YAML from discovery and physical cable tracing. Do not equate `logical_id: 0x01` to slave index 1. Do not assume there are exactly 16 slaves or that an external hub port label equals an ESC port number. Non-DC infrastructure currently assumes upstream ESC port 0; confirm this with the hub documentation. Set `require_serial: true` only where the expected serial is reliable; topology matching remains mandatory when serials are absent or duplicated.

Repeat discovery until it prints the correct full 7+5+3 mapping without a mismatch. Then inspect and optionally read selected parameters, still with no ROS hardware master running:

```bash
ros2 run erob_tools erob_inspect config/robot_real.yaml inspect
# Replace 2 with the DISCOVERED EtherCAT slave index of the intended motor:
ros2 run erob_tools erob_inspect config/robot_real.yaml read 2 0x6064 0 4
ros2 run erob_tools erob_inspect config/robot_real.yaml read 2 0x6041 0 2
ros2 run erob_tools erob_inspect config/robot_real.yaml read 2 0x603f 0 2
```

These read position, status and error code. The CLI prints raw unsigned hexadecimal. Position 0x6064 is signed 32-bit: when the parsed number is at least 2^31, subtract 2^32 before treating it as a signed position count.

## 5. Complete the real profile before enabling anything

The profile must record and verify:

| Data | Required interpretation |
| --- | --- |
| Hub model, ESI reference, topology | Actual junction and physical branch/port correspondence |
| Motor identity and firmware | Each of the 15 configured motor locations |
| Position and velocity scales | Verified output-joint counts/rad and counts/(rad/s) |
| Direction and zero_counts | Joint sign and raw feedback count at defined mechanical zero |
| Position, speed and raw torque limits | Values appropriate to each joint and supported by its drive |
| Quick-stop and disable timing | Actual stop/brake behavior for this mechanism |
| CSP hold acceleration, jerk and tracking error | Positive verified `hold_acceleration`, `hold_jerk`, `hold_tracking_error`; set `hold_verified` only after verification |
| Drive communication watchdog | Verified response when communication is lost |
| PDO mapping/padding and DC period | Firmware/ESI compatibility, initially requested 1 ms |

For CSP leave `effort_verified: false`. Physical Nm calibration is required for effort feedback or library CST, not for exporting the ROS position command interface. Raw current/torque limits are still required for CSP. The 20 ms command timeout and five invalid-feedback cycles originated as mock defaults; verify appropriate real thresholds rather than treating them as commissioned values.

At a known, secured mechanical reference, establish `zero_counts` from signed 0x6064 using the manufacturer's permitted procedure. Do not force a braked joint to a fixture by hand. Choosing zero_counts=0 means ROS zero is the drive's count zero; it does not set the current posture to zero. Confirm how multi-turn position and reference survive a power cycle. Do not reuse a reference that the drive no longer preserves.

Only after verification populate the actual `hub_model` and `esi_reference`, set `topology_verified`, `stop_verified`, `watchdog_verified`, `padding_verified` true, and set each motor's `verified` true (or a shared profile flag only when every motor is covered). These are operator assertions, not automatic certifications. The driver records the ESI reference; it does not import an ESI and prove the physical installation from it.

**Pause here if these data are unavailable.** The current template is deliberately incomplete. Real launch/configuration checks the verification gates before programming the motion PDO/stop profile.

## 6. Start the hardware inactive and check feedback

In Terminal A, after inspection has exited:

```bash
ros2 launch erob_bringup robot.launch.py \
  backend:=soem \
  config_file:=/home/xurui/downloads/Robot_motor/erob_soem_ws/config/robot_real.yaml
```

Expected: configuration succeeds and ErobSystem remains **inactive**. Configuration already performs SDO writes for mappings/limits/stop settings, establishes DC and runs EtherCAT OP with the drives disabled. It is not a read-only launch. No controllers are automatically spawned in real mode.

In ordinary-user Terminal B, set up the same ROS environment/domain:

```bash
source /opt/ros/humble/setup.bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
source install/setup.bash
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/log_real_client"
ros2 control list_hardware_components
ros2 run controller_manager spawner joint_state_broadcaster
ros2 topic echo --once /joint_states --qos-reliability best_effort
ros2 topic echo --once /diagnostics
```

Check all 15 names against actual hardware, valid feedback, matching WKC/expected WKC, no drive error and plausible stationary positions. Disabled does not necessarily mean every gravity-loaded joint is stationary; the mechanical support must establish that condition. Numerical zeros alone are not evidence of valid feedback. Stop if discovery, scaling, direction, position plausibility or diagnostics are inconsistent.

## 7. Load a controller for only the first joint, then enable the hardware

The supplied [commissioning controller YAML](commissioning_position_controller.yaml) claims only motor_01_joint/position. It does not claim the other 14 command interfaces. Keep the standard all-joint position controller inactive during this test. ROS velocity/effort command interfaces are not exported.

In Terminal B:

```bash
ros2 run controller_manager spawner commissioning_position_controller \
  -t joint_trajectory_controller/JointTrajectoryController \
  -p "$PWD/doc/commissioning_position_controller.yaml" \
  --inactive
ros2 control list_controllers
```

When ready for **all 15 drives to enable and their brakes to follow the normal enable sequence**, activate the hardware:

```bash
ros2 control set_hardware_component_state ErobSystem active
ros2 control list_hardware_components
ros2 topic echo --once /diagnostics
```

The driver seeds measured positions and waits at least 500 ms with all drives enabled. It should hold the current posture in CSP. The other 14 drives also enable and hold their seeded targets. If the test requires only one drive to be physically enabled, this version's activation behavior needs a separate change; do not treat the one-joint controller as such isolation.

After stable holding feedback is confirmed:

```bash
ros2 control set_controller_state commissioning_position_controller active
ros2 control list_hardware_interfaces
ros2 topic echo --once /joint_states --qos-reliability best_effort
```

Expect motor_01_joint/position claimed, without simultaneous velocity/effort claims. No substantial posture change should occur merely from claiming the interface.

## 8. Send one small absolute CSP goal

Read motor_01_joint's current angle by matching its name to the position array in fresh /joint_states. Select a small, permissible move from that measured angle. Example only: if the current value is 0.300 rad, a +0.010 rad test means an **absolute target of 0.310 rad**. That increment and a three-second duration are not universal safe values; the mechanism and verified limits determine the test.

Enter the selected absolute destination in Terminal B:

```bash
read -r -p 'Verified absolute target for motor_01_joint, in radians: ' TARGET_RAD
ros2 action send_goal \
  /commissioning_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [motor_01_joint], points: [{positions: [$TARGET_RAD], velocities: [0.0], time_from_start: {sec: 3}}]}}" \
  --feedback
```

The [Humble trajectory controller](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) generates intermediate setpoints and exposes this action interface. The hardware driver sends cyclic absolute targets to 0x607A. ROS commands remain radians, even with zero_counts=0.

Watch the physical direction, feedback, drive errors and the other supported joints. Re-read /joint_states and /diagnostics after completion. An accepted/succeeded action alone does not prove correct physical calibration or tracking. Record the measured start, commanded target, final measured position and observed motion.

If the first step is correct, command the recorded original absolute angle with another deliberate slow goal. Do not use a guessed zero as the return position. Interrupting the action CLI or losing an upstream publisher is not a reliable motor-stop command; the controller may continue its goal or hold its last target.

## 9. Stop normally and shut down

Once the joint has completed the move and is stationary, with the robot mechanically supported, use Terminal B:

```bash
ros2 control set_controller_state commissioning_position_controller inactive
ros2 control set_hardware_component_state ErobSystem inactive
ros2 control list_hardware_components
ros2 topic echo --once /diagnostics
```

Stopping the controller alone leaves CSP holding. Hardware deactivation performs the communicating quick-stop/disable sequence and waits through the configured >300 ms interval. Confirm inactive/disabled feedback and the actual brake/support state before ending the master. Then press Ctrl-C in Terminal A and follow the robot's normal power-off procedure. Do not begin normal shutdown by killing the process, disconnecting Ethernet or removing motor power.

On unexpected motion, use the independent hardware stop. On command expiry or initial feedback/deadline thresholds, the driver first latches a powered hold. After resolving the cause, pause upstream publishers, wait for confirmed holding, and run `ros2 run erob_bringup resume_control.py`; then send a new trajectory. See [hold_recovery.md](hold_recovery.md) for the exact three-times escalation thresholds and required controller reset. Hard faults still latch a whole-group stop and do not automatically resume on reconnection. Record diagnostics, secure the mechanism, resolve the cause and explicitly reconfigure/reactivate from a controlled state; do not repeatedly try enable commands. PDO state/error inspection tools must wait until the ROS master has exited, because both require the same exclusive bus lock. Never delete the lock file to get around a running master.

## 10. Repeat for other motors

Keep the hardware inactive between initial per-motor commissioning tests. Deactivate and unload the commissioning controller:

```bash
ros2 control unload_controller commissioning_position_controller
```

Change its YAML joint name to motor_02_joint, reload it inactive, then repeat the feedback/enable/small-goal/stop sequence. Change both the YAML and action joint name consistently through motor_0f_joint. The logical-to-physical mapping remains the full 15-motor profile.

Use the normal 15-joint position_controller for coordinated trajectories only after each joint, branch mapping and coupled robot motion have been commissioned. Branch-loss/brake and stopping-profile tests require their own verified procedures; this first CSP test does not validate them. CST is a library capability only.

## Current limits and common failures

- No slaves: wrong NIC, wrong cable/port, power/link problem, raw-socket privilege, or incompatible hub. No IP ping test proves EtherCAT readiness.
- Profile mismatch: inspect discovered topology and identities; do not guess slave indexes from hexadecimal labels.
- SDO/PDO/DC failure: compare installed firmware/ESI and the manual. First hardware bring-up may require driver adjustments; do not suppress readback or size checks.
- Real-time setup failure or deadline faults: resolve permissions and actual host scheduling/load. Passing a mock demo is not a 15-drive real-time qualification.
- Unexpected mode: the ROS plugin requires CSP; resolve the drive mode mismatch before lifecycle recovery.
- Lock unavailable: stop the existing owner; do not unlink the lock file.

The mechanical support, watchdog, brake timing, torque calibration and actual 15-drive timing remain physical commissioning work. This guide does not claim those tests have already passed.

# eRob ROS 2 Humble workspace

A single driver and `erob_hardware/ErobSystem` plugin control 15 logical motors on one EtherCAT master network: A = 0x01–0x07, B = 0x08–0x0C, C = 0x0D–0x0F. The four-port hub is provisionally an EtherCAT junction. Its model, port layout, ESI, motor identities, scaling, watchdog and stop behavior still require commissioning.

**The installed example defaults to a fake EtherCAT transport.** Its encoder scale, Nm calibration, identities and topology paths are synthetic test data. The real template is deliberately unverified and cannot configure motion hardware. No physical interfaces are opened by the tests.

| Package | Purpose |
| --- | --- |
| `erob_soem_vendor` | Vendored, pinned SOEM v1.4.0; no network download during build |
| `erob_driver` | C++17 protocol, lifecycle, conversions, limits, worker, SOEM and fake transports |
| `erob_hardware` | CSP-only Humble SystemInterface, powered hold, explicit resume and diagnostics |
| `erob_tools` | Exclusive-bus discovery, typed SDO reads, inspection and allowlisted offline gains |
| `erob_bringup` | 15-joint URDF example, profiles, controllers, launch and mock acceptance client |
| `wheel_human_interfaces` | Public ownership, discrete joint/Cartesian action and status contracts |
| `wheel_human_commander` | Whole-robot ownership, discrete goal preparation and monitored execution |
| `erob_controllers` | CSP quintic controller with synchronized velocity/torque feedforward and heartbeat stop |
| `wheel_human_description` | Robot kinematics, dummy geometry, RViz and standalone display launch |
| `wheel_human_bringup` | Robot-specific motor mapping, controllers, validated Xacro composition and launch |

For the new robot, see [wheel human package usage](doc/wheel_human.md) and the
[commander/feedforward contract](doc/commander.md).

The old demonstration is preserved in `archive/soem_master` under `COLCON_IGNORE`. The sibling `wheel_legged_motors` is unchanged. The supplied PDF is preserved at the workspace root.

See [software validation](doc/validation.md) and the [CSP hold/recovery contract](doc/hold_recovery.md).

## Build and test

Calculation-only Pinocchio FK/IK is documented in the [kinematics guide](doc/kinematics.md).
The installed `Kinematics_fixed` CAD, upright reference pose, and workspace-owned
noVNC viewer are documented in the [CAD description guide](doc/cad_description.md).

The on-robot deployment is **Ubuntu 22.04 x86-64 with system-wide ROS 2 Humble**. Follow the [native installation guide](doc/native_humble_install.md) for apt repository setup, dependencies and rosdep. ROS is installed under /opt/ros/humble; the eRob packages are a colcon workspace overlay.

On the robot, in a fresh ordinary-user Bash terminal:

```bash
source /opt/ros/humble/setup.bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
bash scripts/setup_pinocchio.sh  # Once per underlay; isolated pinned dependency
source scripts/pinocchio_env.sh
bash scripts/test_software.sh
source install/setup.bash
```

The script defaults to native Humble and uses the standard `build/` and `install/`
directories, with logs under `log/native/`. It selects system Python/compiler
tools and rejects active conda/virtualenv environments. Build from source on the
target; do not copy or source the server's generated `artifacts/dev/install/` tree.

On the development server only, explicitly activate the existing environment and choose the development mode:

```bash
source /home/xurui/miniconda3/etc/profile.d/conda.sh
conda activate ros_humble
bash scripts/setup_pinocchio.sh  # Once per underlay; isolated pinned dependency
source scripts/pinocchio_env.sh
bash scripts/test_software.sh --current
source artifacts/dev/install/setup.bash
```

Development builds and installs live under `artifacts/dev/`; their logs live under
`log/dev/`. See [workspace outputs](doc/workspace_outputs.md) for the directory
layout and migration details. The resolved environment is recorded in
`environment/`. The conda lock files describe development tests and are not robot
deployment requirements. [Recorded validation](doc/validation.md) distinguishes
tested development results from pending native acceptance.

The mock checks start their own launch groups, verify all 15 positions using a trajectory controller, test one-joint commissioning, and exercise timeout hold and explicit recovery. They shut down their child processes. Logs are written under `log/<profile>/tests/` (`profile` is `native` or `dev`). ROS tests need local DDS sockets; they do not need raw Ethernet privileges. Keep the test domain separate from a robot.

For an interactive mock:
```bash
ros2 launch erob_bringup robot.launch.py
# In a second shell with the same environment and overlay:
ros2 run erob_bringup mock_demo.py
```

The position trajectory controller initially loads inactive. The client activates it explicitly. The URDF contains placeholder joints attached to a common base; it demonstrates interfaces and is not the robot's kinematic model.

## Control contract

The one worker owns all SOEM calls and its private context. It runs an absolute monotonic schedule with bounded DC phase correction, optional CPU affinity, FIFO priority and locked memory. It allocates its cyclic storage before running. Preallocated SPSC triple buffers for commands and state snapshots separate the worker from ROS. `read()` and `write()` do no bus I/O. The driver API has one command producer and one snapshot consumer; lifecycle calls must be serialized with those calls.

Configuration discovers and matches every configured motor by physical port path plus vendor/product/revision, and optional serial. Infrastructure slaves are counted separately. Duplicate indexes, ambiguous paths, missing motors, identity mismatches, lack of required modes or DC support prevent configuration. Discovery order does not define logical IDs. Repeated/zero serials are permitted when serial matching is disabled; topology remains mandatory.

Drives enter OP while disabled. Activation seeds measured position and zero velocity/effort, follows feedback-confirmed CiA 402 states, and waits at least 500 ms after all drives report enabled. The ROS plugin exports position commands by default; the opt-in feedforward profile adds velocity_offset and torque_offset commands in CSP. Unclaimed CSP joints retain their position targets. CSV/CST remain standalone library capabilities, not ROS command interfaces.

The ROS plugin requires CSP throughout active control. Library mode changes still require stationary feedback and displayed-mode acknowledgment. Deactivation and fault stops continue communicating during the configured quick-stop/disable interval (>300 ms). Normal control words operate the brake; there is no manual brake-release API.

All motors share one hold/fault group. CSP command expiry, feedback loss and repeated deadline misses first latch a recoverable powered hold; persistent feedback/deadline failures escalate at three times the initial threshold. Drive faults and unsafe stop/tracking conditions remain hard faults. Reconnection and fresh commands never automatically resume motion. Use `ros2 run erob_bringup resume_control.py` for healthy confirmed holds; hard faults still require explicit lifecycle recovery after resolving the cause. No automatic drive-fault-reset bit is sent. See [hold recovery](doc/hold_recovery.md) for thresholds, required profile limits and controller reset behavior.

Every submitted command is validated before publication; bursts of valid snapshots are coalesced to the latest complete command. This accommodates controller-manager catch-up after lifecycle transitions without accumulating stale commands. An invalid snapshot is rejected without changing the previous command or refreshing its timestamp. Rejection is reported separately from driver unavailability.

The 20 ms command timeout watches the controller-to-driver handoff. A controller repeatedly writing an unchanged command is alive from the driver's perspective. Upstream publisher loss must be handled by the controller/application. Feedback invalidity appears immediately in diagnostics; the configured consecutive-cycle/age threshold latches unconfirmed hold, with a hard fault at three times either threshold. Complete bus loss cannot deliver a stop command and requires a verified drive watchdog.

States are rad and rad/s. Calibrated effort is exposed only when `effort_verified` is true; the raw 0x6077 rated-current ratio is retained separately. It is not a torque-sensor measurement. The mock's 524288 counts/revolution is output-side: do not multiply by an internal gearbox ratio again.

## Commissioning and tools

Follow the [first real-robot test guide](doc/real_robot_first_test.md) for installation, disabled discovery, calibration requirements, activation, a one-joint CSP test and shutdown.

See [protocol and configuration notes](doc/implementation.md). Discovery/inspection open the selected interface exclusively and put the network through SOEM discovery; stop all other masters first. Tools and driver share `/tmp/erob-ethercat-INTERFACE.lock`; the lock file is intentionally never unlinked. Other unrelated EtherCAT software must also be kept off this interface.

```bash
ros2 run erob_tools erob_inspect src/erob_bringup/config/robot_mock.yaml discover
ros2 run erob_tools erob_inspect src/erob_bringup/config/robot_mock.yaml inspect
# slave index, object index, subindex, width in bytes:
ros2 run erob_tools erob_inspect src/erob_bringup/config/robot_mock.yaml read 2 0x1018 1 4
# offline, allowlisted UINT16 gain with readback:
ros2 run erob_tools erob_inspect src/erob_bringup/config/robot_mock.yaml gain 2 0x2381 1 42
```

Use a copy of `robot_real.template.yaml` for discovery with the real NIC. Replace all synthetic identities and paths with measured values. Complete the hub/ESI, motor calibration, drive limits, padding, stop and watchdog verification before marking the relevant flags true. Real PDO configuration is gated as well as activation, because guessing a stop or mapping profile is not safe. Raw Ethernet access and the configured real-time permissions are deployment requirements.

```bash
ros2 launch erob_bringup robot.launch.py backend:=soem config_file:=/absolute/path/verified_robot.yaml
```

Real hardware starts **inactive**; controllers are not automatically spawned or activated. Incorporate this component into the actual robot description and controller configuration after commissioning. Test 15-drive cycle performance, every branch-loss response, braking under load and torque calibration on hardware before robot operation.

Optional wrist-only Linker SDK integration is documented in
[doc/wrist_integration.md](doc/wrist_integration.md). It defaults to disabled;
`commander.launch.py wrist_mode:=mock` enables software-only wrist testing with
independent wrist fault handling and fingers remaining in the SDK.

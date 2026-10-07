# Explicit-model FK and IK

The current CAD is now installed in `wheel_human_description`. Use
`ros2 launch wheel_human_commander cad_kinematics.launch.py` for that model and
`ros2 run wheel_human_commander reference_pose_example.py` to query its saved
upright reference. See the [CAD and reference-pose guide](cad_description.md).
The explicit-file examples below remain supported for testing other exports.

`wheel_human_commander` exports the C++ `wheel_human_kinematics` library and a
standalone `kinematics_node`. The node calculates poses and joint solutions; it
has no state subscriptions, controller clients, SDK nodes, or command publishers.
It does not require a motor profile or ownership token. `move_to_pose` execution
still returns `NOT_IMPLEMENTED`; commander capability flags remain unchanged.
Gravity compensation, collision checking, and trajectory planning are separate
unfinished features.

## Build on this server

In a fresh Bash shell:

```bash
source /home/xurui/miniconda3/etc/profile.d/conda.sh
conda activate ros_humble
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
bash scripts/setup_pinocchio.sh
source scripts/pinocchio_env.sh
bash scripts/test_software.sh --current
source artifacts/dev/install/setup.bash
```

The dependency script pins Pinocchio 3.8.0 at commit
`655877b314baed68c7e2d4dd56b0a0200bb9f98e`. It builds against the active underlay,
with C++17 and URDF support, without Python bindings, collision support, examples,
or benchmarks. Source/build/install directories are under `artifacts/dev/deps`;
it does not replace packages in `ros_humble`. Default build parallelism is two;
`PINOCCHIO_BUILD_JOBS` overrides it. `test_software.sh` sources the dependency
environment but never downloads dependencies automatically.

For a native Humble target, use a fresh native underlay and run the same dependency
setup; its separate prefix is `artifacts/native/deps`. Rebuild the workspace
natively. Conda build results are not native deployment acceptance.

## Run calculation-only queries

```bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_human_commander kinematics.launch.py \
  model_file:="$(ros2 pkg prefix wheel_human_description)/share/wheel_human_description/urdf/kinematics_fixed.urdf"
```

In a second shell with the same underlay, dependency environment, workspace
overlay and domain:

```bash
ros2 run wheel_human_commander kinematics_example.py \
  --model-file "$(ros2 pkg prefix wheel_human_description)/share/wheel_human_description/urdf/kinematics_fixed.urdf"
```

The example uses bent-arm seeds, queries both TCPs, solves both target modes,
and verifies solutions with FK. Its numerical test angles are intended for the
current provisional CAD model; adjust them if a future model has narrower limits.
It sends no motor commands. Model input is mandatory; there is no fallback to the
body dummy model. The launch expands URDF/Xacro once and passes the exact XML to
the node. Restart after replacing the model or solver settings. Startup logs show
the Pinocchio version, FNV-1a 64-bit model fingerprint, joint count and TCP groups.
The fingerprint identifies the loaded XML; it is not a security hash.

## Service contracts

| Endpoint | Inputs | Outputs |
| --- | --- | --- |
| `/wheel_human_kinematics/compute_fk` | `configuration` JointState; `target_frame` URDF link name | code/message; pose in `base_link` |
| `/wheel_human_kinematics/compute_ik` | complete `seed` JointState; `planning_group`; `target_mode`; target PoseStamped | code/message; full named solution; achieved pose; residuals; iterations |

Every request must provide every movable joint exactly once (currently 19), using
names and positions in SI units. JointState velocity, effort, and timestamp fields
are unused. Fixed joints, including the two TCP joints, have no configuration
entries. Names are mapped explicitly; motor YAML order is irrelevant. Missing,
duplicate, unknown, non-finite, or out-of-position-limit values are rejected.
Only independent bounded revolute/prismatic joints are supported by this version.
FK accepts URDF link names, not joint names.

IK targets must explicitly name `base_link` as their frame. No TF lookup or live
state fallback occurs. Target timestamps are unused. `POSITION_ONLY=0` constrains
XYZ and ignores orientation, including a zero quaternion. `FULL_POSE=1` also
constrains orientation and requires a finite nonzero quaternion, which is
normalized. Calculated poses have zero timestamps because they are not live
measurements. A solution is a geometric result, not an executable or
collision-checked trajectory.

`left_arm` varies `Left_Shoulder_Joint`, `Left_Arm_Joint1` through `4`, and
`Left_Wrist_Joint`, targeting `Left_Hand_TCP`; `right_arm` is symmetric. All
other joints remain exactly at their seed positions. Group settings are in
`config/kinematics.yaml`; `kinematics_config:=/path/to/file.yaml` overrides them.
Group joints must be unique and on the configured tip's ancestor chain.

Result codes are `SUCCESS=0`, `INVALID_INPUT=1`, `NO_CONVERGENCE=2`, `TIMEOUT=3`,
and `INTERNAL_ERROR=4`. Failed IK results have an empty solution. Residuals and the
achieved pose describe the last evaluated iterate when available, not an approved
alternative. Unavailable residuals are NaN; orientation error is NaN in
position-only mode. The achieved pose is valid only when its frame is populated.

## Solver and model limitations

The solver uses damped least squares, frame-consistent Jacobians, SE(3) logarithm
correction for pose targets, active bound handling, and backtracking. It uses one
explicit seed with no random restarts. A bounded local solver may fail even when
another seed could reach the target. The CAD zero posture is poorly conditioned;
bent configurations are better initial round-trip tests.

Startup defaults: 200 iterations, 100 ms steady-clock budget, 0.1 rad maximum
revolute-joint step (0.1 m for prismatic joints), 0.1 mm position tolerance,
0.001 rad orientation tolerance, 0.1 m/rad orientation weighting, and 0.001
damping. Success checks physical position and angle errors separately through
FK. Time limits are checked between operations; this is not a hard real-time
execution guarantee. Calls use a single-threaded executor. C++ callers must
serialize access to a model instance or create a separate instance per worker.

C++ consumers include `wheel_human_commander/kinematics.hpp` and link the exported
target without including Pinocchio headers:

```cmake
find_package(wheel_human_commander REQUIRED)
target_link_libraries(your_target PRIVATE wheel_human_commander::wheel_human_kinematics)
```

Construct `KinematicsModel` with expanded URDF XML, planning groups, and optional
`IkOptions`; call `forward`, `jacobian`, or `inverse` with named configurations.
Keep the dependency prefix available at runtime using `scripts/pinocchio_env.sh`.

The current CAD model's position bounds are provisional +/-180 degrees and its
TCP offsets are provisional +0.05 m in each hand's local X direction. Its zero
velocity/effort limits are accepted for these geometric calculations; they do not
provide usable motion or torque limits. No control model, motor mapping, or
physical configuration is changed by this integration.

## Acceptance

```bash
python3 scripts/check_kinematics.py \
  --model-file "$(ros2 pkg prefix wheel_human_description)/share/wheel_human_description/urdf/kinematics_fixed.urdf"
```

This isolated domain-188 check runs the query node, both IK modes, failure cases,
and a separate robot_state_publisher reference driven by explicit joint states.
It compares both TCP frames at two wrist angles and cleans up its processes.
It does not operate the existing domain-185 CAD viewer. Run it in a DDS/socket-
capable environment. Logs go to `log/dev/tests/kinematics` by default; set
`EROB_TEST_LOG_DIR` to a fresh directory to preserve previous evidence.

Colcon includes analytic FK/TCP tests, finite-difference Jacobian checks, both-arm
IK tests, bounds, malformed input, singular/unreachable cases, and budget failures.
All acceptance here is software-only.

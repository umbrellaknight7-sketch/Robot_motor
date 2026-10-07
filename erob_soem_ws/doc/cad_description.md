# Packaged CAD and upright reference

`wheel_human_description` contains the imported `Kinematics_fixed` CAD, its mesh
files, two virtual hand TCPs, and a complete named `upright` pose. Everything
needed for display and FK/IK is installed in the ROS package. The source export
under `wheel_human_urdf` is preserved and is not required at runtime.

This is a **visually confirmed upright reference**. It is not a measurement of
physical perpendicularity, a new URDF zero convention, encoder-zero calibration,
or a startup motion command. Body-control bringup still defaults to its existing
dummy model. Motor profiles and physical startup behavior are unchanged.

## Reference configuration

The authoritative file is `wheel_human_description/config/poses/upright.yaml`:

- `Left_Arm_Joint3 = -1.5707963267948966` rad (exact double-precision -pi/2).
- `Right_Arm_Joint3 = -1.5707963267948966` rad.
- All other 17 movable joints are explicitly set to `0.0` rad.
- Both fixed TCP joints have no configuration entries.

The YAML identifies its name (`upright`), model (`Kinematics_fixed`), units
(`radians`), and `joint_positions` mapping. The shared
`wheel_human_description.poses.load_pose(path, description)` loader returns a
complete name-to-float mapping after checking model identity, duplicate YAML keys,
missing/unknown joint names, finite numeric values, and position limits. Joint
ordering is independent of motor YAML order. `zero_pose(description)` explicitly
constructs the CAD all-zero posture; it does not mean upright.

The confirming [original screenshot](media/upright-reference-20260927.png) used
GUI values of -1.570628 rad for both joints (approximately -89.9904 degrees),
with the other joints zero. Its full [observed values](media/upright-observed-20260927.json)
are retained. The saved reference uses the intended exact -90 degrees instead
of copying this slider approximation.

The [installed-model screenshot](media/upright-installed-20260927.png) confirms
the packaged CAD renders in the same upright pose with RViz Global Status OK.

The GUI represents each [-3.14, 3.14] joint range with 10,000 steps. A small
workspace-owned wrapper rounds to the nearest slider step, avoiding repeated
truncation during the upstream GUI's refresh. Startup and Center differ from the
reference by at most half a step (one step is 0.000628 rad, about 0.036 degrees).
Headless publication and the reference-pose query client use the exact
saved values. GUI Center restores the selected startup reference, not necessarily
all-zero CAD angles.

## Build and use

Use the existing Humble environment and pinned Pinocchio prefix as described in
[kinematics.md](kinematics.md). Build the workspace with
`bash scripts/test_software.sh --current`, then source
`artifacts/dev/install/setup.bash`. The CAD package installs its Python pose
helpers through `ament_cmake_python`; `python3-yaml` and the standard joint-state
publisher packages are runtime dependencies.

On a graphical machine, or inside the configured remote viewer environment:

```bash
ros2 launch wheel_human_description cad_display.launch.py
# Alternative CAD zero posture:
ros2 launch wheel_human_description cad_display.launch.py initial_pose:=zero
# Exact upright state publication without a GUI:
ros2 launch wheel_human_description cad_display.launch.py gui:=false rviz:=false
```

The display launch offers `initial_pose:=upright|zero`, `gui:=true|false`, and
`rviz:=true|false`. Defaults are upright, GUI on, and RViz on. It runs only the
joint-state publisher, robot-state publisher, and optional RViz. It removes root
inertia from the in-memory display description for KDL; the installed URDF keeps
all exported inertial data. Use a separate ROS domain from motor control.

For queries, use the CAD-specific preset in one terminal:

```bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_human_commander cad_kinematics.launch.py
```

In another terminal with the same environment and domain:

```bash
ros2 run wheel_human_commander reference_pose_example.py
```

The client loads the installed reference YAML, sends all 19 named joint positions
explicitly, and prints both TCP poses relative to `base_link`. Optional
`--model-file` and `--pose-file` arguments select matching local files; the server
must be launched with the same model. The generic `kinematics.launch.py` still
requires `model_file`. The CAD preset accepts `kinematics_config` for solver/group
settings. Services and their explicit-state/explicit-seed requirements are
unchanged; the server never silently assumes upright.

## Remote noVNC viewer

From the workspace root, using the existing server-local viewer dependencies:

```bash
bash scripts/rviz_web.sh start           # upright
bash scripts/rviz_web.sh status
bash scripts/rviz_web.sh stop
bash scripts/rviz_web.sh start zero     # optional alternative initial pose
```

These workspace-owned scripts use `ros_humble`, the previously installed GUI
dependency overlay under `artifacts/dev/rviz_viewer/install`, and then the main
workspace overlay. They launch the installed CAD package directly; they do not
use the sibling model package or generate temporary URDFs. The existing viewer
runtime must already be installed. They do not download tools or change conda
packages. `RVIZ_CONDA_ROOT` can select a different conda installation.

The persistent session remains `tmux -L wheel-rviz`, session `kinematics-rviz`,
display `:99`, and isolated ROS domain 185. Ports remain bound to localhost:
VNC 5901, noVNC 6080. The existing VNC password is reused. On your computer:

```bash
ssh -N -L 6080:127.0.0.1:6080 YOUR_SSH_HOST
```

Open <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>. Retrieve the
password in your server terminal from
`artifacts/dev/rviz_viewer/run/vnc_password.txt`. Logs and screenshots are in
`log/dev/rviz_viewer`; runtime `active_model.json` identifies the installed model,
hash, and initial pose. Stop before switching startup poses. No controller,
EtherCAT process, or hand SDK is launched.

## Import provenance and checks

`config/kinematics_fixed.provenance.json` records source archive/URDF hashes,
the installed URDF hash, and every mesh hash. Import changes only mesh URI prefixes
to `package://wheel_human_description/meshes/kinematics_fixed/...`. Both fixed TCPs
retain their provisional local +0.05 m X offsets. There are 22 links, 19 revolute
joints, and two fixed joints. All exported origins, axes, inertias, and +/-3.14
rad position limits are preserved. Zero velocity/effort limits are accepted only
for geometry; this is not a calibrated control or gravity-compensation model.

After sourcing the underlay, GUI dependency overlay when needed, Pinocchio prefix,
and main workspace overlay:

```bash
python3 scripts/check_cad_description.py
python3 scripts/check_kinematics.py --model-file \
  "$(ros2 pkg prefix wheel_human_description)/share/wheel_human_description/urdf/kinematics_fixed.urdf"
```

The first check uses domain 189 and verifies installed resources, exact upright
and zero publication, root-inertia handling, both TCPs against FK, and the example
client. The second uses domain 188 for both-arm/both-mode IK and failure cases.
Both clean up their own ROS processes. Set `EROB_TEST_LOG_DIR` to a fresh directory
to retain each run's evidence. Colcon tests cover the pose loader and CAD contract.

For the running noVNC session, with nobody manipulating the GUI:

```bash
source scripts/rviz_viewer/environment.sh
export DISPLAY=:99
export XAUTHORITY="$viewer_artifacts/run/Xauthority"
export XDG_RUNTIME_DIR="$viewer_artifacts/run/runtime"
python scripts/rviz_viewer/check_cad_viewer.py --initial-pose upright --test-center
```

The GUI check expects the standard startup window layout, temporarily moves the
simulated Feet joint, restores the selected reference through Center, validates
the display-only graph and authenticated noVNC connection, and captures a screen.
All validation described here is software-only.

Dense CAD meshes can take time to load with software OpenGL. Inspect the captured
image and recapture after loading if it still shows an incomplete RViz canvas;
successful joint-state checks alone do not prove the meshes have finished rendering.

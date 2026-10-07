# ROS 2 Kinematics-only viewer

ROS 2 package name: `kinematics_only` (lowercase). The source directory and URDF
filename remain `Kinematics_only`. Mesh URIs in the URDF/CSV now use
`package://kinematics_only/meshes/...`.

This is a display-only package: `robot_state_publisher`,
`joint_state_publisher_gui` and `rviz2`. It does not start ros2_control,
EtherCAT, the commander, or hand SDK. The GUI publisher supplies simulated
joint angles; `joint_state_broadcaster` is a hardware-state broadcaster and
is not needed for these sliders.

## Browser connection

On your own computer, forward the remote server's localhost port:

```bash
ssh -N -L 6080:127.0.0.1:6080 YOUR_SSH_HOST
```

Use the same SSH host/alias that you normally use to access this server. Leave
that terminal running. Open:

<http://localhost:6080/vnc.html?autoconnect=true&resize=scale>

Retrieve the VNC password in your server terminal:

```bash
cat /home/xurui/downloads/Robot_motor/erob_soem_ws/artifacts/dev/rviz_viewer/run/vnc_password.txt
```

RViz is on the left; joint sliders are on the right. Scroll the sliders to
reach the right arm. The values are radians. Use **Center** to return all
joints to zero. RViz mouse controls rotate/pan/zoom the model. This is a
1600x1000 virtual X screen with a small window manager, not a full desktop.
Rendering uses software OpenGL, so dense CAD meshes can limit frame rate.

Both the VNC server (5901) and noVNC HTTP/WebSocket service (6080) bind only to
127.0.0.1. VNC requires the generated password. SSH provides the encrypted
remote connection. The X server does not listen on TCP and uses Xauthority.

## Preview ranges

The CAD export has zero lower/upper joint limits. The display launcher defaults
to `preview_limits:=true`, replacing only zero-width revolute ranges **in memory**:

- Body and wrist joints: -pi/2 to +pi/2 radians.
- Wheel joints: -pi to +pi radians.

These are visualization ranges, not mechanical or motor-control limits. The
stored URDF retains its original joint limits, axes, transforms and inertia.
The root-link inertia is omitted only from the display copy to satisfy KDL.
To inspect the exact exported joint ranges, use `preview_limits:=false`.

## Start, stop and status

From `/home/xurui/downloads/Robot_motor`:

```bash
bash wheel_human_urdf/Kinematics_only/tools/rviz_web.sh status
bash wheel_human_urdf/Kinematics_only/tools/rviz_web.sh stop
bash wheel_human_urdf/Kinematics_only/tools/rviz_web.sh start
```

The viewer runs in a dedicated tmux server (`tmux -L wheel-rviz`) and session
`kinematics-rviz`; it survives SSH disconnection, but does not automatically
restart after a reboot. Start refuses to replace an occupied display :99 or
ports 5901/6080. Use stop before restarting. If a child exits unexpectedly,
inspect the logs before starting again.

Logs: `erob_soem_ws/log/dev/rviz_viewer/`.
Build, dependencies and private runtime files:
`erob_soem_ws/artifacts/dev/rviz_viewer/`.

The viewer uses ROS domain 185 and `ROS_LOCALHOST_ONLY=1`. Keep this domain
separate from motor control. Its GUI joint states must never drive hardware.

## Rebuild/setup

The setup script uses the existing `ros_humble` conda environment, downloads
Ubuntu 22.04 tools into the workspace, extracts them without sudo, and builds
this package plus the official joint-state publisher packages (tag 2.4.0):

```bash
bash wheel_human_urdf/Kinematics_only/tools/setup_viewer.sh
```

No system services or full desktop environment are installed. System libraries
already present on this Ubuntu server are used by the extracted tools. The
optional `RVIZ_CONDA_ROOT` environment variable selects a different conda root.

For a machine that already has ROS 2 Humble, RViz and the joint-state publisher
packages, build this package normally with colcon. From an actual graphical
session, run:

```bash
ros2 launch kinematics_only display.launch.py gui:=true preview_limits:=true
```

The launch also accepts `model:=/absolute/path/to/model.urdf` and `gui:=false`.
Original ROS 1 launch/config files are retained in `legacy_ros1/` for reference
and are not installed. Gazebo simulation has not been ported or configured.

## Verification on this server

- All three viewer packages build successfully in conda Humble.
- RViz displays the mesh model with global status OK and OpenGL 4.5.
- Authenticated RFB/VNC negotiation succeeds through the noVNC WebSocket proxy.
- All 19 joint states and 19 transforms are present.
- Moving the shoulder GUI slider changes its angle and link transform; Center
  restores zero. No motor-control nodes appear in the isolated viewer domain.

Evidence: `log/dev/rviz_viewer/acceptance.json` and `rviz-initial.png`.
The optional `tools/check_viewer.py` repeats the network and slider checks after
sourcing `tools/environment.sh` and setting DISPLAY=:99 and XAUTHORITY to
`artifacts/dev/rviz_viewer/run/Xauthority`. Its GUI check expects the original
window layout and should only be run when nobody is interacting with the viewer.

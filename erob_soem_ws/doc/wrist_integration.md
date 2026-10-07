# Optional Linker wrist integration

The commander owns the body and the two wrist joints under its existing robot
lease. Fingers remain in the Linker SDK, outside this lease. A goal may contain
body joints **or** one/both wrist joints. Mixed body/wrist goals are rejected;
there is no synchronized cross-backend trajectory in this version.

```
MoveJoints -> wheel_human_commander -> CSP controller -> erob_hardware (15 joints)
                                 -> wheel_human_wrist_adapter (2 wrists)
                                      -> SDK wrist_bridge -> CAN-FD hand
SDK finger clients ------------------> SDK wrist_bridge (same command buffer)
```

The wrist adapter is a ROS node, not an EtherCAT or ros2_control hardware plugin.
Its feedback joins `/joint_states` as partial messages. Commander freshness is
tracked per joint; wrist messages cannot refresh or invalidate body feedback.
The EtherCAT/controller-manager configuration remains 1 kHz; SDK adapter and
mock hand polling run at 50 Hz. Real polling throughput depends on SDK CAN I/O.

## Build and mock launch

Only the SDK's headless `linker_hand_l30_v6` package is linked into `src/`.
The relative link expects `linkerhand-l30-sdk` beside `erob_soem_ws`; retain both
when moving the workspace. GUI packages are not built. This is a locally
extended SDK; preserve its local changes when updating the manufacturer repo.

```bash
conda activate ros_humble
cd erob_soem_ws
bash scripts/test_software.sh --current
source artifacts/dev/install/setup.bash
ros2 launch wheel_human_bringup commander.launch.py wrist_mode:=mock
```

`wrist_mode:=disabled` is the default. It launches no SDK/adapter nodes and
supplies no wrist joint states or wrist transforms. The dummy wrists remain
revolute in every mode; the legacy Xacro `wrists_enabled` argument does not
change their type. `Left_Wrist_Joint` and `Right_Wrist_Joint` use synthetic limits
[-1, 1] rad and a synthetic +Y axis. Neither wrist is an EtherCAT motor. The
standalone description also exposes these revolute wrists. Real geometry is
not inferred from this placeholder model.

The body CSP controller still starts inactive. Activate it explicitly using
the existing bringup workflow before acquiring commander ownership. Mock wrists
are available immediately, but cannot receive commander goals without that
ownership. Submit wrist targets through the existing `move_joints` action in
radians; no torque/gravity feedforward is supported for wrists.

## SDK extension and exclusive bus ownership

Use the new executable `ros2 run linker_hand_l30_v6 wrist_bridge` **instead of**
the original `linker_hand_l30_v6` executable for each integrated hand. Never run
both against the same hand. The bridge is the sole CAN owner. The original
SDK node and examples retain their legacy behavior when used independently.
The optional bridge exposes position feedback and finger position/settings
commands; the original tactile/info publishers are not mirrored in this minimal
mode. The manufacturer API remains available for future telemetry integration.

* `/cb_left_hand_control_cmd` (and right): SDK `JointState.position`, either 16
  or legacy 17 raw positions. Only entries 0–15 are applied; entry 16 is ignored.
* `/cb_left_wrist_control_cmd`: one raw wrist target; `header.frame_id` carries
  the opaque session returned by `/cb_left_wrist/arm`. These are internal adapter
  endpoints, not the application's ownership API.
* `/cb_left_wrist/stop`: revoke the session and request measured-position hold.
  This acknowledges command fencing, **not physical standstill**. The adapter
  separately waits for fresh stationary measurements before reporting a stop.
* `/cb_left_wrist/reset`: reset the SDK's latched wrist watchdog after fresh
  feedback. Applications use the adapter's reset endpoint below.
* `/cb_left_hand_state`: measured raw positions; timestamps are query-completion
  times. No zero velocity/effort is invented when SDK feedback is unavailable.
* `/cb_left_wrist/status`: SDK availability and latched watchdog errors.
* `/cb_hand_setting_cmd`: finger settings require `hand_type: left` or `right`;
  wrist speed/torque settings come from commissioned bridge parameters.

All position writes pass through one locked 17-entry buffer initialized from
measured positions. SDK timeout/stop changes only its wrist target. Finger
commands continue. Old wrist session tokens cannot be reused after stop.
A local 500 ms command/feedback watchdog covers adapter-process loss. An SDK
process or CAN link failure cannot guarantee a physical stop; this protocol's
physical stopping behavior must be commissioned.

## Fault isolation and recovery

A timeout during wrist execution aborts that wrist goal and requests local
hold. It does **not** fault the body, revoke the robot owner, or interrupt an
already-running body trajectory. The other wrist can remain usable. A failed
wrist stays unavailable after feedback returns; it never resumes the old goal.

A commander cancel/release/takeover waits for body hold and the wrist's stop
result. If wrist hold cannot be confirmed within the configured wrist timeout,
the wrist is quarantined and body ownership can still be released/transferred.
This deliberately differs from a body stop failure, which blocks handover and
faults the commander. Do not interpret a successful body handover as physical
standstill of an unavailable wrist. See `wrist_message` and per-wrist readiness.

After resolving a wrist error, request an explicit reset while idle:

```bash
ros2 service call /wheel_human_wrist_adapter/left/reset_fault std_srvs/srv/Trigger '{}'
```

This requests asynchronous SDK reset. Wait for the joint's `ready=true` in
`/wheel_human_wrist_adapter/status` or `CommanderStatus.wrist_ready` before a new
goal. Fresh stationary feedback and no pending SDK operations are required.
Use `/right/reset_fault` for the other hand. `CommanderStatus.ready` describes
body readiness; wrist availability is reported independently.

Wrist goal thresholds are startup parameters in the adapter configuration:
`goal_tolerance` (rad, default 0.02), `standstill_velocity` (rad/s, 0.03), and
`goal_time_tolerance` (s, 2.0). Success requires measured position and velocity
within those thresholds for 100 ms. Other startup parameters are
`feedback_timeout` (0.5 s), `heartbeat_timeout` (0.5 s), and `stop_timeout` (2 s).
Commander `wrist_stop_timeout` defaults to 3 s. Restart to apply parameter
changes; these values are not dynamically reconfigured. FJT per-goal tolerance
overrides are rejected explicitly. Ordinary goals use rest-to-rest quintic
position interpolation with velocity/acceleration feasibility checks.

## Real hardware remains a separate commissioning step

There is no installed hand here. The SDK's 17-channel protocol identifies the
wrist as index 16, but this does not establish compatibility with every L30 Pro
revision. Confirm the delivered device's firmware/protocol, joint count, wrist
channel and CAN addressing with the manufacturer before enabling this path.

Start from `wheel_human_wrist_adapter/config/real.template.yaml`.
Real mode requires explicit `model_file`, `wrist_config` and
`commissioned: true`; synthetic default conversion values are rejected. Configure
both sides' `raw_per_radian` (signed), `raw_zero`, `lower`, `upper`,
`max_velocity`, and `max_acceleration`. Conversion is
`raw = raw_zero + radians * raw_per_radian`; velocity is estimated from measured
position timestamps because the SDK speed units are not assumed to be SI.
Calibrate joint axes, zero offsets, direction, travel and stop thresholds.

Start each real SDK bridge separately with its verified `hand_type`, `channel`,
`device_id`, limits, speed and torque settings. It disables SDK auto-enable on
connection and does not configure the operating-system CAN interface. After
fresh measured initialization, `/cb_left_hand/activate` requests the SDK's
**whole-hand** enable, including fingers, and publishes completion in SDK status.
The protocol does not offer a verified per-wrist enable/stop guarantee here.
Use the corresponding right-hand service separately.

Commission measured hold while moving, loss of commands/feedback, bus/process
loss, raw-unit range/quantization and independent fingers before deployment.
Mock tests validate command routing, not physical dynamics or stopping distance.

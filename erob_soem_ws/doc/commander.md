# Discrete commander and CSP feedforward

`wheel_human_interfaces` contains ROSIDL contracts usable from Python or C++.
`wheel_human_commander` arbitrates whole-robot ownership, validates measured state
and goals, expands joint subsets, and monitors execution. `erob_controllers`
contains the workspace `erob_controllers/CspTrajectoryController` plugin. It
claims position, velocity_offset and torque_offset together and sends them
through `erob_hardware`; commander never accesses EtherCAT.

The existing position-only launch and 14-byte RxPDO remain the default. This
composition opts into the extended 20-byte RxPDO:

```bash
# Source the appropriate built overlay first (install/ or artifacts/dev/install/).
ros2 launch wheel_human_bringup commander.launch.py
# Separate shell, same ROS domain and overlay:
ros2 control switch_controllers --activate csp_controller --strict
ros2 topic echo /wheel_human_commander/status
```

Mock hardware activates on launch, but the motion controller starts **inactive**.
Real hardware also remains inactive and does not auto-spawn controllers. Real
launch requires `backend:=soem`, an explicit verified `config_file` and calibrated
`model_file`. The dummy model and synthetic mock profile are rejected for real
use. Configure/spawn controllers explicitly after hardware commissioning.

## Application contracts

All commander endpoints are under `/wheel_human_commander`:

| Endpoint | Contract |
| --- | --- |
| `acquire_control` | `AcquireControl`: source_id and takeover; returns success, opaque token, message |
| `renew_control` | `RenewControl`: token; returns success and remaining lease seconds |
| `release_control` | `ReleaseControl`: token; success only after confirmed stationary neutral hold |
| `move_joints` | `MoveJoints` action: token, named positions in rad, duration, optional named torque offsets in Nm |
| `move_to_pose` | `MoveToPose` action: token, group, end-effector link, stamped pose, duration; currently NOT_IMPLEMENTED |
| `status` | `CommanderStatus`: owner, readiness, goal ID, execution state, capability flags, message |
| `reset_fault` | `std_srvs/Trigger`: explicit acknowledgement after healthy stationary neutral recovery |

A source acquires ownership, renews it at 5 Hz, and waits for each discrete goal
result before sending another. Default lease is 2 seconds, timed with a steady
clock. A 10–15 Hz policy loop may continue computing, but cannot stream new goals
into an executing action: busy goals are rejected, with no queue or implicit
preemption. Cancellation is explicit. Whole-robot ownership also applies to
subset goals. Tokens coordinate trusted applications; they are not credentials.

Takeover revokes the old token immediately, stops execution, waits for fresh
controller acknowledgement and measured standstill with neutral offsets, then
responds with a new token. Release and expiry use the same stopping contract.
Deferred service responses keep the ROS executor processing feedback during the
handover. Missing stop acknowledgements or feedback block handover; after the
configured timeout, status latches `fault`. Old tokens cannot be renewed or reused.
No automatic motion or ownership resume occurs after a fault or hardware hold.

Joint goals require fresh complete measured position/velocity feedback, healthy
hardware diagnostics and a ready controller. Names must be unique and known;
values finite and within limits. Torque names must be a subset of the explicitly
requested joint names. Omitted joints keep their measured stationary position;
omitted torque entries target zero. Duration must be 0.001–3600 seconds and must
satisfy velocity, acceleration and jerk limits. New goals during execution or
stopping are rejected. Malformed goals are rejected before execution; accepted
actions return these result codes:

| Code | Meaning |
| --- | --- |
| 0 | SUCCESS |
| 1 | NOT_IMPLEMENTED |
| 2 | INVALID_GOAL |
| 3 | NOT_OWNER |
| 4 | BUSY |
| 5 | NOT_READY |
| 6 | EXECUTION_FAILED |
| 7 | CANCELED / ownership-related stop |

The last two ownership/validation rejection categories ordinarily appear as an
unaccepted action, rather than a terminal action result. A stopped goal due to
lease loss or takeover is aborted with code 7; an explicitly canceled action uses
the ROS canceled terminal state after hold is confirmed.

See `scripts/check_commander.py` for a complete Python action/service client,
including renewal, two sources, subset movement, takeover, recovery and process loss.
Calculation-only FK/IK is available through a separate Pinocchio library and
standalone query node; see [kinematics](kinematics.md). Cartesian execution, gravity
compensation and advanced planning remain unimplemented and return
NOT_IMPLEMENTED. Requesting automatic
gravity compensation explicitly fails. Caller-supplied torque offsets work
independently of those TODOs. Collision checking is not implemented.

## Cyclic execution

The custom controller exposes `/csp_controller/follow_joint_trajectory` using
`control_msgs/action/FollowJointTrajectory`. Its initial supported contract is
exactly one immediate endpoint with the full configured joint order. Optional
velocities and accelerations must be zero; optional `effort` is torque feedforward
in Nm. Multi-DOF trajectories and per-goal tolerance overrides are rejected.

Position follows a rest-to-rest quintic. Its derivative supplies velocity offset;
torque blends from the currently applied offset to the endpoint on the same
timeline. Analytic segment bounds reject infeasible durations rather than
retiming. The profile's hold acceleration and jerk also bound planned movement.
Success requires measured goal tolerance and standstill for 100 ms. Final position
and torque are retained, with zero velocity offset, until a new valid goal or stop.

`~/controller_state` publishes measured/reference positions, velocities and torque
reference. `~/status` publishes execution phase, consumed request generation,
neutral offsets, heartbeat validity and cyclic state age. `~/stop` responds with
a request generation, **not** a claim that the robot has already stopped. Commander
waits for a newer holding/neutral snapshot and measured standstill. In-flight
action requests must resolve before the stop fence is issued.

Commander publishes `std_msgs/Empty` to `~/heartbeat` at 10 Hz. A 500 ms timeout
stops an active segment or withdraws a retained torque offset even when hardware
continues receiving cyclic commands. Cancellation uses measured position/velocity
for a bounded deceleration and smoothly ramps torque to zero. An infeasible or
unconfirmed stop reports a fault and never acknowledges handover. Stop duration
is bounded by the configured acceleration/jerk and torque ramp; missing settling
feedback faults after the configured settling timeout. Lifecycle deactivation
and hard hardware faults clear offsets; they are distinct from a normal powered
cancellation ramp.

ROS callbacks, publications and allocation are outside the cyclic sampler/update.
Preallocated SPSC exchanges carry fixed-size requests and snapshots. Controller
manager and EtherCAT remain configured at 1000 Hz; commander input rate does not
set their rate. A host without real-time scheduling has not thereby demonstrated
physical 1 kHz timing acceptance.

## Wire layout and commissioning

| RxPDO byte | Object | Meaning |
| --- | --- | --- |
| 0 | 0x6040, uint16 | Control word |
| 2 | 0x607A, int32 | Target position |
| 6 | 0x60FF, int32 | CSV target velocity, unchanged |
| 10 | 0x6071, int16 | CST target torque, unchanged |
| 12 | 0x6060, int8 | Operating mode |
| 13 | padding, uint8 | Existing verified padding |
| 14 (optional) | 0x60B1, int32 | CSP velocity offset |
| 18 (optional) | 0x60B2, int16 | CSP torque offset |

Enable with `bus.csp_feedforward: true`; all drives use the same selected layout.
Every mapping entry is read back and process-image byte sizes are checked.
Additional profile keys are `max_velocity_offset` (rad/s), `max_torque_offset`
(Nm), `offset_ramp_seconds` (0.001–10 s), and `feedforward_verified`. Velocity
conversion uses `direction * rad_s * velocity_counts_per_rad_s`. Torque uses
`direction * Nm / nm_per_raw`. The latter is a calibrated joint-side conversion
from thousandths of rated current, not an assumption that the wire unit is Nm.
`effort_verified` is required. Finite, range and representability checks happen
before command publication; invalid offsets never refresh the driver watchdog.

A hardware watchdog powered hold ramps the captured torque offset to zero,
including while feedback cannot confirm standstill. Its deceleration supplies a
consistent velocity offset. Hold never auto-resumes. Initialization, disable and
hard-fault paths clear offsets. The recovery helper recognizes both supported
controllers, verifies feedback initialization parameters and neutral offset
reports, resets controllers, and checks the worker's hold generation/readiness
before explicitly releasing hold:

```bash
ros2 run erob_bringup resume_control.py
# If commander has latched a fault, after controller/hardware recovery:
ros2 service call /wheel_human_commander/reset_fault std_srvs/srv/Trigger '{}'
```

Before enabling physical feedforward, record per-drive firmware support, PDO
mapping/readback, encoder and torque-offset sign/scaling, joint-side torque
calibration, configured limits, loaded stopping/ramp behavior, bus-loss watchdog,
and disable/recovery behavior. Set `feedforward_verified` only after those checks;
it is an additional real configuration/activation gate. Keep existing topology,
effort, hold and stop verification gates. The mock profile intentionally leaves
physical verification false and labels all conversion values synthetic.

Actual gravity computation additionally needs calibrated geometry, masses,
inertias, joint conventions and the gravity vector. Software mock tests inspect
transmitted offsets; they do not model gravity or establish physical safety.

## Validation

Build the pinned Pinocchio dependency once using the [kinematics build guide](kinematics.md).

`bash scripts/test_software.sh --current` builds into `artifacts/dev`, logs under
`log/dev`, runs all colcon tests and the existing four runtime checks, then
`scripts/check_commander.py`. Native mode keeps root `build/` and `install/`.
The commander runtime acceptance uses explicitly documented relaxed scheduling
watchdogs on the shared development host; deterministic tests cover watchdog and
stop thresholds. Physical feedforward commissioning and native target validation
remain separate acceptance steps.

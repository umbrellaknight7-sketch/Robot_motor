# CSP timeout hold and explicit recovery

The ROS plugin exposes **position commands only**. All 15 drives form one holding/fault group. Position and velocity feedback remain available; effort feedback is exported only with verified calibration. CSV/CST remain library capabilities, with their original hard-fault timeout behavior.

## Responses and thresholds

| Event during active CSP | Initial response | Hard-fault threshold |
| --- | --- | --- |
| No accepted command for `command_timeout_ms` | Decelerate/hold; keep EtherCAT and drive enable | No timer merely for absent commands |
| Invalid feedback/WKC or non-advancing DC time | Immediately freeze last transmitted targets; latch unconfirmed hold at `bad_cycles` **or** `feedback_timeout_ms` | `3 * bad_cycles` **or** `3 * feedback_timeout_ms` |
| Wake-up later than `lateness_us` | Latch hold at `missed_deadlines` consecutive late wake-ups | `3 * missed_deadlines` |
| Invalid command snapshot | Reject the entire snapshot; retain the previous valid command | None from rejection itself; command expiry still applies |

Counts continue across hold entry. With `missed_deadlines: 5`, hold begins at the fifth consecutive late wake-up and a hard fault occurs at the fifteenth total. A timely wake-up resets the deadline counter. Valid feedback resets the feedback counter and age; neither event clears a hold. Feedback age is measured from the last valid exchange. `lateness_us` measures worker wake-up delay, not motor response.

Rejected values include non-finite/out-of-limit/overflowing values, wrong joint counts, wrong modes, stale/future timestamps and out-of-order generations. Rejection does not refresh the last valid command timestamp or accepted generation. A continuous stream of invalid commands therefore eventually causes command-expiry hold, not an immediate hard fault.

Command expiry monitors the controller-to-driver handoff, not upstream publishers. A controller repeating an unchanged command remains alive. These host checks do not program or replace the drive communication watchdog.

## What the motors are asked to do

`decelerating`: capture measured position and velocity once; stationary joints retain their captured position. Moving joints use a common-duration stopping profile with bounded acceleration and jerk. They stop at a natural stopping position, without intentionally returning to the position captured at timeout. The driver checks the full path against position limits and raw count range. It checks tracking while stopping.

`holding`: repeatedly transmit fixed final targets with CSP and control word `0x000F` (operation enabled). Enter this confirmed phase only after settling within position/velocity tolerances for `standstill_ms`. Fresh commands are ignored for motion, even after communication recovers. There is no duration limit on a healthy hold.

`hold_unconfirmed`: feedback is invalid, so the driver freezes the last transmitted targets and continues attempting cyclic exchange. It cannot confirm actual posture, motor enable, braking, or delivery. On feedback return, drives must still be enabled, in CSP and fault-free. The driver captures fresh feedback and stops/holds from there; it does not automatically resume or re-enable a watchdog-disabled drive.

Drive faults, unexpected disable/mode, persistent feedback/deadline failure, infeasible stop paths, excessive tracking error and failure to settle escalate to the existing hard-fault quick-stop/disable cleanup. Failed startup is not converted to powered hold. Complete communication loss may prevent stop commands from reaching the drives.

## Required profile values

Set these in `profile` or override them per motor:

```yaml
hold_acceleration: 1.0       # rad/s^2 — MOCK value only
hold_jerk: 10.0              # rad/s^3 — MOCK value only
hold_tracking_error: 0.05   # rad — MOCK value only
hold_verified: false
```

Real activation requires positive finite limits and `hold_verified: true` for every motor, in addition to the existing verification gates. The real template uses zero placeholders to prevent accidental use of mock limits. Discovery remains available with an incomplete profile. These settings are distinct from the raw drive quick-stop parameter `quick_stop_deceleration`.

The stopping velocity is `v0 * (1 - 3u^2 + 2u^3)` for `u=t/T`; choose common `T` at least `1.5*abs(v0)/hold_acceleration` and `sqrt(6*abs(v0)/hold_jerk)` for every moving joint. Round upward to whole cycles. The endpoint is `q0 + v0*T/2`. Failure to settle within `transition_timeout_ms` after this profile finishes is a hard fault. Stationarity uses `standstill_velocity`.

## Explicit resume

First resolve the cause and pause upstream trajectory producers. Inspect `/diagnostics`: each joint includes `phase_name`, `hold_reason`, `hold_generation`, `hold_start`, `hold_target`, `feedback_valid`, `feedback_age_ms`, `bad_cycles`, `late_cycles`, `claims_released`, `resume_ready`, and command-rejection details. Unconfirmed/latched holds are warnings; hard faults are errors.

Wait for confirmed `holding`, then run:

```bash
ros2 run erob_bringup resume_control.py
# Optional namespace/component overrides:
ros2 run erob_bringup resume_control.py --hardware ErobSystem --controller-manager /controller_manager
```

The helper checks the affected controllers before changing them. It supports Humble `joint_trajectory_controller/JointTrajectoryController` with:

```yaml
open_loop_control: false
set_last_command_interface_value_as_state_on_activation: false
```

It deactivates the claiming controllers (aborting old goals), waits for the worker to observe released interfaces, and reactivates them from measured feedback **while the worker still holds**. Once the original claims are restored, feedback and scheduling are stable, and controller targets are fresh and consistent with feedback, it calls `/ErobSystem/resume` (`std_srvs/srv/Trigger`) as its final step. Only the worker can acknowledge release of the current hold generation. Errors before this step leave holding latched. Concurrent hardware shutdown/deactivation takes precedence.

On success, send a **new** trajectory from current feedback. Neither the driver nor helper restores the interrupted trajectory. Do not let an external publisher continuously stream old goals during recovery. The raw resume service is not a substitute for resetting controllers; it rejects missing reset/freshness/health conditions.

Normal hardware deactivation still performs quick-stop/disable. Ctrl+C/process termination still closes the worker and EtherCAT. Software holding cannot support the robot after its controlling process or power is gone. Commission watchdogs, brakes, torque capacity, stopping clearances and whole-robot balance separately.

## Software reproduction

`scripts/test_software.sh` runs driver/plugin tests plus three mock checks. `scripts/check_hold_resume.py` covers a moving 15-joint action, command expiry, fixed holding, rejection before controller reset, helper failure, explicit recovery, old-goal abortion and a new trajectory.

The integration test enables `enable_mock_fault_injection:=true`. This defaults to false and is rejected with a real backend before hardware access. Only when explicitly enabled with `backend:=mock`, `/ErobSystem/test_expire_commands` suppresses ROS command handoff for twice the configured command timeout. It does not suspend the bus worker. No physical EtherCAT interface is opened by these tests.

## Updating an existing deployment

Add the four hold-profile fields to an existing real YAML and commission them before setting `hold_verified`. Older profiles intentionally fail real activation until these values are supplied. In a custom robot URDF, remove velocity/effort **command** interfaces for ErobSystem; keep the feedback interfaces. Apply the two trajectory-controller recovery parameters above. Rebuild and source the appropriate native overlay; do not use an older installed plugin with the new configuration.

The end-to-end recovery test uses a generated mock-only profile with a 200 ms command timeout and 100 ms lateness tolerance to isolate shared-server scheduling jitter. Deterministic engine tests verify the 20 ms expiry and exact five/15 failure boundaries. These overrides do not modify the robot template or qualify physical cycle timing.

## Optional CSP feedforward

The recovery helper also supports `erob_controllers/CspTrajectoryController`.
It verifies neutral velocity/torque offset diagnostics before releasing hold.
Powered hardware hold ramps captured torque offsets to zero; initialization,
disable and hard faults clear offsets. See [commander/feedforward](commander.md)
for the separate execution heartbeat, ownership and physical verification rules.

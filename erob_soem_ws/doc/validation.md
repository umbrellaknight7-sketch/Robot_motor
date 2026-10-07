# Software acceptance

## Commander and CSP feedforward — 2026-09-22

The complete current suite passed in the server's Humble development environment:

- All **10 packages** built into `artifacts/dev/build` and `artifacts/dev/install`.
- `colcon test-result`: **96 results, 0 errors, 0 failures, 0 skipped** (including CTest wrapper results).
- Existing generic mock, one-joint commissioning, powered-hold recovery, and wheel-human model/trajectory runtime checks all passed.
- Commander acceptance passed: inactive startup, explicit activation, competing source rejection, busy rejection, subset expansion, signed nonzero feedforward, and action feedback/results.
- Cartesian and gravity TODO requests returned NOT_IMPLEMENTED without dispatching motion.
- Takeover, stale-token rejection, cancellation, lease expiry and explicit release confirmed stationary neutral hold before completion or ownership transfer.
- Deactivating the controller while owned caused a visible stop failure, blocked takeover, revoked the token, and required explicit fault reset after recovery.
- Injected hardware command expiry blocked acquisition. The recovery helper recognized the CSP controller, restarted it from measured state with neutral offsets, and explicitly released the hardware hold.
- Killing the private test's commander process caused the controller heartbeat watchdog to stop motion and withdraw torque while hardware cycles continued.
- Deterministic tests verified legacy/extended PDO layouts, signed offset conversion and byte order, mapping readback failures, invalid offset watchdog behavior, torque ramp-down, disable/fault clearing, and bundled interface claims.

The final command was `bash scripts/test_software.sh --current`. Complete output is
`log/dev/commander_acceptance.log`; runtime launch and recovery logs are in
`log/dev/tests/commander/`. The initial sandbox run could not create ROS sockets;
the successful run used local DDS access and mock transport only. No physical
EtherCAT interface was opened.

The application test ran at 10 Hz with ownership renewal at 5 Hz; controller-manager
update rate and EtherCAT cycle configuration remained 1000 Hz / 1 ms. This is not
a measured real-time timing guarantee. Shared-host runtime checks use a synthetic
profile with relaxed scheduling watchdogs; production thresholds are covered by
deterministic driver tests. See [commander usage and commissioning](commander.md).
Native Ubuntu 22.04 Humble acceptance and physical feedforward commissioning remain
pending; FK, IK, gravity computation and advanced planning remain explicit TODOs.

## Earlier driver acceptance — 2026-09-21

Validated in the existing `ros_humble` development environment on this server. Resolved environment/package versions remain recorded in `environment/`. This is software acceptance, not physical robot commissioning.

- All five packages built and installed into `install_impl`.
- `colcon test-result`: **61 tests, 0 errors, 0 failures, 0 skipped**: 54 driver Google Test cases, five hardware Google Test cases, and two CTest wrapper results.
- Driver coverage includes 15-motor topology/mapping, PDO encoding/readback, conversions, legacy CSV/CST operation, invalid-command rejection, coherent concurrent handoffs, stopping profiles, feedback/deadline five/15 boundaries, elapsed-time thresholds, hold recovery, stale generations, tracking/settling failures, and shutdown concurrent with resume.
- Hardware tests load the Humble plugin, validate position-only command interfaces and retained states, exercise lifecycle/claims, reject real-backend fault injection before hardware access, and verify invalid commands do not cause ROS hardware errors.
- The 15-joint mock trajectory demonstration passed, reaching 0.1 rad on every joint.
- The commissioning regression passed: deactivate hardware, read feedback, reactivate, move only `motor_01_joint` while the other 14 hold, then deactivate controller/hardware.
- The hold/resume integration passed: interrupt command handoff during a trajectory, keep all 15 drives enabled at fixed positions, reject premature resume, retain hold on helper failure, reset controllers, resume explicitly, abort the old goal, and execute a new trajectory from feedback.
- A final review added regression coverage for immediate post-resume command expiry and disturbances after a long healthy hold. Shutdown posts the driver stop request before joining the ROS service thread.
- No physical EtherCAT interface was opened. SOEM was compiled; real profile and backend guards were tested without physical bus access.

Reproduce from this workspace in Bash:

```bash
source /home/xurui/miniconda3/etc/profile.d/conda.sh
conda activate ros_humble
bash scripts/test_software.sh --current
```

The historical run was captured in `log_impl/software_acceptance.log`, with mock
launch logs under `log_mock/`. After the September 22 directory reorganization,
these are preserved at `log/legacy/layout_20260922/log_impl/software_acceptance.log`
and `log/legacy/layout_20260922/log_mock/`. The old build/install artifacts are
under `artifacts/legacy/layout_20260922/`; their recorded paths refer to their
original locations. See [workspace outputs](workspace_outputs.md) for current
build, source and log paths. Generated outputs are ignored by version control.
That run used all colcon tests and three mock demonstrations on local ROS domains
176, 178, and 179. Local DDS sockets are required; sandbox socket restrictions
prevented the initial ROS run, so the accepted run used local socket access.

The shared server is not a real-time robot computer. An early end-to-end hold test encountered host scheduling jitter before its intended injected timeout. That integration test now generates its own mock-only profile with a 200 ms command timeout and 100 ms lateness tolerance. Deterministic engine tests separately verify 20 ms command expiry and exact consecutive failure boundaries. The real template's existing settings were preserved: command timeout 20 ms, feedback timeout 10 ms, five bad cycles, five missed deadlines, and lateness 4000 µs.

Expected negative tests emit rejection/error messages, including the helper deliberately targeting a nonexistent controller manager. Humble also reports the existing robot-description deprecation and unavailable FIFO scheduling on the ordinary development account. None of this establishes physical cycle performance.

## Remaining acceptance

The robot target is Ubuntu 22.04 x86-64 with apt-installed Humble under `/opt/ros/humble`. This server has no native Humble installation. Build and run the default `scripts/test_software.sh` on the robot to record native acceptance; do not reuse the conda-built binaries. See [native installation](native_humble_install.md).

Physical acceptance still requires the hub/ESI and topology, actual motor profiles, encoder scales and reference persistence, PDO/DC behavior, stopping acceleration/jerk/tracking limits, torque capacity, branch-loss/watchdog behavior, braking under load, and robot balance/collision clearance. Complete and verify the new hold profile before real activation. See [hold recovery](hold_recovery.md).

## Wrist integration software acceptance (September 22)

Built 12 packages with the `ros_humble` conda environment, using the existing
`artifacts/dev/` and `log/dev/` output layout. Final colcon results: **104 tests,
0 errors, 0 failures, 0 skipped** (`log/dev/wrist-final-unit.log`).

The five existing body acceptance scripts passed, including hardware hold/resume,
CSP feedforward, whole-robot ownership and commander-process loss. Their evidence
is in `log/dev/wrist-full-suite.log`. That combined run exposed a subsequent
wrist readiness race; after fixing action discovery and post-takeover heartbeat
readiness, the final focused wrist acceptance passed all cases in
`log/dev/wrist-acceptance.log`:

- SI wrist goals, measured success, subset preservation and finger command merging.
- Busy/mixed-goal rejection, cancel, takeover, stale owner tokens and lease expiry.
- Wrist feedback timeout with continued body/other-wrist control under the same owner.
- No automatic wrist recovery when feedback returns; explicit reset and a new goal.
- SDK command watchdog, revoked-session fencing, and adapter-process loss during motion.

All hand transport was mocked. No real CAN-FD or EtherCAT interface was opened.
The SDK extension's pure buffer tests run instead of upstream placeholder lint
tests; the manufacturer's existing source has not been reformatted for lint.
The wrist dummy axis, limits and conversion are synthetic; L30 Pro revision,
calibration and physical stopping behavior remain unverified. See
[wrist integration](wrist_integration.md) for startup, reset and commissioning.

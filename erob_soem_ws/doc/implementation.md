# Protocol and commissioning reference

The drive-specific source is the supplied **eRob CANopen and EtherCAT User Manual V1.10**, chapters 4, 5, 7 and 8, in [the searchable PDF](../eRob_Chapters_4_5_7_8_searchable.pdf). Printed page numbers differ from PDF page indexes.

## Process data

Sections 4.4–4.5 describe mapping and distributed clocks. The driver writes individual SDOs in PRE-OP, reads every write back, assigns 0x1600/0x1A00 through 0x1C12/0x1C13, checks 14/16-byte process images (20/16 bytes with opt-in CSP feedforward), configures DC and then enables SYNC0 on matched DC-capable motors. Infrastructure slaves remain in the master network.

| Direction | Object sequence | Wire bytes |
| --- | --- | --- |
| RxPDO, master to drive | 6040:00 u16, 607A:00 i32, 60FF:00 i32, 6071:00 i16, 6060:00 i8, padding u8 | 14 |
| RxPDO, optional CSP feedforward | Legacy RxPDO followed by 60B1:00 i32 velocity offset, 60B2:00 i16 torque offset | 20 |
| TxPDO, drive to master | 6041:00 u16, 603F:00 u16, 6064:00 i32, 606C:00 i32, 6077:00 i16, 6061:00 i8, padding u8 | 16 |

Encoding is explicitly little-endian. The final padding descriptor 0x00000008 requires confirmation against the installed firmware/ESI. The real configuration gate includes padding_verified for this reason. PDO assignments are unchanged across CSP/CSV/CST.

The dictionary identifies 0x1C32:0A as read-only despite conflicting narrative instructions. This implementation never writes it. It reads the minimum cycle at 0x1C32:05 and cycle time at 0x1C32:02; SOEM configures the ESC's DC registers. The initial period is 1 ms. Smaller-network examples do not validate this robot's physical cycle performance.

## Drive behavior and units

Chapter 5 supplies the CiA 402 status/control-word transitions and CSP/CSV/CST behavior. The engine follows status feedback through shutdown/switch-on/enable, waits at least 500 ms with every drive enabled, and seeds safe targets. Mode transitions require sustained standstill and a 0x6061 acknowledgment. Quick stop uses option 0x605A=2 and the profile's raw 0x6085 deceleration; disable is held off for the configured >300 ms interval. Check braking under load during commissioning.

Dictionary §§8.2.38–8.2.41 (printed pp. 183–184) define offline gain writes: 0x2381:01/02 velocity PI, 0x2382:01 position P, 0x2382:03 velocity feedforward, all UINT16. The first three use the UINT32 0x2383 bus-tuning gate; its prior value is restored. Feedforward uses 1/16384 units, so 0x4000 means unity. Tools send no persistent save command. The public transport interface has no unrestricted write API.

0x6080 maximum velocity is pulses/s, not rpm (printed p. 208). The driver applies the configured velocity conversion. It writes 0x607D position limits, 0x6080 speed limit, 0x6072 current-related torque limit and quick-stop settings with readback. Confirm these objects and units for every real motor profile.

0x6071 and 0x6077 are rated-current-related permille values. The nm_per_raw scale requires verified calibration; it is not inferred from an unverified rated-torque SDO. A 0x6077 value is not a direct torque-sensor sample. Hardware with no calibration exports position/velocity states only. The ROS plugin exports position commands by default; `bus.csp_feedforward` adds custom velocity_offset and torque_offset command interfaces. See the [commander/feedforward contract](commander.md) for commissioning gates and stopping behavior. No command requests manual brake release (0x4602), encoder reset, persistent saving or firmware update.

## Shared YAML configuration

The bus section holds backend (mock/soem), NIC, expected motor count, timing, real-time settings and verification records. The profile section provides shared motor defaults; motor entries override them.

| Field | Meaning |
| --- | --- |
| logical_id | Unique robot label, quoted hexadecimal; never an EtherCAT address |
| joint | Stable ROS name, ordered identically in YAML and ros2_control description |
| branch, branch_position | Robot branch label and 1-based physical slot |
| topology_path | Physical parent ESC output port sequence, including infrastructure; first discovered slave is the root |
| identity | Exact vendor/product/revision and optional serial; require_serial controls comparison |
| position_counts_per_rad, velocity_counts_per_rad_s | Explicit raw-to-output-joint scales |
| zero_counts, direction | Raw position at joint zero and sign (+1/-1) |
| effort_verified, nm_per_raw | Whether Nm is calibrated, and Nm per raw torque/current-ratio unit |
| min_position, max_position, max_velocity, max_effort | Software limits in joint units |
| max_torque_raw | Positive 0x6072 limit in raw units, independent of Nm |
| quick_stop_deceleration | Positive raw UINT32 0x6085 value, requiring hardware verification |
| standstill_velocity, initial_mode | Standstill tolerance in rad/s; ROS requires mode 8 (CSP); library also supports 9/10 |
| csp_feedforward (bus) | Selects the extended offset PDO layout; false by default |
| max_velocity_offset, max_torque_offset, offset_ramp_seconds, feedforward_verified | CSP offset limits in SI units, torque withdrawal duration, and physical verification gate |
| hold_acceleration, hold_jerk, hold_tracking_error, hold_verified | CSP stopping acceleration (rad/s²), jerk (rad/s³), tracking tolerance (rad), and real-profile verification |

Position is direction × (raw − zero_counts) / position_counts_per_rad. Velocity is direction × raw / velocity_counts_per_rad_s. Calibrated effort is direction × raw × nm_per_raw. No extra internal gearbox ratio is applied.

The sample has 15 entries with the 7+5+3 logical branch arrangement. Library storage is dynamically sized. Extra infrastructure does not change the required motor count. All motors must match before tuning writes or driver configuration.

## Physical topology verification

SOEM v1.4.0's parentport field belongs to its DC propagation calculation and can refer to a DC ancestor across non-DC infrastructure. It is not used as a physical port path. Discovery derives paths from immediate physical parents, active-port masks, entry ports and EtherCAT traversal order 0→3→1→2. DC entry-port metadata is established before deriving paths. Non-DC infrastructure currently requires standard upstream port 0; an unknown/unavailable entry port is rejected. Confirm this convention and the ESC-to-external-port correspondence against the installed hub model/ESI.

A junction may expose several infrastructure slaves. Paths include each one; there is no assumption of 16 total slaves. Replace synthetic paths with discovery output. Swapping indistinguishable motors at the same topology locations cannot be detected if serials are absent/nonunique; such replacements require recommissioning. Verified unique serials add an identity check where supported.

Lost WKC does not necessarily identify a physical branch without additional bus access. The driver reports a group/bus fault with unknown culprit instead of guessing. A drive-reported PDO fault identifies its motor and branch. The fake transport also supports a known disconnected-motor culprit for deterministic tests. No mailbox transactions or topology rescans run in the cyclic loop.

## Validation boundaries

The fake transport sits below the production engine and shared mapping/limits/readback implementation. Tests cover discovery/matching, signed PDO byte order, SDO/mapping failures, conversion/limits, lifecycle timing, mode acknowledgment/timeout, group stops, stale commands, WKC loss/reconnection and concurrent state exchange. Humble tests load the plugin, validate interfaces and exercise lifecycle/claim rules. The ROS acceptance client drives all 15 joints with standard controllers.

The mock is a protocol simulator, not a dynamics/load model. Software tests cannot validate SOEM raw sockets, the installed hub, firmware PDO acceptance, drive watchdog reaction, DC accuracy, loaded braking, torque calibration or robot cycle performance. Those remain physical commissioning tasks.

## CSP timeout recovery

The ROS plugin selects `BusConfig::csp_hold_enabled`; direct library users must select that policy explicitly. New hold phases and rejected-command diagnostics are described in [hold_recovery.md](hold_recovery.md). `Driver::submit` returns Accepted, Rejected, or Unavailable (its explicit boolean conversion means Accepted). `Driver::resume(generation)` waits for bounded worker acknowledgment without consuming the shared state snapshot. Recovery requires controller claim release and reacquisition since hold entry. Legacy CSV/CST timeout handling remains a hard stop; invalid submissions are rejected for all modes.

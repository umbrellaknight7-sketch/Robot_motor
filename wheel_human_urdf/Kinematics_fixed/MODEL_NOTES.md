# Kinematics_fixed comparison and TCP additions

Extracted from `../Kinematics_fixed.zip`. The ZIP and `../Kinematics_only` were not modified. Only the extracted URDF was edited; all other exported files remain byte-for-byte identical to the archive.

## Added TCP frames

- `Left_Hand_Link` -> fixed `Left_Hand_TCP_Joint` -> `Left_Hand_TCP`.
- `Right_Hand_Link` -> fixed `Right_Hand_TCP_Joint` -> `Right_Hand_TCP`.
- Both transforms: `xyz="0.05 0 0"`, `rpy="0 0 0"`, expressed in the respective hand link frame.
- These are virtual frames without visual, collision or inertial geometry.

## Structure comparison with Kinematics_only

Both models now have root `base_link`, 22 links, 19 revolute joints and 2 fixed TCP joints. Link names, joint names/types, parent-child connections, and local joint axes match exactly. The tree has one root, is connected and has no cycles. All mesh URIs resolve to files within the extracted package.

The existing six-joint `left_arm` and `right_arm` planning groups remain valid, including wrists. Complete FK/IK configurations still require exactly 19 joint positions; fixed TCP joints need no entries.

Topology consistency does not imply identical transforms or joint-angle calibration. Preserve the newly exported frame definitions when using this model.

## Exported differences retained

All 19 revolute joints have limits [-3.14, +3.14] rad (about +/-179.909 degrees), versus the old exact +/-pi. Velocity and effort limits remain zero. Inertial data differ for 16 links; this check does not validate physical mass properties.

The following 11 joint origins differ. Values are local to each parent link.

| Joint | Old xyz | New xyz | Old rpy | New rpy |
|---|---|---|---|---|
| Feet_Joint | -0.1175 -0.088326 0.027791 | -0.1175 -0.088326 0.027791 | -1.5708 0 -3.1416 | 1.5708 -1.5708 0 |
| Knee_Joint | 0 -0.385 0.0075 | 0.385 0 0.0075 | 0 0 3.1416 | 0 0 -3.1416 |
| Hip_Joint | 0 0.385 0 | -0.385 0 0 | 0 0 0 | 0 0 3.1416 |
| Neck_Joint | 0 0.46565 -0.09575 | 0.46565 0 -0.09575 | 1.5708 1.5708 0 | 1.5708 0 1.5708 |
| Face_Joint | -0.03635 0 0.1196 | 0 0.03635 0.1196 | 1.5708 0 -1.5708 | 1.5708 0 3.1416 |
| Left_Shoulder_Joint | 0 0.462 -0.24615 | 0.462 0 -0.24615 | 0 0 0 | 0 0 -3.1416 |
| Left_Arm_Joint1 | -0.044 0 -0.0605 | 0 -0.044 -0.0605 | 0 1.5708 0 | 1.5708 0 3.1416 |
| Left_Arm_Joint2 | 0 -0.1422 0.0438 | -0.1422 0 0.0438 | -1.5708 0 0 | -1.5708 0 -1.5708 |
| Right_Shoulder_Joint | 0 0.462 0.055651 | 0.462 0 0.055651 | 0 0 0 | 0 0 0 |
| Right_Arm_Joint1 | -0.044 0 0.0595 | 0 0.044 0.0595 | 3.1416 -1.5708 0 | 1.5708 0 3.1416 |
| Right_Arm_Joint2 | 0 -0.1425 -0.044 | 0.1425 0 -0.044 | 1.5708 0 0 | 1.5708 0 1.5708 |

Detailed comparison and preserved hashes: [model_comparison.json](model_comparison.json).

## Software validation

The existing `erob_soem_ws/scripts/check_kinematics.py` passed using this URDF in `ros_humble`, on isolated ROS domain 188:

- Both arms, position-only and full-pose FK -> IK -> FK: all successful in 2 iterations, position residuals <=0.017 mm.
- Non-group joints held, invalid requests rejected, unreachable target failure and exact zero-pose success checked.
- Both TCPs match a separate robot_state_publisher at wrist angles 0.1 and 0.7 rad (<1e-8 m, <1e-6 rad).
- Query graph contains no controllers, SDK nodes or command publishers.
- Log: `/home/xurui/downloads/Robot_motor/erob_soem_ws/log/dev/kinematics_fixed_20260927_100934/acceptance.log`. Socket/interface warnings occurred, but all service and TF checks completed successfully via available ROS transport.

## ROS integration

The exported package still uses ROS 1 catkin and `.launch` XML files, unlike the converted ROS 2 `Kinematics_only` package. It was not converted or registered as a ROS 2 mesh package in this task. The existing ROS 2 kinematics query launch accepts its URDF directly because it loads no mesh geometry. From a shell with ros_humble, the Pinocchio prefix and workspace overlay sourced:

```bash
ros2 launch wheel_human_commander kinematics.launch.py \
  model_file:=/home/xurui/downloads/Robot_motor/wheel_human_urdf/Kinematics_fixed/urdf/Kinematics_fixed.urdf
```

Existing body bringup model selection and the CAD viewer were not changed.

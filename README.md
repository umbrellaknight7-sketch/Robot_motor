# Robot_motor

Robot control software, the locally extended LinkerHand SDK, and robot models.

| Directory | Contents |
| --- | --- |
| [erob_soem_ws](erob_soem_ws/README.md) | ROS 2 workspace with EtherCAT drivers, controllers, robot descriptions, tests, and deployment instructions |
| [linkerhand-l30-sdk](linkerhand-l30-sdk/README.md) | LinkerHand L30 SDK with local wrist integration changes and supplied CAN bus driver archives |
| [wheel_human_urdf](wheel_human_urdf/) | Original and corrected CAD robot models and supporting tools |

Start with the [workspace build instructions](erob_soem_ws/README.md) and
[native ROS 2 Humble installation guide](erob_soem_ws/doc/native_humble_install.md).
The example workspace defaults to mock hardware; real hardware requires the
commissioning steps documented in the workspace.

Keep the three directories together when cloning or moving this project.
The headless LinkerHand package is currently a regular directory in
`erob_soem_ws/src/linker_hand_l30_v6`, as well as in the sibling SDK. Some existing
integration documentation describes an earlier symlink layout.

Generated builds, installed environments, logs, caches, editor databases, the
workspace ZIP, and historical `erob_soem_ws/archive` and `erob_soem_ws/lib`
contents are excluded. Rebuild the workspace on the target computer.

## SDK provenance

The SDK was originally cloned from
[linker-bot/linkerhand-l30-sdk](https://github.com/linker-bot/linkerhand-l30-sdk)
at commit `0103fc55f937b2a9edd888cf60b9cb22876d2870`.
This repository includes its working files and local modifications directly.
Existing third-party license and attribution files are retained.

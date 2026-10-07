# Workspace output directories

The standard ROS workspace directories stay at the root. Separate development
builds use `artifacts/`, and all new logs go under `log/`:

```text
erob_soem_ws/
├── src/
├── scripts/
├── doc/
├── environment/
├── archive/                       # Archived source packages
├── build/                         # Native/system Humble build
├── install/                       # Native/system Humble install
├── artifacts/
│   ├── COLCON_IGNORE
│   ├── dev/
│   │   ├── build/                 # Conda development build
│   │   └── install/               # Conda development install
│   └── legacy/                    # Preserved obsolete build/install outputs
└── log/
    ├── native/
    │   ├── colcon/
    │   ├── ros/
    │   └── tests/
    ├── dev/
    │   ├── colcon/
    │   ├── ros/
    │   └── tests/
    │       ├── mock/
    │       ├── csp_startup/
    │       ├── hold/
    │       └── wheel_human/
    └── legacy/                    # Preserved historical logs
```

`build/`, `install/`, `log/` and `artifacts/` are ignored by Git. The scripts create
`artifacts/COLCON_IGNORE` so generic colcon discovery skips its contents. Normal
workspace builds explicitly discover only `src/`.

## Native robot

In a fresh Bash terminal without conda or a Python virtual environment:

```bash
bash scripts/test_software.sh
source install/setup.bash
```

This requires apt-installed Humble at `/opt/ros/humble`. Existing root `build/`
and `install/` contents are retained during the September 2026 reorganization;
on this server they contain the old `soem_master` example, not a validated native
build of the current packages. Build from source on the robot as described in
[native installation](native_humble_install.md).

## Development server

In a fresh Bash terminal, activate only the development underlay before building:

```bash
source /home/xurui/miniconda3/etc/profile.d/conda.sh
conda activate ros_humble
bash scripts/test_software.sh --current
source artifacts/dev/install/setup.bash
```

Do not source an obsolete overlay before rebuilding. Build caches and install
setup files contain absolute paths; development outputs must be rebuilt at their
new location, rather than moved and treated as working installations.

## Test logs

The test runner sets `EROB_TEST_LOG_DIR` to `log/<profile>/tests`. Each acceptance
script creates its own subdirectory for launch logs, ROS logs and any generated
test configuration. When invoked independently, scripts choose `dev` if conda is
active and `native` otherwise. Override `EROB_TEST_LOG_DIR` when another location
is needed. Manual ROS launches can use, for example:

```bash
export ROS_LOG_DIR="$PWD/log/dev/ros"
ros2 launch wheel_human_bringup robot.launch.py
```

Colcon generates dated build/test logs inside the selected `colcon/` directory.
Acceptance logs use stable names and are replaced on the next run.

## Preserved pre-migration outputs

After the new development build and acceptance checks pass, old `build_impl/`
and `install_impl/` trees are preserved under
`artifacts/legacy/layout_20260922/`. They are historical artifacts, not usable
relocated overlays. Old `log_impl/`, `log_mock/`, `log_hold/` and `log_ros/`
directories are collected under `log/legacy/layout_20260922/` with their original
directory names. Existing standard root `log/` history stays in place.

Nothing in these historical directories is automatically deleted. Their logs
may contain the original absolute paths from the runs they record.

## Migration verification — 2026-09-22

All seven packages were rebuilt from the conda Humble underlay into
`artifacts/dev/`. The new setup files and CMake caches contained no references
to the previous `install_impl/` or root `install/` overlay. `colcon test-result`
reported **76 tests, zero errors, zero failures, zero skipped** (including CTest
wrapper results). All four mock acceptance scripts passed: the generic 15-axis
trajectory, one-joint commissioning, hold/resume, and wheel human trajectory/TF
checks. The run is recorded in `log/dev/software_acceptance.log`.

The six obsolete directories were archived after those checks passed. The move
manifest is `artifacts/legacy/layout_20260922/migration.json`. Root `build/` and
`install/` were retained in place. Native/system Humble validation remains pending
on the robot; this server has no `/opt/ros/humble` installation.

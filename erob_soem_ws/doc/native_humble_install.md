# Native ROS 2 Humble installation on the robot

Target: **Ubuntu 22.04 Jammy, x86-64/amd64**. Install ROS with Ubuntu/ROS Debian packages under **/opt/ros/humble**. No conda environment is needed on the robot. The eRob packages remain a normal colcon overlay in this workspace's **install** directory; they do not need to be copied into /opt/ros/humble.

The development server's conda lock files document its previous tests only. Do not deploy those binaries, Python environment, or build/install caches onto the robot. This server currently has no native /opt/ros/humble installation, so native build acceptance must be performed with the steps below.

## 1. Prepare Ubuntu and the ROS apt repository

Use an ordinary-user Bash terminal without a conda/virtualenv or a different ROS distribution sourced. On a fresh Ubuntu installation, update the base system before adding ROS dependencies, particularly systemd/udev. Review package changes and reboot first if the upgrade requires it.

```bash
cat /etc/os-release
dpkg --print-architecture
sudo apt update
sudo apt upgrade
sudo apt install locales software-properties-common curl python3
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

If the ROS 2 apt repository is not already configured, install its official source/key package:

```bash
ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

These steps follow the [official Humble Ubuntu installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and the [official repository setup source](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst).

## 2. Install ROS and the eRob build/control dependencies

```bash
sudo apt install \
  ros-humble-ros-base \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-ament-cmake-gtest \
  ros-humble-ament-cmake-pytest \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep python3-yaml \
  build-essential cmake libyaml-cpp-dev git libeigen3-dev \
  libboost-filesystem-dev libboost-serialization-dev libboost-system-dev liburdfdom-dev
```

ROS Base provides the headless motor-control runtime. The joint-state publishers
and RViz support the new robot description's display launch; headless control
does not start them. SOEM is already vendored and built from source by
erob_soem_vendor; do not substitute the old demo's prebuilt libsoem.so.

```bash
source /opt/ros/humble/setup.bash
command -v python3
command -v colcon
ros2 pkg prefix controller_manager
```

Expected: system Python /usr/bin/python3, apt colcon /usr/bin/colcon, and controller_manager under /opt/ros/humble. The package does not require the development server's Python 3.12; Ubuntu 22.04's system Python is the intended interpreter here.

## 3. Resolve workspace dependencies

Copy/check out the source workspace on the robot, for example at /home/xurui/downloads/Robot_motor/erob_soem_ws. Adjust that example path for the actual account. Keep src/, scripts/, doc/, the manual and your real configuration. Do not copy generated build/install/log directories.

```bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
source /opt/ros/humble/setup.bash

# Initialize once per machine, if not already initialized:
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Build as the ordinary user, not with sudo. The real master may require additional privileges later; compilation does not.

## 4. Build and validate using native directories

The default test script sources `/opt/ros/humble`, selects the system
compiler/Python, builds into `build/` and `install/`, and writes logs under
`log/native/`:

```bash
bash scripts/setup_pinocchio.sh  # Once per underlay; isolated pinned dependency
source scripts/pinocchio_env.sh
bash scripts/test_software.sh
source install/setup.bash
```

It runs the C++/plugin tests, robot model/configuration tests, the 15-joint mock
CSP demonstration, the disabled-feedback → activation → one-joint CSP →
deactivation regression, the hold/resume regression, and the wheel human
trajectory/TF acceptance check. Tests use local ROS domains 176–179, which
should be reserved for software validation. No physical EtherCAT interface is opened.

For a manual build instead:

```bash
source /opt/ros/humble/setup.bash
source scripts/pinocchio_env.sh
export CMAKE_BUILD_PARALLEL_LEVEL=8 MAKEFLAGS=-j8
colcon --log-base log/native/colcon build --base-paths src \
  --build-base build --install-base install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Keep native and conda build trees separate, even on the same machine. Colcon's generated setup scripts and CMake caches retain paths to the underlay and tools used during the build. Do not source artifacts/dev/install on the robot.

Record the actual native versions after a successful installation:

```bash
mkdir -p log/native
dpkg-query -W 'ros-humble-*' libyaml-cpp-dev cmake gcc g++ python3 \
  > log/native/native-package-versions.txt
```

The historical 42-test result was obtained in the development environment. Record the native run's actual result; do not treat the old report as proof of a native build.

## 5. Source the correct environment in every robot terminal

Ordinary ROS client terminal:

```bash
source /opt/ros/humble/setup.bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
source install/setup.bash
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/log_real_client"
```

For the supervised privileged bus terminal used by the [real-robot guide](real_robot_first_test.md), source the same native installation inside the root shell:

```bash
sudo /bin/bash --noprofile --norc
source /opt/ros/humble/setup.bash
cd /home/xurui/downloads/Robot_motor/erob_soem_ws
source install/setup.bash
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/erob-real-ros-log
ulimit -l unlimited
ulimit -r 80
```

System-wide ROS installation does not automatically grant raw Ethernet, FIFO scheduling or memory-lock privileges. The bus ownership and commissioning requirements are unchanged. Do not enable real motors merely because installation succeeds.

Continue with disabled discovery, verified configuration, inactive launch, feedback checks and the first CSP test in [real_robot_first_test.md](real_robot_first_test.md). It now uses native Humble throughout.

## Development server only

The script no longer activates conda implicitly. To reproduce development tests on this server, explicitly activate its existing environment, then opt into it:

```bash
source /home/xurui/miniconda3/etc/profile.d/conda.sh
conda activate ros_humble
bash scripts/setup_pinocchio.sh  # Once per underlay; isolated pinned dependency
source scripts/pinocchio_env.sh
bash scripts/test_software.sh --current
```

That mode uses `artifacts/dev/build/`, `artifacts/dev/install/` and `log/dev/`.
It is separate from the robot deployment.

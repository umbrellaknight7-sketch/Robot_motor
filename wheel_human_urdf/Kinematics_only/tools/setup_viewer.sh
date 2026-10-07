#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/environment.sh"
mkdir -p "$viewer_artifacts"/{debs,deps,source}
cd "$viewer_artifacts/debs"
# Extract Ubuntu packages locally: no sudo, apt install, or system service changes.
apt-get download xvfb x11vnc novnc python3-websockify websockify mesa-utils mesa-utils-bin \
  openbox libobt2v5 libobrender32v5 libimlib2 libstartup-notification0 wmctrl
for package in ./*.deb; do dpkg-deb -x "$package" "$viewer_artifacts/deps"; done
if [[ ! -d "$viewer_artifacts/source/joint_state_publisher" ]]; then
  git clone --depth 1 --branch 2.4.0 https://github.com/ros/joint_state_publisher.git \
    "$viewer_artifacts/source/joint_state_publisher"
fi
cd "$viewer_root/erob_soem_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon --log-base "$viewer_logs/colcon" build \
  --base-paths "$viewer_package" "$viewer_artifacts/source/joint_state_publisher" \
  --build-base "$viewer_artifacts/build" --install-base "$viewer_artifacts/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

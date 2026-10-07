#!/usr/bin/env bash
# Source this file. User-local viewer artifacts are separate from motor bringup.
viewer_package=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
viewer_root=$(cd "$viewer_package/../.." && pwd)
viewer_artifacts="$viewer_root/erob_soem_ws/artifacts/dev/rviz_viewer"
viewer_logs="$viewer_root/erob_soem_ws/log/dev/rviz_viewer"
viewer_conda="${RVIZ_CONDA_ROOT:-/home/xurui/miniconda3}"
source "$viewer_conda/etc/profile.d/conda.sh"
conda activate ros_humble
mkdir -p "$viewer_logs"
export ROS_LOG_DIR="$viewer_logs/ros"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=185
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 QT_AUTO_SCREEN_SCALE_FACTOR=0
export LP_NUM_THREADS=4
if [[ -f "$viewer_artifacts/install/setup.bash" ]]; then
  source "$viewer_artifacts/install/setup.bash"
fi

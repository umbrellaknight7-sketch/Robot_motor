#!/usr/bin/env bash
# Display-only runtime: reuse installed GUI dependencies, then the workspace overlay.
viewer_tools=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
viewer_workspace=$(cd "$viewer_tools/../.." && pwd)
viewer_artifacts="$viewer_workspace/artifacts/dev/rviz_viewer"
viewer_logs="$viewer_workspace/log/dev/rviz_viewer"
viewer_conda="${RVIZ_CONDA_ROOT:-/home/xurui/miniconda3}"
source "$viewer_conda/etc/profile.d/conda.sh"
conda activate ros_humble
if [[ ! -f "$viewer_artifacts/install/setup.bash" || ! -f "$viewer_workspace/artifacts/dev/install/setup.bash" ]]; then
  echo 'Existing viewer runtime and built workspace overlay are required; see doc/cad_description.md.' >&2
  return 1
fi
source "$viewer_artifacts/install/setup.bash"
source "$viewer_workspace/artifacts/dev/install/setup.bash"
mkdir -p "$viewer_logs"
export ROS_LOG_DIR="$viewer_logs/ros" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=185
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 QT_AUTO_SCREEN_SCALE_FACTOR=0 LP_NUM_THREADS=4

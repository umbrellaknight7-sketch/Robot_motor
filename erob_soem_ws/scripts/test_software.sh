#!/usr/bin/env bash
# Default: apt-installed Humble. --current uses an already activated dev environment.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---system}"
cmake_args=(-DCMAKE_BUILD_TYPE=RelWithDebInfo)

# Keep native and conda development build/install trees separate.
case "$mode" in
  --system)
    if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
      echo "Native mode requires a fresh shell without conda or a Python venv." >&2
      exit 2
    fi
    if [[ ! -r /opt/ros/humble/setup.bash ]]; then
      echo "Install system Humble first; see doc/native_humble_install.md." >&2
      exit 2
    fi
    if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != humble ]]; then
      echo "A different ROS distribution is sourced. Use a fresh shell." >&2
      exit 2
    fi
    export PATH="/usr/bin:/bin:$PATH"
    source /opt/ros/humble/setup.bash
    profile=native
    build_base=build
    install_base=install
    python_bin=/usr/bin/python3
    colcon_bin=/usr/bin/colcon
    cmake_args+=(-DCMAKE_C_COMPILER=/usr/bin/gcc
                 -DCMAKE_CXX_COMPILER=/usr/bin/g++
                 -DPYTHON_EXECUTABLE=/usr/bin/python3
                 -DPython3_EXECUTABLE=/usr/bin/python3)
    ;;
  --current)
    if [[ "${ROS_DISTRO:-}" != humble ]]; then
      echo "Source/activate your Humble environment before using --current." >&2
      exit 2
    fi
    profile=dev
    build_base=artifacts/dev/build
    install_base=artifacts/dev/install
    python_bin="$(command -v python3)"
    colcon_bin="$(command -v colcon)"
    ;;
  *)
    echo "Usage: bash scripts/test_software.sh [--system|--current]" >&2
    exit 2
    ;;
esac
set -u
cd "$workspace"
source scripts/pinocchio_env.sh
if [[ "$profile" == dev ]]; then
  mkdir -p artifacts
  touch artifacts/COLCON_IGNORE
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-8}"
export MAKEFLAGS="${MAKEFLAGS:--j8}"
log_base="log/$profile/colcon"
export ROS_LOG_DIR="$workspace/log/$profile/ros" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=176
export EROB_TEST_LOG_DIR="$workspace/log/$profile/tests"
mkdir -p "$ROS_LOG_DIR"

# Build the workspace before running the existing unit and contract tests.
"$colcon_bin" --log-base "$log_base" build --base-paths src \
  --build-base "$build_base" --install-base "$install_base" \
  --cmake-args "${cmake_args[@]}"
# Humble environment hooks reference unset variables.
set +u
source "$install_base/setup.bash"
set -u
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$colcon_bin" --log-base "$log_base" test --base-paths src \
  --build-base "$build_base" --install-base "$install_base" \
  --event-handlers console_direct+
"$colcon_bin" --log-base "$log_base" test-result --test-result-base "$build_base" --verbose

# Runtime checks launch isolated mock systems and clean up their child processes.
"$python_bin" scripts/check_mock.py
"$python_bin" scripts/check_csp_startup.py
"$python_bin" scripts/check_hold_resume.py
"$python_bin" scripts/check_wheel_human_mock.py
"$python_bin" scripts/check_commander.py

"$python_bin" scripts/check_wrists.py

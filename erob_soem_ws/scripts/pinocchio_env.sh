# Source after the ROS/conda underlay, before building or using the workspace.
_pin_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_pin_profile=native
[[ -z "${CONDA_PREFIX:-}" ]] || _pin_profile=dev
_pin_prefix="$_pin_workspace/artifacts/$_pin_profile/deps/install/pinocchio-3.8.0"
if [[ ! -f "$_pin_prefix/lib/cmake/pinocchio/pinocchioConfig.cmake" ]]; then
  echo 'Pinned Pinocchio is missing. Run bash scripts/setup_pinocchio.sh first.' >&2
  unset _pin_workspace _pin_profile _pin_prefix
  return 1
fi
export CMAKE_PREFIX_PATH="$_pin_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$_pin_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _pin_workspace _pin_profile _pin_prefix

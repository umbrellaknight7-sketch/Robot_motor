#!/usr/bin/env bash
# Isolated C++ dependency build. Activate the intended ROS underlay first.
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_DISTRO:-}" != humble ]]; then
  echo 'Activate/source the intended Humble environment first.' >&2
  exit 2
fi
profile=native
[[ -z "${CONDA_PREFIX:-}" ]] || profile=dev
root="$workspace/artifacts/$profile/deps"
source_dir="$root/src/pinocchio-3.8.0"
build_dir="$root/build/pinocchio-3.8.0"
install_dir="$root/install/pinocchio-3.8.0"
revision=655877b314baed68c7e2d4dd56b0a0200bb9f98e
mkdir -p "$root/src"
touch "$workspace/artifacts/COLCON_IGNORE"
if [[ ! -d "$source_dir" ]]; then
  git clone --branch v3.8.0 --depth 1 https://github.com/stack-of-tasks/pinocchio.git "$source_dir"
fi
if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]]; then
  echo 'Pinocchio checkout differs from the pinned revision; refusing to overwrite it.' >&2
  exit 2
fi
git -C "$source_dir" submodule update --init --depth 1 cmake
cmake -S "$source_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_PREFIX="$install_dir" \
  -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_LIBPYTHON=OFF \
  -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=OFF \
  -DBUILD_WITH_OPENMP_SUPPORT=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARK=OFF -DBUILD_TESTING=OFF -DBUILD_UTILS=OFF
cmake --build "$build_dir" --parallel "${PINOCCHIO_BUILD_JOBS:-2}"
cmake --install "$build_dir"
printf 'Pinocchio installed at %s\nSource scripts/pinocchio_env.sh before building the workspace.\n' "$install_dir"

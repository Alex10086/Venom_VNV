#!/usr/bin/env bash
# Workspace-local Ceres; keep system Ceres and FAST-LIO untouched.
set -eo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
third="$ws/third_party/bievr"
prefix="$third/ceres-2.2.0-install"
# ROS1 is shipped upstream but must not enter this ROS2 workspace build.
touch "$repo/localization/lio/BIEVR-LIO/interfaces/ros1/COLCON_IGNORE"
mkdir -p "$third"
touch "$third/COLCON_IGNORE"
if [ ! -f "$prefix/lib/cmake/Ceres/CeresConfig.cmake" ]; then
  if [ ! -f "$third/ceres-solver-2.2.0/CMakeLists.txt" ]; then
    curl -fL --retry 3 https://codeload.github.com/ceres-solver/ceres-solver/tar.gz/refs/tags/2.2.0 -o "$third/ceres-2.2.0.tar.gz"
    tar -xzf "$third/ceres-2.2.0.tar.gz" -C "$third"
  fi
  cmake -S "$third/ceres-solver-2.2.0" -B "$third/ceres-build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF -DUSE_CUDA=OFF
  cmake --build "$third/ceres-build" --parallel "${BIEVR_BUILD_JOBS:-2}"
  cmake --install "$third/ceres-build"
fi
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
ros2 pkg prefix livox_ros_driver2 >/dev/null
cd "$ws"
export CMAKE_BUILD_PARALLEL_LEVEL="${BIEVR_BUILD_JOBS:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}}"
colcon build \
  --packages-select bievr_lio bievr_ros_common bievr_lio_ros2 venom_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCeres_DIR="$prefix/lib/cmake/Ceres"

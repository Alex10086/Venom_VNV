#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$(cd "$ROOT/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then source "$WS/install/setup.bash"; fi
cd "$ROOT"
git submodule update --init exploration/tare_planner exploration/autonomous_exploration_development_environment
apply_patch_once() {
  if git -C "$1" apply --check "$2" 2>/dev/null; then
    git -C "$1" apply "$2"
  elif ! git -C "$1" apply --reverse --check "$2" 2>/dev/null; then
    echo "Patch does not match source: $2" >&2; exit 1
  fi
}
# Only deploy the real-robot processing packages; do not pull Gazebo/RViz plugins
# into the default hardware build. No user planning/ ignore file is changed.
for pkg in loam_interface sensor_scan_generation vehicle_simulator velodyne_simulator visualization_tools waypoint_example waypoint_rviz_plugin; do
  touch "$ROOT/exploration/autonomous_exploration_development_environment/src/$pkg/COLCON_IGNORE"
done
apply_patch_once "$ROOT/exploration/tare_planner" "$ROOT/tools/tare/patches/tare-runtime.patch"
apply_patch_once "$ROOT/exploration/autonomous_exploration_development_environment" "$ROOT/tools/tare/patches/path-follower-empty.patch"
cd "$WS"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
# colcon passes make -j explicitly unless MAKEFLAGS contains a jobs limit.
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}}"
colcon build --packages-select tare_planner terrain_analysis terrain_analysis_ext local_planner venom_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$WS/install/setup.bash"
if ldd "$WS/install/tare_planner/lib/tare_planner/tare_planner_node" | grep -q 'not found'; then
  echo 'TARE runtime library missing' >&2; exit 1
fi

# Hunter MID360 LIO Drift Handoff

## Current Symptom

The Hunter SE MID360 mapping stack initially looks correct with the restored 3D Point-LIO + 31.45° static mount compensation, but after driving part of a lap the robot marker jumps into the air and the map becomes tilted. Running only Livox + Point-LIO reproduces the failure, so the primary issue is upstream of scan conversion, slam_toolbox, Nav2, and mission_commander.

## Known Good Geometry Baseline

- Use `point_lio_mid360_tilted.yaml` or `point_lio_mid360_light.yaml` with `odometry.enable_2d_mode: False`.
- Keep Point-LIO body frame as `mid360_link`; do not rename it to `base_link`.
- Publish static `mid360_link -> base_link` with pitch about `0.5489 rad` and yaw `pi`.
- Do not put the chassis mount pitch into `mapping.extrinsic_R`; that is the LiDAR-to-MID360-IMU calibration.

## Evidence Collected

- When healthy at startup, `odom -> base_link` roll/pitch is near level, and a 30 s stationary probe showed only about 5 mm odom drift.
- After failure, `odom -> base_link` gained large roll/pitch and z offset; later `/odom` reached tens to hundreds of meters of z error.
- `/cloud_registered` and `/map_cloud` were not just visually wrong: whole point clouds moved to large negative z, e.g. `z=-120~-109 m` and radius around `160 m`.
- LiDAR input remained stable at about `10 Hz`; IMU input remained stable at about `200 Hz`.
- Point-LIO output dropped to about `3 Hz` while input stayed at `10 Hz`.
- Point-LIO logs repeatedly showed `Point-LIO realtime warning: odometry loop overrun` and `dropped stale LiDAR frame because IMU starts after frame begin`.
- With only Livox + Point-LIO running, the same failure mode reproduced, so RViz/slam_toolbox are load amplifiers but not the root cause.

## Working Diagnosis

Point-LIO cannot keep up with the MID360 data and current matching parameters after the car starts moving. Once per-frame processing exceeds the 0.1 s LiDAR interval, frames are delayed or dropped, LiDAR/IMU synchronization degrades, scan matching converges to wrong poses, and bad frames get inserted into the internal map. That causes a feedback loop: polluted map -> worse matching -> more compute and larger pose jumps.

## New Light Test Entry

Use the lightweight LIO-only script before re-enabling scan conversion or downstream mapping:

```bash
cd ~/venom_ws
colcon build --packages-select point_lio venom_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

./src/venom_vnv/venom_bringup/scripts/start_hunter_mid360_lio_light.sh
```

Monitor in another terminal:

```bash
source ~/venom_ws/install/setup.bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /odom
ros2 topic hz /cloud_registered
ros2 run tf2_ros tf2_echo odom base_link
```

Expected health criteria:

- `/livox/lidar` stays near `10 Hz` and `/livox/imu` near `200 Hz`.
- `/odom` should not collapse to `3 Hz`; it should stay close to the LiDAR processing rate.
- Point-LIO log loop time should stay below the `0.1000 s` LiDAR interval, ideally below `0.08~0.09 s`.
- `dropped_stale_lidar` should not keep growing.
- `odom -> base_link` z should not fly to meters/tens of meters, and roll/pitch should remain near level after static mount compensation.
- `/cloud_registered` should remain near the physical scene; whole frames at tens of meters of z error mean LIO already diverged.

## Light Config Changes

`point_lio_mid360_light.yaml` intentionally trades map detail for real-time safety:

- `point_filter_num: 8`
- `filter_size_surf: 0.15`
- `filter_size_map_internal: 0.5`
- `filter_size_map_publish: 0.5`
- `publish.path_en: False`
- `publish.map_publish_en: False`
- `odometry.enable_2d_mode: False`

## Next Improvement Plan

1. First prove light LIO stays real-time for a full driving lap with no downstream consumers.
2. If it still overruns, increase filtering further: try `point_filter_num: 10~12`, `filter_size_surf: 0.2`, and `filter_size_map_internal: 0.8`.
3. Confirm Point-LIO was rebuilt with Release optimization. If loop time remains high, inspect compiler flags and CPU governor.
4. Once LIO-only is stable, add `pointcloud_to_laserscan` back and retest; keep RViz off.
5. Once LIO + scan is stable, add slam_toolbox mapping and retest from an empty map.
6. Add self-filter/crop for vehicle body and mechanical arm if `/scan` contains robot-fixed points.
7. Only after the first-lap static map is clean should Nav2/mission_commander be used.

## Things Not To Do

- Do not continue with a map after a Point-LIO divergence; bad point clouds are already inserted.
- Do not judge fixes using the already tilted or polluted map.
- Do not tune Nav2/TEB before LIO-only is stable.
- Do not use Point-LIO 2D mode with `base_frame_id: mid360_link` for this tilted sensor installation.
- Do not modify `mapping.extrinsic_R` to compensate the chassis mount angle.

# Scout Mini / MID360 Calibration, 2026-11-09

Hardware reported by the operator: chassis length 0.60 m, width 0.57 m including
wheels; LiDAR centered laterally, 0.35 m ahead of chassis center, about 0.26 m high.

## Findings

- The old body-to-chassis transform compensated 53.6 degrees of pitch. Static IMU
  and floor measurements show an installation near 63 degrees. The old chassis
  frame therefore remained tilted by roughly 10 degrees on level ground.
- The projected LaserScan had negative Z in the continuous BIEVR odometry frame.
  Nav2's default observation minimum height of zero discarded its obstacle points.
  This produced an empty local costmap even though the global SLAM map contained
  obstacles. The previously activated lifecycle nodes did not validate this behavior.
- A scan projected at chassis center also cast clearing rays from the wrong origin:
  the MID360 is in front of the chassis.
- The first forward-looking scan put chassis center on/outside the SLAM raster
  border. NavFn seals its outermost cells, so even `allow_unknown: true` could not
  produce a path. The global costmap now uses a 30 m rolling planning window;
  the local costmap and live scan still provide obstacle checking.
- The previously selected goals had no traversable connection in the global
  inflated costmap. Allowing unknown cells does not allow crossing occupied cells.

## New Calibration and Filtering

`venom_bringup/config/scout_mini/bievr/mount.yaml` holds this deployment's transform.
It is separate from the legacy FAST-LIO calibration. BIEVR and TARE's BIEVR mode
load this file, and each BIEVR run keeps a copy alongside its log.

The static sample contains 2001 IMU measurements. The floor fit uses 13327 inliers
with approximately 4.6 mm residual. Its estimated IMU height is 0.2216 m. Applying
the existing factory LiDAR-to-IMU displacement places the LiDAR data origin at
approximately 0.2503 m, consistent with the approximate 0.26 m housing measurement.

Raw floor points and the chassis footprint are removed in calibrated base_link
coordinates before projection. The scan origin is scan_link at the LiDAR's actual
location. Nav2 accepts the projected scan's Z in its odometry/map reference frames;
the physical obstacle-height filtering has already happened upstream.

Footprints use the 0.60 by 0.57 m chassis plus 0.03 m padding. Initial Nav2 speed
is 0.15 m/s. TARE remains a separate, manually enabled entry.

Static calibration cannot identify heading around gravity. An operator-controlled
straight run measured 0.9584 m from wheels and 0.9717 m from BIEVR, with less than
0.3 degrees of turning. It identified a -4.908 degree mounting-heading correction,
which has been included in the final transform and lever arm. This is not a new
LiDAR/IMU temporal or factory-extrinsic calibration.

## Mapping and Saved-Map Localization

```bash
cd /home/hyq/venom_ws/src/venom_vnv
bash tools/bievr/start_bievr.sh mapping
python3 tools/bievr/save_map.py /home/hyq/venom_ws/maps/my_area/map
```

The save command writes the occupancy map and the serialized SLAM Toolbox pose
graph. A PCD snapshot alone is insufficient for this localization mode.
Stop the mapping run before starting localization:

```bash
bash tools/bievr/start_bievr.sh localization map_file:=/home/hyq/venom_ws/maps/my_area/map
```

By default localization starts near map pose [0, 0, 0]. For another
known initial pose add `initial_pose:='[x, y, yaw_radians]'`, or use RViz's 2D Pose
Estimate. This is scan matching against a saved map with an initial pose; arbitrary
position global relocalization is not claimed. The loader rejects saved maps whose
recorded mounting calibration differs from the active calibration.

Old maps and logs are preserved. Rebuild maps made with the incorrect mounting
transform; changing a transform cannot repair their already inserted scans.

## Verified Deployment Results

- `venom_bringup` rebuilt successfully after the final launch/config changes.
  Python syntax, YAML parsing, shell syntax and `git diff --check` passed.
- Isolated BIEVR/Nav2 smoke and both TARE integration modes (BIEVR and FAST-LIO)
  passed. These synthetic checks do not replace physical driving acceptance.
- Saved occupancy map and pose graph are at
  `/home/hyq/venom_ws/maps/scout_calibrated_20261109/map` with `.yaml`, `.pgm`,
  `.posegraph`, `.data` and matching `.mount.yaml` artifacts.
- The explicit-initial-pose localization restart passed live data, TF, seven
  active Nav2 lifecycle nodes, and read-only path computation for both known free
  and unknown cells. Evidence:
  `/home/hyq/venom_ws/runtime/bievr_localization_verified_20261109_155753/live_validation.json`.
  This was a stationary restart near the saved pose, not a kidnapped-robot test.
- Returned to online mapping with TARE off and no active navigation goal:
  `/home/hyq/venom_ws/runtime/bievr_manual_ready_20261109_155918`.
  The latest `live_validation.json` reports all seven Nav2 nodes active, 353 finite
  scan returns, 160 lethal local-costmap cells, and successful read-only paths
  with 32 points (known free) and 38 points (unknown).
- Static 12-second odometry peak-to-peak was [0.00651, 0.00515, 0.00341] m;
  calibrated chassis roll/pitch residual was [0.0116, 0.0171] degrees.
  Scout reported CAN mode, zero speed, and error code zero.
- No real `NavigateToPose` goal or nonzero velocity command was sent for these
  checks. Actual moving map quality, turning behavior, stopping and arrival still
  require operator-supervised physical tests. The rolling global window is 30 m
  across, so first manual goals should be nearby rather than beyond its bounds.

Read-only verification can be repeated with:

```bash
python3 tools/bievr/validate_live.py /tmp/bievr_live_validation.json
```

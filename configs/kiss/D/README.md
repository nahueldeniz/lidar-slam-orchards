# KISS-ICP: configuration D, source verification and output frames

Source: `PRBonn/kiss-icp` tag `v0.3.0` = `460c6f2cc3616c122c42006898036df9c20b636f` (2024-02-22), the last release with the ROS 1 wrapper (v0.4.0 removed it, PR #280). Paths are relative to `env/src/kiss/kiss-icp/`; "launch:N" means `ros/launch/odometry.launch` line N. `odometry.launch` loads no YAML: all parameters are ROS params set in the launch file and read in `ros/ros1/OdometryServer.cpp:55-65`. `config/*.yaml` are only for the Python pipeline.

Files here:

- `kiss_D.launch`: configuration D, derived from `odometry.launch`. RViz and the bag-player group are removed, `visualize` and `deskew` are false, the topic is `/velodyne_points`, and every parameter is explicit. The tuned parameters are arguments with D defaults.

## 1. Brief Table 5.6, checked against the source

All parameters act on the odometry, which is the only output (FE).

| Row | ROS param | launch default (line) | C++ default (`cpp/kiss_icp/pipeline/KissICP.hpp`) | Unit | Use (trace) | Brief | Verdict |
|---|---|---|---|---|---|---|---|
| a | `initial_threshold` | 2.0 (launch:31) | 2.0 (:43) | m | τ0. It is returned while the sensor has moved ≤ 5·`min_motion_th` from the first pose (`pipeline/KissICP.cpp:90-94, 104-108`), and afterwards until the first model deviation larger than `min_motion_th` is recorded (`core/Threshold.cpp:46-47`). ICP then uses 3σ as the correspondence gate and σ/3 as the Geman-McClure scale (`KissICP.cpp:73-75`). Paper Table I: "Initial threshold τ0 = 2 m". | 2.0 | OK. In practice it only matters during start-up. |
| b | `min_motion_th` | 0.1 (launch:32) | 0.1 (:42) | m | δmin: only deviations with model error > δmin enter σ (`Threshold.cpp:41`). It also sets the "has moved" test at 5·δmin (`KissICP.cpp:107`). Paper Table I: "Min. deviation threshold δmin = 0.1 m". | 0.1 | OK. |
| c | `max_points_per_voxel` | 20, a literal in the param line (launch:30) | 20 (:39) | points | Nmax: map voxels stop accepting points once full (`core/VoxelHashMap.hpp:44-46`). Paper Table I: Nmax = 20. | 20 | OK. |
| d | `voxel_size` | arg default `""` (launch:15) → param set to the **empty string** (launch:29) | 1.0 (:36) | m | `pnh_.param("voxel_size", voxel_size, max_range/100.0)` (`OdometryServer.cpp:62`). A string cannot be read as a double, so the fallback max_range/100 = **1.0 m** is used. v is the map voxel size; the scan is down-sampled at 0.5·v (map update) and 1.5·v (registration) (`KissICP.cpp:83-87`, α and β are hard-coded). Paper Table I: "Voxel size map v = 0.01 rmax". | max_range/100 | OK: 1.0 m with max_range 100. `kiss_D.launch` sets 1.0 explicitly, which is numerically identical. Caveat: changing `max_range` would no longer change `voxel_size`, but `max_range` is fixed. |
| e | `min_range` | 5.0 (launch:14) | **5.0 (:38)** as well | m | Points with ‖p‖ ≤ min_range are dropped before everything else (strict `>`, `core/Preprocessing.cpp:69`). It is not a parameter in the paper (Table I lists seven parameters, and the text says rmax is a sensor value). | 5.0 (launch) | OK. Note that the C++ struct and Python defaults are also 5.0 (`python/kiss_icp/config/config.py:31`), so 5.0 is the library default, not only a launch quirk. |

Fixed settings (brief):

- `max_range` 100.0 (launch:13; C++ 100.0, :37). It is used in three places: range crop (`Preprocessing.cpp:69`), local-map radius (`VoxelHashMap.cpp:166-173`), and δ_rot = 2·rmax·sin(θ/2) in the adaptive threshold (`Threshold.cpp:29-34`).
- `deskew` false (launch:12; C++ false, :46). When false, no per-point time field is read (`OdometryServer.cpp:116-119`).
- `visualize` defaults to **true** upstream (launch:5). True starts RViz (launch:41-43) and publishes `/kiss/frame`, `/kiss/keypoints` and `/kiss/local_map` (`OdometryServer.cpp:81-86`). D uses false, and the RViz group is removed.

Not exposed (compile-time constants, unchanged):

- α = 0.5 and β = 1.5 (`KissICP.cpp:85-86`).
- ICP stop at ‖dx‖ < 1e-4 (γ, paper Table I) and at most 500 iterations (`core/Registration.cpp:65-66`). The paper says there is no iteration cap, but the code has one.
- Subscriber queue size **1** (`ros/ros1/OdometryServer.hpp:67`). If processing a scan takes longer than the scan period, the node silently drops scans, so compare `/kiss/odometry` and scan message counts in the manifest.

## 2. Subscriptions and LiDAR-only

The node subscribes to exactly one topic: `pointcloud_topic` (`sensor_msgs/PointCloud2`), remapped to `$(arg topic)` = `/velodyne_points` (`OdometryServer.cpp:75`, launch:20). It reads only the `x, y, z` fields (`ros/ros1/Utils.hpp`, `PointCloud2ToEigen`), and the time field only if `deskew` is true.

It has no IMU, GNSS or wheel-odometry input. Its tf2 listener is used only when `base_frame` is non-empty or `publish_odom_tf` is true; both are off in D. So it is LiDAR-only by construction, and playing only `/velodyne_points` is sufficient.

## 3. Outputs (brief Section 6.1)

| Output | Type | `frame_id` / `child_frame_id` | Meaning | Stamp | Rate |
|---|---|---|---|---|---|
| `/kiss/odometry` (absolute name, `OdometryServer.cpp:79`) | `nav_msgs/Odometry` | `odom` / **empty** (never set, :168-172) | With `base_frame` empty (`egocentric_estimation`, :120), the pose is the KISS pose of the **LiDAR (cloud) frame** in `odom`: T_odom←lidar (:126, :130). `odom` is the LiDAR frame at the first scan: with an empty map, `RegisterFrame` returns the initial guess = identity. Twist and covariance are zero. | `msg->header.stamp` of the input scan, copied **exactly** (ns preserved) (:136, :169). | One per processed scan; scans are dropped silently if the node falls behind (queue 1). |
| `/kiss/trajectory` | `nav_msgs/Path` | `odom` | The whole path, republished every scan; it grows without bound. **Do not record** (brief 6.1). | – | – |
| TF `odom`→cloud frame | – | – | Only if `publish_odom_tf` (false in D). | – | – |

**LiDAR-frame pose (brief Section 10):** T_odom←lidar(t) = pose of `/kiss/odometry` at stamp t, with no further transform. The `child_frame_id` is empty in the message, so `eval/frames.md` must state the child frame (the bag's LiDAR `frame_id`) explicitly.

## 4. Open questions (KISS)

1. **`min_range` levels** (see `../tuning_rationale.md`): the ×2 level (10 m) is proposed to be replaced by the submitted 0.5 m. Nahuel to confirm.
2. **Dropped scans:** with queue size 1, only the newest waiting scan is kept, so whenever processing falls behind the LiDAR rate, older waiting scans are discarded. This cannot be changed without a code patch (not allowed). Report the number of output messages divided by the number of input scans per run.
3. **Child frame:** `/kiss/odometry` leaves `child_frame_id` empty. The evaluation must take the LiDAR frame name from the inventory.

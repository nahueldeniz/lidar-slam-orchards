# SLAMesh: brief vs source (verification record)

Checked against `env/src/mesh/SLAMesh` at `50b4356e8a24c4fe2e2dc1be37d682e703843314` (master, 2025-01-05), on 12 October 2026. Paths are relative to that clone. `node.cpp` means `src/slamesher_node.cpp`.

## 1. Version and packaging (brief Section 3)

| Brief | Source | Verdict |
|---|---|---|
| `master`, pin a commit | `50b4356…` (2025-01-05) | pinned |
| Base config `config/online.yaml`, launch `slamesh_online.launch` | `launch/slamesh_online.launch:7` loads `config/online.yaml` at global scope (keys become `/slamesher/*`) | OK |
| Remove RViz | `slamesh_online.launch:19` (`required="false"`) | removed in D |
| Remap the input | The node subscribes to `/velodyne_points` (`node.cpp:920`); upstream remaps it to `/os_cloud_node/points` (`slamesh_online.launch:13`) | D maps it to `/velodyne_points` |
| Official image `pleaserun/rjy_slam_work:slamesh_18.04` "used in 2024" | The image was pushed 2023-08-14. It is a VNC desktop base (2020) plus one `docker commit` layer with no build recipe. The README tells users to clone and build SLAMesh **inside** it, so the image does not fix the SLAMesh version | The 2024 SLAMesh commit is unknown (open question) |
| Or `docker/Dockerfile` (`osrf/ros:melodic-desktop-full`) | Clones the lvr2, mesh_tools (`-b noetic`) and SLAMesh default branches **unpinned**. Today lvr2's default branch is `main` (25.x, CMake ≥ 3.16), which cannot configure on 18.04 | Not reproducible as written; see `env/docker/mesh/BUILD_NOTES.md` |
| Needs mesh_tools (ROS 1, built from source) | `naturerobots/mesh_tools`, branch `noetic`, pinned at `95433f1`. Only `mesh_msgs` and `mesh_msgs_conversions` are needed (`CMakeLists.txt:25-33`; message types only, plus `find_package(LVR2 2)`) | OK |

## 2. Table 5.7, row by row

The YAML is `config/online.yaml`. Code fallbacks are in `node.cpp:486-543` and are used only if a key is missing.

| Row | Key | YAML line | YAML default | Code fallback | Unit and use in code | Verdict |
|---|---|---|---|---|---|---|
| a | `range_max` | 12 | 60 | 100 | m. Points with range ≥ range_max are dropped, after the voxel filter and in the sensor frame (`map.cpp:60-74`). **Only if range_max ≤ 100**; above 100 there is no range filter at all, and `range_min` is ignored too (`map.cpp:75-78`). Also sizes `cells_now` (`map.cpp:105`). | OK |
| b | `range_min` | 13 | 0.5 | 1.0 | m. Points with range ≤ range_min are dropped (`map.cpp:70`). **Changed from 3.0 to 0.5 on 2025-01-05** (commit 5f56fd1). | OK for current master; P1 = old default |
| c | `register_times` | 14 | 5 | 5 | **Maximum** iterations of the scan-to-map loop (`map.cpp:1073, 1084-1122`). The loop stops earlier when the update norm is < 1e-5 (`converge_thr`, hard-coded at `node.cpp:541`) or after 4.5 s wall time (`node.cpp:840`, `map.cpp:1118`). The last iteration uses cross-cell length 0. | OK; it is a maximum |
| d | `num_margin_old_cell` | 17 | −1 | −1 | **Scan steps**, not time. Global cells not updated for at least this many steps are excluded from registration and from map update (`map.cpp:338-339, 369-370, 403-404`); −1 = never. **Changed from 500 to −1 on 2025-01-05.** | OK for current master; P1 = old default |
| e | `grid` | 26 | 1.2 | 1.0 | m, cell edge (`map.cpp:115-131, 209-224`; `cell.cpp:74-76`). Also sets the input voxel leaf `voxel_size = grid / num_test` (`node.cpp:521`), i.e. 0.2 m in D. | OK; note the coupling |
| f | `min_points_num_to_gp` | 27 | 3 | 8 | Points. A cell is GP-reconstructed only if it holds **more than** this many points (`map.cpp:170-173, 267-270`). **Confirmed: minimum points for Gaussian-process reconstruction, not ground-plane detection.** | OK (the manuscript wording must change) |
| g | `num_test` | 28 | 6 | 10 | GP test locations per cell side, giving num_test² vertices per cell and direction (`cell.cpp:73-105`). Also enters `voxel_size`. | OK |
| h | `variance_min` | 33 | 1.36 | 5.0 | Added to the residual weight denominator **only** in the point-to-point solver `Map::computeT` (`map.cpp:949-950`). With `point2mesh: true` (YAML line 18) registration always uses `Map::computeTPointToMesh`, where it is read into an unused local (`map.cpp:995`). **Inactive in D.** | **Discrepancy: listed as tunable but inactive** |
| i | `variance_sensor` | 34 | 0.02 | 0.1 | GP observation noise, added **squared** to the kernel diagonal (`cell.cpp:77, 170`). It is therefore a standard deviation in m (variance 4e-4 in D). The paper gives σ_in² = 0.02. | Value OK; name and paper suggest a variance, the code uses a std |
| rec. | `num_thread` | 25 | 8 | 1 | OpenMP threads for per-cell GP reconstruction (`map.cpp:177, 277`); Ceres runs with 1 thread (`map.cpp:976, 1052`). Runtime only. | recorded |
| rec. | `visualisation_type` | 23 | 3 | 1 | 0: registered raw points, pose and path messages only; 1: + vertex clouds; 2: + local mesh per scan; 3: + global mesh every 50 scans (`node.cpp:734-799`). Visualisation only reads the map. **D sets 0.** | recorded |

Other values (recorded): `grt_available: false`, `odom_available: false`, `read_offline_pcd: false`, `dataset: 1`, `max_steps: 10000` (only pre-sizes log arrays, which grow automatically; it does **not** stop processing, `node.cpp:89-117, 878`), `save_mesh_map: true`, `cross_overlap: true` (never used), `cross_cell_overlap_length: 1`, `point2mesh: true`, `residual_combination: false` (**changed from true on 2025-01-05**), `meshing_tsdf: false` (never used), `full_cover: true`, `variance_register: 0.60` (vertex variance gate for matching), `variance_map_update: 0.50`, `variance_map_show: 0.20` (visualisation and saved mesh), `test_param: 0` (never used). `use_sim_time: true` in this file sets `/slamesher/use_sim_time`, not the global flag, and the code never reads it. Read by code but absent from the YAML, and written explicitly in D with their code defaults: `imu_feedback: false`, `range_unit: 1.0` (unused), `correction_{x,y,z,roll_degree,pitch_degree,yaw_degree}: 0`.

Hard-coded values (not parameters): kernel `exp(-1.2·d)` (`cell.cpp:78`; the paper says κ = 1); PCA thresholds 48/0.95/0.2 (`node.cpp:538-540`); Ceres `DENSE_QR` + `LAPACK`, 10 iterations, `HuberLoss(0.1)`, 1 thread (`map.cpp:1004, 1047-1052`).

## 3. Topics, frames and timestamps (brief Sections 6.1 and 10)

**Subscriptions** (`node.cpp:907-920`):

| Topic | Type | Condition | D |
|---|---|---|---|
| `/velodyne_points` (queue 100) | `sensor_msgs/PointCloud2` (read as `PointXYZ`) | always | remapped to `$(arg lidar_topic)`, default `/velodyne_points` |
| `/pose`, `/mavros/local_position/odom` | ground truth | only if `grt_available` | not created |
| `/odom`, `/mavros/global_position/rel_alt` | odometry, altitude | only if `odom_available` | not created |
| IMU (`/imu/data`) | | commented out in code (`node.cpp:916`) | never created |

LiDAR only requires `grt_available: false` and `odom_available: false` (both upstream values), plus `imu_feedback: false` (code default, now explicit). Scans are buffered in an unbounded deque that is filled at every `spinOnce()`. They are processed first-in first-out (`map.cpp:24-44`), so no scan is dropped unless more than 100 arrive between two steps. If processing is slower than real time, the output lags and SLAMesh keeps working after playback ends. On shutdown it finishes the current step and stops, so any scans still queued are lost.

**Output evaluated:**

| Topic | Type | frame_id | child_frame_id | Stamp | Rate |
|---|---|---|---|---|---|
| `/lidar_odometry` (queue 1) | `nav_msgs/Odometry` (pose only; twist and covariances zero) | `/map` (with a leading slash) | `/slamesher_odom` | `header.stamp` = `header.stamp` of the scan processed in that step (`node.cpp:807-813`, buffer set at `map.cpp:42`), because `read_offline_pcd` is false | one per processed scan, from the 2nd scan on (scan 1 builds the initial map) |
| TF `/map` → `/slamesher_odom` | | | | same stamp | same |

**Timestamps: the scan stamp, not `ros::Time::now()`.** `ros::Time::now()` is used only when `read_offline_pcd` is true (KITTI files), and for the visualisation topics (`/raw_points_in_world`, `/pose_pub`, mesh messages; `src/utility/tool.cpp:79, 96, 113`, `map.cpp:1229, 1292`), which are not evaluated. No logging patch is needed. **History:** before commit `acabece` (committed 2024-05-16) `g_data.pcl_msg_buff` was never filled. `/lidar_odometry` then had `header.stamp = 0` and empty frame ids. Any 2024 run made with an earlier SLAMesh checkout could not have been evaluated by header stamps.

**Which frame the pose refers to:** the pose is T_map←L(t), the **LiDAR frame** (labelled `/slamesher_odom`) in `map`. With no ground truth and no odometry input, `map` is the LiDAR frame of the first scan, multiplied by `createTrans(correction_*)`, which is identity in D (`node.cpp:353-358`). The install-transform line is commented out (`map.cpp:58`), so no extrinsic is applied. To get the LiDAR-frame pose (brief Section 10), use the Odometry pose as it is. Its orientation is built through `getEulerYPR`/`setRPY` (`src/utility/tool.cpp:160-170`), which is a lossless round trip. The KITTI-format `*_pred.txt` that SLAMesh writes is **not** in the LiDAR frame: for `seq` `/00` the KITTI velodyne→camera extrinsic is applied (`node.cpp:246-279`). It has no stamps; do not use it.

## 4. Discrepancies with the brief (summary)

1. Row h `variance_min` is **inactive** with the default `point2mesh: true`.
2. Row i `variance_sensor` is a standard deviation in code (squared), not a variance.
3. Rows b and d (and `residual_combination`, which is not in the table) changed defaults on 2025-01-05: `range_min` 3.0 → 0.5, `num_margin_old_cell` 500 → −1, `residual_combination` true → false. The submitted P1 (`range_min` 3, margin 500) equals the old defaults.
4. Row d is in scan steps, not time.
5. Row c is a maximum; the loop can stop earlier on convergence.
6. Row a above 100 m disables range filtering entirely, including `range_min`.
7. The official image does not pin SLAMesh (it is built inside the container), and the repository Dockerfile is unpinned and no longer builds on 18.04 as written.
8. Brief Section 6.1: `/lidar_odometry` is confirmed (`nav_msgs/Odometry`, scan stamps).

## 5. Open questions

1. Which SLAMesh commit was used in 2024? If it predates 2024-05-16, the submitted SLAMesh trajectories had zero header stamps, so their time association must have used bag record times (Section 1, P8).
2. `num_thread: 8` (upstream) while the laptop has 16 logical CPUs: keep it? It is runtime only.
3. Protocol step 7 (maximum 60 s wait after playback): SLAMesh drops its remaining queue at shutdown. Check that the last `/lidar_odometry` stamp equals the last scan stamp in every run, or allow a longer wait for SLAMesh.
4. `save_mesh_map: true` (upstream) writes a PLY at shutdown, after the measured period. Keep it, or set it to false to shorten shutdown? Either choice has no trajectory effect.
5. SLAMesh stops for good (saves and exits its loop) if one scan has **no** points left after range filtering (`map.cpp:80-83`, `node.cpp:850-853`). This is unlikely with a VLP-16, but a run that ends early should be checked for it.

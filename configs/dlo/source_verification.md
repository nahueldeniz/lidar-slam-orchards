# DLO: brief vs source (verification record)

Checked against `env/src/dlo/direct_lidar_odometry` at `11528c02e89a1619ee3c42ba39ecaa992bd20bf4`, on 12 October 2026. Paths are relative to that clone. `odom.cc` means `src/dlo/odom.cc`.

## 1. Version (brief Section 3)

| Brief | Source | Verdict |
|---|---|---|
| `master`, latest commit `11528c0`, May 2023 | `master` HEAD = `11528c02e89a…`, committed 2023-05-19 | OK |
| `dlo.yaml` version 1.4.3 | `cfg/dlo.yaml:12` `version: 1.4.3`. Tag `v1.4.3` = `ba00a42` (2023-04-10); master adds only the `save_traj` service on top of it | OK |
| `dlo.launch` always starts RViz | `launch/dlo.launch:52`, unconditional; the `rviz` arg (line 13) is declared but never used | OK |
| Outputs under `/robot/dlo/odom_node/` | Node `/robot/dlo_odom`; `~odom` and `~pose` are remapped to `dlo/odom_node/odom` and `dlo/odom_node/pose` inside namespace `robot` (`dlo.launch:12, 19, 30-31`) | OK |

## 2. Table 5.5, row by row

Every value below is the YAML value loaded by `dlo.launch`. The code fallback (used only if the key were missing) is given because several fallbacks differ from the YAML values.

| Row | Key | File:line | YAML default | Code fallback (`odom.cc` line) | Unit and use in code | Verdict |
|---|---|---|---|---|---|---|
| a | `preprocessing/cropBoxFilter/size` (`use: true`, l. 28) | `cfg/params.yaml:29` | 1.0 | 1.0 (232); `use` fallback **false** (231) | m, **half edge**. `CropBox` with `setNegative(true)`, min (−s,−s,−s), max (s,s,s) (`odom.cc:123-125`), applied to every scan (459-462). D removes a 2 × 2 × 2 m cube. [DLO] Sec. II-B describes "a box filter of size 1 m³". | Value OK; **unit/meaning to correct**: half edge, not edge or volume |
| b | `preprocessing/voxelFilter/scan/res` (`use: true`, l. 32) | `cfg/params.yaml:33` | 0.25 | 0.05 (236); `use` fallback true (235) | m, leaf of the per-scan `VoxelGrid` (127, 465-468). Enabled by default. | OK |
| c | `preprocessing/voxelFilter/submap/res` (`use: true`, l. 35) | `cfg/params.yaml:36` | 0.5 | 0.1 (238); `use` fallback **false** (237) | m, leaf applied to each keyframe cloud (in world frame) when it is stored (492-495, 1165-1168), not to the concatenated submap. Enabled by default (YAML). | OK; note the meaning |
| d1 | `keyframe/threshD` | `cfg/params.yaml:39` | 5.0 | 0.1 (208) | m. **Inactive** with `adaptiveParams: true`: overwritten on every scan (see Section 3) | OK (inactive, as stated) |
| d2 | `keyframe/threshR` | `cfg/params.yaml:40` | 45.0 | 1.0 (209) | deg, rotation between current pose and nearest keyframe (1142-1158). A rotation-only keyframe requires dd ≤ threshD **and** at most one keyframe within 1.5 × threshD. Not changed by `adaptiveParams`. [DLO] Sec. II-E-2 states 30°. | OK; paper and code differ (30° vs 45°) |
| e | `submap/keyframe/knn`, `kcv`, `kcc` | `cfg/params.yaml:44-46` | 10, 10, 10 | 10, 10, 10 (212-214) | Number of keyframes taken from: all keyframes by distance (1269), the convex hull (1285, ≥ 4 keyframes), and the concave hull (1301, ≥ 5 keyframes, alpha = current threshD). Ties at the k-th distance are all included (1236-1239). The concave hull is not in the paper. | OK |
| f | GICP | `cfg/params.yaml:52-71` | `minNumPoints` 10; s2s: k 10, maxCorrDist 1.0 m, maxIter 32, transEps 0.01, eucFitEps 0.01, RANSAC 5 / 1.0; s2m: k 20, 0.5 m, 32, 0.01, 0.01, RANSAC 5 / 1.0 | 100; 20, ∞, 64, 5e-4, −∞, 0, 0.05 (249-263) | Used in 101-115. **Inactive**: `euclideanFitnessEpsilon` and both RANSAC values (NanoGICP's own optimiser, `include/nano_gicp/impl/lsq_registration_impl.hpp:89-115`, never reads them). `transformationEpsilon` is the translation convergence test; the rotation epsilon is fixed at 2e-3. `kCorrespondences` is the neighbour count for covariances. | OK (not varied) |
| fixed | `dlo/imu` | `cfg/dlo.yaml:16` | **true** | false (244) | With `true`, `initializeDLO()` returns until the IMU is calibrated (594-596). With no IMU messages DLO never initialises and publishes nothing. Must be `false`. | OK (default true, as stated) |
| fixed | `dlo/adaptiveParams` | `cfg/dlo.yaml:14` | **true** | false (241) | See Section 3 | OK |

Other values (recorded, not in the table): `gravityAlign: true` (`dlo.yaml:17`, inactive without IMU); `initialPose/use: false` (`params.yaml:15`); `imu/calibTime: 3`, `bufferSize: 2000`; `mapNode`: `publishFullMap: true`, `publishFreq: 1.0` (used as a timer **period** in s, `map.cc:26`), `leafSize: 0.25`.

Hard-coded values (not parameters): spaciousness low-pass filter 0.95/0.05 and threshold bins (`odom.cc:1009, 1196-1204`); NanoGICP rotation epsilon 2e-3 and 10 Levenberg–Marquardt iterations (`lsq_registration_impl.hpp:52-58`); OpenMP threads = `omp_get_max_threads()` (`nano_gicp_impl.hpp:50-55`), i.e. all logical CPUs unless `OMP_NUM_THREADS` is set.

## 3. Adaptive keyframing (brief C5 and Section 5.5)

- Called on every scan after preprocessing, before the first keyframe and before every keyframe test (`odom.cc:661-664`).
- Mapping (`odom.cc:1193-1209`, [DLO] Eq. 6): spaciousness > 20 m gives 10 m; (10, 20] m gives 5 m; (5, 10] m gives 1 m; ≤ 5 m gives 0.5 m. **Confirmed: 0.5/1/5/10 m.**
- It changes only `keyframe_thresh_dist_` (used as the keyframe distance threshold and in the "nearby" count of 1.5 × threshold, 1120) and the concave-hull alpha (1207). It does **not** change `threshR`, `knn`/`kcv`/`kcc` or any voxel size.
- Spaciousness = low-pass filtered median range of the preprocessed scan (after crop and voxel filters), computed in a detached thread (`odom.cc:658-659, 995-1015`). `setAdaptiveParams()` therefore usually reads the previous scan's value. This is an unsynchronised access in upstream code; it is recorded here and not changed.
- DLO does not log the threshold. Patch `env/patches/dlo/0002-log-adaptive-keyframe-threshold.patch` adds `ROS_INFO` lines:
  - `[DLO_KF] kf=<n> stamp=<scan stamp> threshD_m=… threshR_deg=… dd_m=… dtheta_deg=… num_nearby=… adaptive=1` at every keyframe, including the first;
  - `[DLO_KF_THRESH] stamp=… threshD_m=… prev_threshD_m=…` whenever the threshold changes. The first line shows the YAML 5.0 being replaced.
  The lines go to the node's stdout and to `/rosout`, so the infrastructure container's `rosout.log` has them as complete lines. The DLO stdout also contains its per-scan full-screen status printout, with ANSI clear-screen codes.

## 4. Topics, frames and timestamps (brief Sections 6.1 and 10)

**Subscriptions** (`odom.cc:31-32`, both queue size 1):

| Private name | Upstream remap (`dlo.launch`) | Resolves to | D launch |
|---|---|---|---|
| `~pointcloud` (`sensor_msgs/PointCloud2`, read as `PointXYZI`) | `$(arg pointcloud_topic)`, default `lidar` | `/robot/lidar` | `/velodyne_points` |
| `~imu` (`sensor_msgs/Imu`) | `$(arg imu_topic)`, default `imu` | `/robot/imu` | `/robot/dlo/imu_unused` |

The map node subscribes to `/robot/dlo/odom_node/pointcloud/keyframe`. LiDAR only requires `dlo/imu: false`; `imuCB` then returns at once (`odom.cc:711-713`). The D launch also points `~imu` at a topic that nobody publishes. The subscriber queue of 1 means that a scan arriving while the previous one is still being processed replaces the waiting scan. Check the output count against the input count.

**Outputs:**

| Topic | Type | frame_id | child_frame_id | Stamp | Rate |
|---|---|---|---|---|---|
| `/robot/dlo/odom_node/odom` | `nav_msgs/Odometry` (pose only; twist and covariances zero) | `robot/odom` | `robot/base_link` | `header.stamp` = input scan `header.stamp` (`odom.cc:637, 362`) | one per processed scan, from the 3rd accepted scan on (scan 1 only initialises, scan 2 becomes the first keyframe) |
| `/robot/dlo/odom_node/pose` | `geometry_msgs/PoseStamped` | `robot/odom` | none | same scan stamp (367) | same as above |
| `/robot/dlo/odom_node/odom/keyframe` | `nav_msgs/Odometry`, latched | `robot/odom` | `robot/base_link` | keyframe scan stamp | one per keyframe (useful to record as a cross-check of the `[DLO_KF]` log) |
| TF `robot/odom` → `robot/base_link` | | | | scan stamp (392) | per scan |

The frame names get the namespace prepended in code (`odom.cc:196-202`). `scan_stamp` is copied from the incoming cloud, so **the output stamps are the scan stamps, not `ros::Time::now()`**. There is one caveat. Publication runs in a detached thread that reads the shared members `pose`, `rotq` and `scan_stamp` without a lock (`odom.cc:695-696`). If the next scan's callback starts before that thread runs, a pose could go out with the next scan's stamp. This is unlikely at 10 Hz, but the evaluation should check that the output stamps are a strictly increasing subset of the input stamps and report any duplicates.

**Which frame the pose refers to:** DLO 1.4.3 has **no extrinsic parameters** (no `baselink2lidar` or `baselink2imu`; those exist only in DLIO). The cloud's own `header.frame_id` is ignored, and the points are registered in the frame in which they arrive. The pose is therefore T_W←L(t): the pose of the **LiDAR frame** (labelled `robot/base_link`) in a world frame W. W is the LiDAR frame at the 2nd accepted scan, because `initialPose/use` is false. With `imu: false` there is no gravity alignment, so W is tilted if the LiDAR is tilted. To get the LiDAR-frame pose (brief Section 10), use the Odometry pose as it is: no extra transform is needed. The first output pose is close to identity.

## 5. Discrepancies with the brief (summary)

1. Row a: `cropBoxFilter/size` is a **half edge**. The default removes a 2 m cube, while the DLO paper describes a 1 m³ box.
2. Row c: the submap voxel leaf is applied to each stored keyframe cloud, not to the assembled submap.
3. Row d2: the code default is 45°, but the paper fixes the rotational threshold at 30°.
4. Row e: `kcc` (concave hull) is in the code but not in the paper.
5. Row f: three GICP parameters (`euclideanFitnessEpsilon`, `ransac/iterations`, `ransac/outlierRejectionThresh`) are inactive.
6. `dlo/imu: true` with no IMU does not merely "use the IMU": DLO never initialises and produces **no output**.
7. Build hint (Section 3): PCL ≥ 1.10 and CMake ≥ 3.16.3 are not needed; see `env/docker/dlo/BUILD_NOTES.md`.

## 6. Open questions

1. Keep the DLO map node in D and T (as upstream)? It does not affect odometry, but it adds CPU and memory: every second it converts the whole map to a ROS message, even when nobody subscribes.
2. Leave `OMP_NUM_THREADS` unset (upstream behaviour: 16 threads on the benchmark laptop) or fix it for all methods? Record it either way. NanoGICP sums its errors with OpenMP reductions under `schedule(guided)` (`nano_gicp_impl.hpp:225, 276`). The summation order, and so the last bits of the result, can therefore vary between runs and with the thread count. This is one source of the run-to-run spread that Section 5.0 measures.
3. Record `/robot/dlo/odom_node/odom/keyframe` in addition to the brief's topics? It is small, and it lets the keyframe count be checked against the `[DLO_KF]` log.
4. To capture the `[DLO_KF*]` lines, save the infra container's `rosout.log`, or the DLO container's stdout (which also contains ANSI escape codes).
5. The DLO version used in 2024 is unknown (C2). Building today's master on Melodic needs build-fix patch 0001, so the 2024 build either used a newer CMake or a different commit.

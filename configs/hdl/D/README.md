# hdl Graph SLAM: configuration D, source verification and output frames

Source: `koide3/hdl_graph_slam` @ `95b8dce41c10667dcf0e3e9801dc22754b3bd525` (master, 2024-07-16), with `ndt_omp` @ `6f4e5f3` (branch `melodic`), `fast_gicp` @ `0e7ec14` (master) and `ros-melodic-libg2o` 2018.3.25 (see `env/src_pins_hdl_kiss.json`). Paths below are relative to `env/src/hdl/hdl_graph_slam/` unless stated otherwise; "launch:N" means `launch/hdl_graph_slam_400.launch` line N. The line numbers of `src/hdl_graph_slam/keyframe.cpp` are before patch 0001.

Files here:

- `hdl_D.launch`: configuration D. It is derived from `hdl_graph_slam_400.launch`, and every line that differs from upstream carries a comment. `sensor_height` and `tilt_deg` must be passed per dataset; the tuned parameters are arguments whose defaults are the D values.

## 1. Brief Table 5.1, checked against the source

FE means `scan_matching_odometry_nodelet` (publishes `/odom`). BE means `hdl_graph_slam_nodelet` (graph, `odom2pub`, dump). Prefiltering output (`/filtered_points`) feeds FE, floor detection and BE.

| Row | Parameter (node) | launch default (line) | C++ default (file:line) | Unit | Used by (trace) | Brief | Verdict |
|---|---|---|---|---|---|---|---|
| a | `downsample_resolution` (prefiltering) | 0.1, VOXELGRID (launch:35, :34) | 0.1 (`apps/prefiltering_nodelet.cpp:52`) | m (voxel leaf) | `/filtered_points` → FE, floor, BE | 0.1, FE+BE | OK. **Ambiguity:** FE has its own `downsample_resolution` (launch:57 = 0.1), but it is unused because FE `downsample_method` is NONE (launch:56 → pass-through, `scan_matching_odometry_nodelet.cpp:96-104`). The tuned one is the prefiltering parameter. |
| b | `radius_radius` (prefiltering) | 0.5 (launch:40), with `outlier_removal_method` RADIUS (launch:37) and `radius_min_neighbors` 2 (launch:41) | 0.8 (`prefiltering_nodelet.cpp:83`); method default STATISTICAL (:72); min neighbours 2 (:84) | m | `/filtered_points` → FE, floor, BE | 0.5; "proposed" RADIUS + 2 | OK. The proposed method and neighbour count are already the launch defaults. Code defaults differ from the launch, so the launch must be used. |
| c | `keyframe_delta_trans` | FE 1.0 (launch:49); BE 2.0 (launch:98) | FE 0.25 (`scan_matching_odometry_nodelet.cpp:74`); BE 2.0 (`include/hdl_graph_slam/keyframe_updater.hpp:23`) | m | FE: when the translation from the current FE keyframe exceeds the threshold, the registration target is replaced (`scan_matching_odometry_nodelet.cpp:241-252`), which changes `/odom` and hence BE. BE: a new graph node is added when dx ≥ threshold (`keyframe_updater.hpp:43-50`). This only changes BE outputs. | 1.0 (odometry); 2.0 (graph); FE+BE | Defaults OK. **Discrepancy in "Affects":** the FE parameter affects FE+BE, but the BE parameter affects BE only. They are two different parameters, so they are tuned separately (`tuning_levels.yaml`). |
| d | `keyframe_delta_angle` | FE 1.0 (launch:50); BE 2.0 (launch:99) | FE 0.15 (`scan_matching_odometry_nodelet.cpp:75`); BE 2.0 (`keyframe_updater.hpp:24`) | rad | FE compares `acos(q.w)`, which is **half** the rotation angle (`scan_matching_odometry_nodelet.cpp:242`). The FE default 1.0 therefore triggers at a 2.0 rad (115°) rotation, and any value ≥ π/2 never triggers. BE compares the full angle-axis angle (`keyframe_updater.hpp:45`), and any value ≥ π never triggers. | 1.0 / 2.0; FE+BE | Defaults OK. **Discrepancies:** (i) the FE and BE parameters have different semantics: FE uses the half-angle; (ii) "Affects" is split as in row c. |
| e | `registration_method` | FAST_GICP (FE launch:59, BE launch:110) | NDT_OMP (`src/hdl_graph_slam/registrations.cpp:26`) | – | FE odometry and BE loop-closure matching (`include/hdl_graph_slam/loop_detector.hpp:47`) | FAST_GICP, FE+BE | OK (fixed). |
| f | `reg_max_correspondence_distance` | 2.0 (FE launch:63, BE launch:114) | 2.5 (`registrations.cpp:33`) | m | **Used with FAST_GICP.** `select_registration_method` calls `setMaxCorrespondenceDistance` in the FAST_GICP branch (`registrations.cpp:27-35`). This sets `pcl::Registration::corr_dist_threshold_`, which `FastGICP::update_correspondences` uses to drop nearest neighbours farther than the threshold (`fast_gicp/include/fast_gicp/gicp/impl/fast_gicp_impl.hpp:139`). FE: odometry. BE: loop-closure alignment only. | 2.0, FE+BE | OK; kept as a tuning candidate. |
| g | `tilt_deg` (floor_detection) | 0.0 (launch:74) | 0.0 (`apps/floor_detection_nodelet.cpp:57`) | deg | The cloud is rotated about y by `tilt_deg` before the height clipping (:113-119), and the verticality check uses the tilted z axis (:152-161). Floor coefficients go only to BE floor edges. | 0, BE | Default OK, BE OK. **Nature:** it is a sensor-mounting setting (LiDAR pitch), like `sensor_height`; it is passed per dataset in `hdl_D.launch` and not tuned (see `../tuning_rationale.md`). |
| h | `sensor_height` (floor_detection) | 2.0 (launch:75) | 2.0 (`floor_detection_nodelet.cpp:58`) | m | Points with z in [−h−r, −h+r) of the `base_link` frame are RANSAC candidates (:118-119, r = `height_clip_range` = 1.0, launch:76). BE floor edges only. | 2.0, BE | OK. Because `lidar2base_publisher` is the identity, h is the height of the **LiDAR** origin above the floor. |

Fixed settings in the brief:

| Setting | Where | Default | Note |
|---|---|---|---|
| `enable_floor_detection: true` | launch arg (launch:5) | true | Controls only whether `floor_detection_nodelet` is started (launch:72 `if=`). BE always subscribes to `/floor_detection/floor_coeffs` (`hdl_graph_slam_nodelet.cpp:117`). |
| `enable_gps: false` | launch arg (launch:6) → BE param (launch:93) | launch false; **C++ default true** (`hdl_graph_slam_nodelet.cpp:119`) | `hdl_D.launch` hard-wires `false`. |
| `enable_imu_acc: false` | launch **arg** (launch:7) → ROS param `enable_imu_acceleration` (launch:94) | false; C++ false (:105) | The brief uses the arg names; the ROS parameter names differ. |
| `enable_imu_ori: false` | launch arg (launch:8) → ROS param `enable_imu_orientation` (launch:95) | false; C++ false (:104) | Same as above. |

Other inputs that could break "LiDAR only" but are not in the brief (all default false, now explicit in `hdl_D.launch`):

- `deskewing` (prefiltering, `prefiltering_nodelet.cpp:40`): when true, it subscribes to `/imu/data`.
- `enable_imu_frontend` (FE, `scan_matching_odometry_nodelet.cpp:48`): when true, it subscribes to `/msf_core/pose` and `/msf_core/pose_after_update`.
- `enable_robot_odometry_init_guess` (FE, launch:15/:53): when true, it uses wheel-odometry TF (`robot_odom`).

Other facts that matter for the evaluation:

- `use_distance_filter` is read but ignored: the distance filter 0.1–100 m always runs (`prefiltering_nodelet.cpp:95, 131`).
- `reg_num_threads 0` means all OpenMP threads.
- `graph_update_interval` (3 s) is a **wall-clock** timer (`hdl_graph_slam_nodelet.cpp:137-139`), so BE behaviour depends on the playback rate (1.0 in the protocol).

## 2. Subscriptions (default names) and what keeps the run LiDAR-only

| Node | Topic (type) | Condition |
|---|---|---|
| prefiltering | `/velodyne_points` (PointCloud2 as `pcl::PointCloud<PointXYZI>`), remapped to `points_topic` (default `/velodyne_points`) | always (`prefiltering_nodelet.cpp:44`) |
| prefiltering | `/imu/data` (Imu) | only if `deskewing` (:40-41) |
| prefiltering | TF `base_link` ← cloud frame, latest available (`ros::Time(0)`) | always when `base_link_frame` is set (:115-129) |
| FE | `/filtered_points` | always (`scan_matching_odometry_nodelet.cpp:53`) |
| FE | `/msf_core/pose`, `/msf_core/pose_after_update` | only if `enable_imu_frontend` (:48-50) |
| floor_detection | `/filtered_points` | always (`floor_detection_nodelet.cpp:44`) |
| BE | `/odom` + `/filtered_points` (ApproximateTime, queue 32) | always (`hdl_graph_slam_nodelet.cpp:112-115`) |
| BE | `/gpsimu_driver/imu_data` (Imu) | always subscribed; messages are dropped unless `enable_imu_*` is true (:116, :361) |
| BE | `/floor_detection/floor_coeffs` (hdl_graph_slam/FloorCoeffs) | always (:117) |
| BE | `/gps/geopoint` (GeoPointStamped), `/gpsimu_driver/nmea_sentence` (nmea_msgs/Sentence), `/gps/navsat` (NavSatFix) | only if `enable_gps` (:119-123) |
| map2odom_publisher | `/hdl_graph_slam/odom2pub` | always (`src/hdl_graph_slam/map2odom_publisher.py:13`) |

LiDAR only requires two things: play only the LiDAR topic (`--topics /velodyne_points`), and keep the six switches above false (`enable_gps`, `enable_imu_acceleration`, `enable_imu_orientation`, `deskewing`, `enable_imu_frontend`, `enable_robot_odometry_init_guess`). A pre-run check with `rostopic info` should show no publisher on `/imu/data`, `/gpsimu_driver/imu_data`, `/gps/*` or `/msf_core/*`.

## 3. Outputs (brief Section 6.1)

| Output | Type | `frame_id` → `child_frame_id` | Meaning | Stamp | Rate |
|---|---|---|---|---|---|
| `/odom` (FE) | `nav_msgs/Odometry` | `odom` → `base_link` (the cloud frame after prefiltering) | T_odom←base_link. Identity at the first scan. Twist and covariance are zero. (`scan_matching_odometry_nodelet.cpp:269-293`) | Input scan stamp **truncated to whole microseconds**: `pcl::PointCloud` headers store µs (pcl_conversions), so ns digits are lost in the prefilter. | One per input scan (also when matching did not converge: the previous pose is repeated, :214-218). Queues are 64 (prefilter) and 256 (FE), so normally nothing is dropped. |
| `/hdl_graph_slam/odom2pub` (BE) | `geometry_msgs/TransformStamped` | `map` → `odom` | **T_map←odom** = (optimised pose of the newest keyframe) · (its FE odometry)⁻¹ (`hdl_graph_slam_nodelet.cpp:590`, `matrix2transform` in `include/hdl_graph_slam/ros_utils.hpp:23`). It maps odom-frame coordinates into the map frame; `map2odom_publisher.py` rebroadcasts it as TF `map`→`odom`. | Stamp of the **newest keyframe in the graph** at optimisation time (:604), not the publication time. Several messages may share a stamp (for example after a floor-only update), and the last one received wins. | At most one per `graph_update_interval` = 3 s wall time, and only when the graph changed (:546-563). Published only while it has a subscriber (:603): `map2odom_publisher` and `rosbag record` subscribe. |
| `/hdl_graph_slam/dump` (service) | `hdl_graph_slam/DumpGraph`: `string destination` → `bool success` | – | See §4. | – | Called once, after playback. |

Other topics, which are not recorded:

- `/filtered_points`, `/scan_matching_odometry/{transform,status,read_until}`, `/aligned_points`.
- `/floor_detection/{floor_coeffs,read_until,floor_points,floor_filtered_points}`.
- `/hdl_graph_slam/{markers,map_points (latched),read_until}`.
- TF `odom`→`base_link` and `odom`→`keyframe` (FE, scan stamps), TF `map`→`odom` (10 Hz, `rospy.Time.now()`), and `base_link`→`velodyne` (tf1 static publisher, every 10 ms).

### Frames and static transforms in `hdl_graph_slam_400.launch`

- `lidar2base_publisher` (launch:19): `static_transform_publisher 0 0 0 0 0 0 base_link velodyne 10`. This is the identity, so **`base_link` is the LiDAR frame**.
- The prefilter transforms every cloud into `base_link` (launch:28), so FE, floor detection and BE all work in `base_link`. `hdl_D.launch` keeps this transform and only takes the child frame from `lidar_frame_id` (default `velodyne`). That value **must equal the `frame_id` of the bag's point clouds**: otherwise `lookupTransform` (`prefiltering_nodelet.cpp:122`) throws and the nodelet manager stops.
- `odom`: FE frame, equal to `base_link` at the first scan.
- `map`: BE frame. The first node is initialised at T_map←odom = I (`hdl_graph_slam_nodelet.cpp:205`). However, with floor detection on, a **fixed** plane vertex (0, 0, 1, 0) is added (:490-493), so the optimised map frame has **z = 0 on the floor** and keyframe poses sit at z ≈ `sensor_height`. T_map←odom therefore contains a vertical offset of ≈ h, plus any initial roll and pitch. An SE(3) or Sim(3) Umeyama alignment absorbs this. Without floor detection (Section 5.2 ablation) this offset does not exist.

### LiDAR-frame pose for each output (brief Section 10)

Here T_base←lidar = I.

1. **FE:** T_odom←lidar(t) = pose of `/odom` at stamp t.
2. **Online full:** T_map←lidar(t) = T_map←odom(s) · T_odom←lidar(t), where s is the stamp of the newest `odom2pub` message with s ≤ t (header stamps). **Note:** a correction stamped s is published up to about 3 s (wall) after s, so header-stamp association uses a correction that the online system did not yet have at t. The alternative is association by bag receive time; see Open questions.
3. **Final:** T_map←lidar(t_k) = `estimate` of keyframe k in the dump, at keyframe stamp t_k (µs-truncated scan stamp).

If a dataset ever needs a non-identity `lidar2base_publisher`, use T_x←lidar = T_x←base_link · T_base_link←lidar for every output above. `sensor_height` then refers to `base_link`.

## 4. Keyframe dump

Call it from any container on the ROS master, once playback has ended and the graph has stopped changing:

```
rosservice call /hdl_graph_slam/dump "destination: '/runs/<run_id>/hdl_dump'"
```

- `destination` is a path **inside the hdl container**, so use a bind-mounted directory. The parent must exist: the node calls `boost::filesystem::create_directory` (one level, `hdl_graph_slam_nodelet.cpp:946-948`). **Do not pass an empty string:** the generated timestamp name is never assigned (:937-944), so the dump fails.
- Only keyframes already inserted into the graph are written (`keyframes`, :955-960). At most 10 queued keyframes are inserted per 3 s optimisation cycle (`max_keyframes_per_update`). Before dumping, wait until `odom2pub` has been silent for ≥ 2 cycles (≥ 6 s wall); the protocol's 10 s idle wait covers this. The dump does not run a final optimisation; it writes the estimates of the last cycle.

Layout of `<destination>/`:

- `graph.g2o`: g2o graph (`GraphSLAM::save`).
- `special_nodes.csv`: lines `anchor_node <id>`, `anchor_edge <id>`, `floor_node <id>` (−1 if absent).
- `zero_utm`: only written with GPS, so absent here.
- `000000/`, `000001/`, …: one directory per keyframe, in insertion (chronological) order (`%06d`). Each holds:
  - `data`: text, written by `KeyFrame::save` (`src/hdl_graph_slam/keyframe.cpp:21-58`):
    ```
    stamp <sec> <nsec>            # keyframe stamp = scan stamp (µs-truncated), integers, exact
    estimate                      # 4x4 T_map<-base_link after the last optimisation (4 rows, space separated)
    <r00 r01 r02 tx> ... (4 lines)
    odom                          # 4x4 T_odom<-base_link from FE at the keyframe
    <4 lines>
    accum_distance <m>            # BE travelled distance (sum of keyframe dx)
    floor_coeffs <a b c d>        # only if a floor plane was associated with this keyframe
    id <g2o vertex id>
    ```
    Upstream writes the matrices with the stream default of **6 significant digits** (1 mm above 100 m and 1 cm above 1 km). Patch `env/patches/hdl/0001-keyframe-dump-full-precision.patch` (logging only) sets `max_digits10`. The stamps are integers and are already exact, so **no patch is needed for stamps**.
  - `cloud.pcd`: binary PCD of the filtered keyframe cloud in `base_link`, about 16 B/point. **Disk:** this is roughly 0.2–0.4 MB per keyframe, so about 1000 keyframes for `cf_3` at 2 m spacing gives a few hundred MB per dump, and tens of GB over all runs. With ~14 GB free, the run script must delete or compress `*/cloud.pcd` after extracting the trajectory (see Open questions).

## 5. Open questions (hdl)

1. **`tilt_deg` as a tuning parameter.** It is the LiDAR's mounting pitch, a sensor setting like `sensor_height`. The submitted P2/P3 values (30°, 15°) on a level-mounted sensor make the 10° verticality check (`floor_normal_thresh`, code default 10, :61) reject almost every plane, which in practice disables floor edges. That is already what the Section 5.2 ablation tests. Proposal: treat it as per-dataset (0° unless the calibration shows a pitch) and do not tune it. Nahuel to confirm, and to provide the CitrusFarm and Pullally pitch (from `05-baselink-lidar-result.txt` and the Pullally authors).
2. **`lidar_frame_id`.** The bag `frame_id` of `/velodyne_points` for CitrusFarm and Pullally must come from the inventory (`datasets/inventory.md`). If it is not `velodyne`, pass `lidar_frame_id:=<frame>`.
3. **`odom2pub` association** for the online full output: by header stamp (as in brief Section 10, which looks up to ~3 s ahead in wall time) or by bag receive time (what the online system actually had)? Both are recorded in the bag, so the evaluation can report both.
4. **Dump disk use:** may the run script keep only `data`, `graph.g2o` and `special_nodes.csv` and delete `cloud.pcd`? A logging-only patch to skip the clouds is possible but not written.
5. **Brief "Affects" for rows c and d:** the brief proposes the same value in both nodelets. The draft tunes FE and BE separately. Confirm.
6. **Brief arg names:** the brief writes `enable_imu_acc`/`enable_imu_ori` (launch args). The ROS parameters are `enable_imu_acceleration`/`enable_imu_orientation`. `rosparam dump` will show the latter.

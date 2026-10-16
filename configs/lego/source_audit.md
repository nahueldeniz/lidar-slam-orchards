# LeGO-LOAM source audit

**Pinned source:** `RobustFieldAutonomyLab/LeGO-LOAM`, branch `master`, commit `896a7a95a8bc510b76819d4cc48707e344bad621` (2020-07-02). Local clone: `env/src/lego/LeGO-LOAM`.

**Dependency stated in the README:** GTSAM 4.0.0-alpha2 (commit `c82fe1f`, 2017-08-05).

All file:line references are to the **unpatched** upstream files; paths are relative to `LeGO-LOAM/`. "FE" means `featureAssociation` (odometry). "BE" means `mapOptmization` (mapping). `imageProjection` is the shared pre-processing stage that feeds both.

## 1. Brief table 5.3: verification

| Row | Constant | Definition | Default (unit) | Where it is used (traced) | Module in code | Brief | Status |
|---|---|---|---|---|---|---|---|
| a | `mappingProcessInterval` | `include/utility.h:105` (`double`) | 0.3 (s) | `mapOptmization.cpp:1499`: mapping runs only if laser-odometry stamps advanced by at least this much | BE | 0.3, BE | ✔ |
| b | `segmentTheta` | `include/utility.h:113` (`float`, `60.0/180.0*M_PI`) | 60° = 1.0472 rad | `imageProjection.cpp:423` (cluster-merge test); its output feeds FE, and through FE, BE | pre-processing → FE and BE | 60, FE+BE | ✔ |
| c | `edgeFeatureNum` | `include/utility.h:120` | 2 (count) | **never referenced**; `featureAssociation.cpp:709` hard-codes `largestPickedNum <= 2` | none | 2, FE+BE | ✘ **dead constant** |
| d | `edgeThreshold` | `include/utility.h:123` (`float`) | 0.1 (code curvature, m²) | `featureAssociation.cpp:705` | FE (BE only through the features it receives) | 0.1, FE+BE | ⚠ used in FE code only |
| e | `surfFeatureNum` | `include/utility.h:121` | 4 (count) | **never referenced**; `featureAssociation.cpp:747` hard-codes `smallestPickedNum >= 4` | none | 4, FE+BE | ✘ **dead constant** |
| f | `surfThreshold` | `include/utility.h:124` (`float`) | 0.1 (code curvature, m²) | `featureAssociation.cpp:740` | FE (BE only through the features it receives) | 0.1, FE+BE | ⚠ used in FE code only |
| g | `nearestFeatureSearchSqDist` | `include/utility.h:125` (`float`) | 25 (m²; a gate of 5 m) | `featureAssociation.cpp:1057, 1061, 1168, 1172` (edge and planar correspondence gates). Mapping uses its own literal `1.0` (`mapOptmization.cpp:1101, 1183`). | FE | 25, FE+BE | ⚠ used in FE code only |

### Notes on the table

- **"Affects" column.** Only (a) is read by BE code. (b), (d), (f) and (g) are read only by pre-processing or FE code. They still change BE outputs, because `mapOptmization` consumes the FE features and odometry. Suggested wording for T3: "used in: FE; influences: FE and BE outputs".
- **(c) and (e) are dead constants.** `sectionsTotal` (`utility.h:122`, 6) is also never referenced: `featureAssociation.cpp:691-694` hard-codes 6 sub-images. The submitted P1–P3 used 2 and 4 throughout, so the submitted results are unaffected. These constants cannot be tuned without editing algorithm code, which is not allowed.
- **Curvature unit.** The code computes `(Σ_{±5} r_j − 10·r_i)²` (`featureAssociation.cpp:626-633`). That is a squared range difference in m², not the normalised c of the paper's eq. (1).
- **Is `nearestFeatureSearchSqDist` the submitted "nearest-feature search distance"?** It is the only `utility.h` constant that is a feature-search distance, and its default (25) equals the submitted P2/P3 value. The other distance constants have different defaults and act on keyframes, not features:
  - `surroundingKeyframeSearchRadius = 50` (`:129`);
  - `historyKeyframeSearchRadius = 7` (`:132`, loop closure only).

  The mapping is therefore consistent, but it cannot be proven without the 2024 files. The value is squared (m²): P1 = 10 means a gate of 3.16 m.
- **`loopClosureEnableFlag`.** Defined at `include/utility.h:104`, default **`false`** ✔. With `false`:
  - `loopClosureThread` returns at once (`mapOptmization.cpp:804`);
  - the surrounding map is built by a 50 m radius search (`:1006`), not from the recent-keyframe queue (`:961`);
  - iSAM2 still receives odometry factors between keyframes, but no loop factors.

### Scan-period constants

| Constant | Where | Use | Active in LiDAR-only mode? |
|---|---|---|---|
| `scanPeriod = 0.1` | `include/utility.h:107` (`float`) | `featureAssociation.cpp:523`: encodes the point's sweep fraction, `intensity = ring + scanPeriod·relTime` | **yes** |
| | | `featureAssociation.cpp:415` (IMU integration, called only from `imuHandler`) | no |
| | | `featureAssociation.cpp:526` (IMU de-skew, inside `if (imuPointerLast >= 0)` at `:525`) | no |
| | | `featureAssociation.cpp:1660-1662` (IMU velocity initial guess, only if IMU velocity ≠ 0) | no |
| | | `mapOptmization.cpp:468, 474, 479, 481` (IMU roll/pitch blending, inside `if (imuPointerLast >= 0)` at `:465`) | no |
| hidden literal `10` | `featureAssociation.cpp:862` (`TransformToStart`), `:887` (`TransformToEnd`) | `s = 10 * (intensity - int(intensity))`, i.e. 1/0.1 s | **yes** |

**Consequence.** Without an IMU, s = 10 · scanPeriod · relTime. With the upstream 0.1 s, s = relTime ∈ [0, 1], the fraction of the sweep. `transformCur` is the motion between consecutive sweeps whatever their duration, so this de-skew does not depend on the real scan rate.

Setting `scanPeriod = 0.25` for a 4 Hz dataset, as brief Section 5.0 asks, would give s ∈ [0, 2.5]. The motion interpolation would then over-extrapolate by a factor of 2.5. **Discrepancy with the brief:** for LeGO-LOAM in LiDAR-only mode, the scan period should stay 0.1 on every dataset (see the open questions). Patch 0002 prints a `ROS_WARN` if `scanPeriod ≠ 0.1`.

**Other rate and sensor assumptions:**

- `Horizon_SCAN = 1800` and `ang_res_x = 0.2°` (`utility.h:64-65`) assume 0.2° horizontal resolution, i.e. a VLP-16 at 10 Hz.
- Synchronisation tolerances of 0.05 s (`featureAssociation.cpp:1821-1822`) and 0.005 s (`mapOptmization.cpp:1489-1491`) hold at any rate.

## 2. Topics

### Subscriptions

| Node | Topic | Type | Queue | Where |
|---|---|---|---|---|
| imageProjection | `/velodyne_points` (`pointCloudTopic`, `utility.h:53`) | `sensor_msgs/PointCloud2` | 1 | `imageProjection.cpp:89` |
| featureAssociation | `/segmented_cloud`, `/segmented_cloud_info`, `/outlier_cloud` | internal | 1 | `featureAssociation.cpp:190-192` |
| featureAssociation | **`/imu/data`** (`imuTopic`, `utility.h:54`) | `sensor_msgs/Imu` | 50 | `featureAssociation.cpp:193` |
| mapOptmization | `/laser_cloud_corner_last`, `/laser_cloud_surf_last`, `/outlier_cloud_last`, `/laser_odom_to_init` | internal | 2 or 5 | `mapOptmization.cpp:238-241` |
| mapOptmization | **`/imu/data`** | `sensor_msgs/Imu` | 50 | `mapOptmization.cpp:242` |
| transformFusion | `/laser_odom_to_init`, `/aft_mapped_to_init` | internal | 5 | `transformFusion.cpp:69-70` |

There are no GNSS, wheel-odometry or TF-lookup inputs.

**Input cloud requirements:**

- `useCloudRing = true` (`utility.h:60`), so the cloud must carry a `ring` field (`PointXYZIR`).
- `is_dense` must be true. Otherwise the node logs `ROS_ERROR` and calls `ros::shutdown()` (`imageProjection.cpp:173-177`).

### LiDAR only

What is needed:

- No publisher on `/imu/data`. The run protocol plays only the LiDAR topic.
- Configuration D also remaps both IMU subscriptions to `/slambench/imu_disabled` (`configs/lego/D/run.launch`), so bag IMU data can never arrive. The run check should confirm that neither `/imu/data` nor `/slambench/imu_disabled` has a publisher.

Behaviour with no IMU:

- `imuPointerLast` stays at −1.
- **featureAssociation:**
  - the IMU de-skew block is skipped (`:525`);
  - all `imu*Start`, `imu*Cur` and `imu*Last` values stay 0, so `updateInitialGuess` (`:1639-1664`) adds nothing;
  - `PluginIMURotation` with all-zero IMU angles is the identity;
  - `checkSystemInitialization` adds a zero roll and pitch (`:1633-1634`).
- **mapOptmization:** the IMU roll/pitch blending is skipped (`:465`).
- **imageProjection** has no IMU input.
- Nothing waits for the IMU. The result is pure LiDAR odometry and mapping.

### Outputs (brief Section 6.1)

All three are verified:

| Topic | Type | header.frame_id | child_frame_id | Node | Rate | Stamp |
|---|---|---|---|---|---|---|
| `/laser_odom_to_init` | `nav_msgs/Odometry` | `/camera_init` | `/laser_odom` | featureAssociation (`:203` advertise, `:305-306` frames, `:1727-1744` publish) | every scan except the first, which initialises (`:1846`): LiDAR rate (10 Hz) | **identical** to the input scan stamp (`:1730`; the header is copied through `/segmented_cloud`) |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | `/camera_init` | `/aft_mapped` | mapOptmization (`:236`, `:259-260`, `publishTF` `:654-679`) | once per mapping step. Features reach mapping on every 2nd scan (`featureAssociation.cpp:1792`), and mapping runs if ≥ 0.3 s have passed: **2.5 Hz at 10 Hz input** (2 Hz at 4 Hz) | `ros::Time().fromSec(timeLaserOdometry)` (`:659`): the input stamp **±119 ns at most** (double round trip, simulated) |
| `/integrated_to_init` | `nav_msgs/Odometry` | `/camera_init` | `/camera` | transformFusion (`:68` advertise, `:71-72` frames, `:202-210` publish) | one per `/laser_odom_to_init`: LiDAR rate | **identical** to `/laser_odom_to_init`, hence to the input |

Notes:

- **Twist fields of `/aft_mapped_to_init`.** `twist.twist.angular/linear` carry the odometry pose before mapping (`transformBefMapped`, `mapOptmization.cpp:667-672`), not a velocity. Do not interpret them as a twist.
- **Before the first mapping step,** `/integrated_to_init` equals the laser odometry, because the mapping correction is zero.
- **Leading slashes.** Frame ids carry a leading `/`, which should be stripped in the evaluation. The nodes also broadcast TF: `/camera_init→/laser_odom`, `/camera_init→/aft_mapped`, `/camera_init→/camera`.
- **Stamp matching for the frame logger (brief Section 8).** Match output stamps to input stamps with a tolerance (for example 1 µs), not by equality. Simulating 200,000 random stamps showed that 99.6 % change by up to 119 ns through `fromSec(toSec())`.

## 3. Frames and the conversion to the LiDAR pose

**Axis convention (inherited from LOAM).** The code works in "camera" axes:

- x_c = y_L (left), y_c = z_L (up), z_c = x_L (forward);
- this is applied at `featureAssociation.cpp:500-502` (`point.x = y; point.y = z; point.z = x`) and `:1751-1753`.

The frames are:

- `camera_init` is that axis frame at the first scan;
- `laser_odom`, `aft_mapped` and `camera` are the same axes at the current scan.

**Published orientation.** The code publishes q = (−g.y, −g.z, g.x, g.w), where g = `createQuaternionMsgFromRollPitchYaw(rz, −rx, −ry)` (`featureAssociation.cpp:1728-1735`; the same pattern is at `mapOptmization.cpp:656-664` and `transformFusion.cpp:199-206`).

- Verified numerically (1000 random angles, maximum error 6e-16): R(q) = R_y(ry)·R_x(rx)·R_z(rz). This is exactly the rotation LOAM applies to map a point from the current frame into `camera_init` (`pointAssociateToMap`, `mapOptmization.cpp:513-527`).
- Each output is therefore T_ci←c(t) = [R(q_t), p_t].

**Static TFs in `run.launch:10-11`.** The arguments are `x y z yaw pitch roll parent child period_ms`.

- `map → camera_init`: yaw 1.570795, pitch 0, roll 1.570795. So R_map←ci = R_z(π/2)·R_x(π/2) = **Pᵀ** = [[0,0,1],[1,0,0],[0,1,0]], i.e. x_map = z_ci, y_map = x_ci, z_map = y_ci. As a quaternion (x, y, z, w) = (0.5, 0.5, 0.5, 0.5).
- `camera → base_link`: yaw −1.570795, pitch −1.570795, roll 0. So R_c←bl = R_z(−π/2)·R_y(−π/2) = **P** = [[0,1,0],[0,0,1],[1,0,0]], with zero translation.
- The product of the two rotations is the identity. 1.570795 differs from π/2 by 1.3e-6 rad, which is negligible; use the exact P.
- **`base_link` here is the LiDAR (velodyne) frame**, not the robot base. `imageProjection` publishes the raw LiDAR clouds with `frame_id = "base_link"` (`imageProjection.cpp:472-504`), and the camera → base_link translation is zero.

**Explicit conversion (brief Section 10).** For every LeGO-LOAM output pose (p_t, q_t):

    T_W←L(t) = T_map←ci · T_ci←c(t) · T_c←bl
    R_W←L(t) = Pᵀ · R(q_t) · P
    p_W←L(t) = Pᵀ · p_t      i.e. (x_W, y_W, z_W) = (p_t.z, p_t.x, p_t.y)
    Pᵀ = [[0,0,1],[1,0,0],[0,1,0]]

- W = `map` = the LiDAR frame at the first processed scan (x forward, y left, z up).
- W is z-up only as far as the LiDAR was level at the start. There is no IMU initialisation: `transformSum` starts at zero (`featureAssociation.cpp:288`).
- The same conversion applies to all three outputs.
- Suggested numerical check (brief Section 10): roll and pitch of R_W←L close to 0 in the first seconds on flat ground, and an XY shape that matches the ground truth after alignment.

## 4. Patches and their verification

Patch files are in `env/patches/lego/`. They apply in order on `896a7a9` with `git apply`; the `# reason:` first line is skipped as preamble.

- **0001-buildtime-constant-overrides.patch** (build configuration only):
  - Wraps `mappingProcessInterval`, `scanPeriod`, `segmentTheta` (given in degrees), `edgeThreshold`, `surfThreshold` and `nearestFeatureSearchSqDist` in `#ifdef <NAME>_OVERRIDE … #else <upstream line verbatim> #endif`.
  - Adds the CMake cache variable `SLAMBENCH_DEFS`: `catkin_make -DSLAMBENCH_DEFS="SEGMENT_THETA_DEG_OVERRIDE=30 …"`. Unknown names stop CMake with an error.
  - Only insertions (46 lines); no upstream line is changed.
- **0002-log-effective-constants.patch** (logging only):
  - Adds `slambenchLogConstants()` to `utility.h` and one call after `ros::init` in each of the four nodes.
  - Each node prints three `[slambench-constants]` `ROS_INFO` lines.
  - It prints a `ROS_WARN` if `scanPeriod ≠ 0.1` (see Section 1).

Checks performed on 2026-10-12 (host Ubuntu 22.04, GCC 11.4; scripts in the session scratchpad):

1. **The patches apply cleanly** to a fresh clone at `896a7a9`. The result is byte-identical to the local clone (`env/src/lego/LeGO-LOAM`, where the patches are applied in the working tree).
2. **Without overrides, `utility.h` matches upstream token for token.** It was preprocessed with includes stripped and no `-D`: 0 differing lines against upstream.
3. **Values.** A compiled harness gives the upstream values without overrides (0.3, 0.1, 1.0472 rad, 0.1, 0.1, 25) and the overridden values with them (0.6, 0.25, 0.5236 rad, 0.2, 0.05, 10).
4. **Compilation.** All four nodes pass `g++ -fsyntax-only -Wformat=2` with and without overrides. This used stub ROS headers, host PCL 1.12 and the GTSAM alpha2 headers; there were no errors or format warnings.
5. **Log values.** The logging function was compiled and run against printing stubs. Its output is recorded as `EXPECT_*` in `configs/lego/D/constants.env`.

Not checked: a real Melodic build, because Docker builds were not allowed.

## 5. Build feasibility on ROS Melodic / Ubuntu 18.04

| Item | Finding |
|---|---|
| C++ standard and flags | `CMakeLists.txt:4`: `${CMAKE_CXX_FLAGS} -std=c++11 -O3`, which appends. Fine with GCC 7.5. |
| OpenCV | `#include <opencv/cv.h>` (`utility.h:13`) exists in OpenCV 3.2 (Melodic). It would fail with OpenCV 4 (Noetic). Only `cv::Mat` is used. |
| PCL / Eigen / Boost | PCL 1.8.1, Eigen 3.3.4 and Boost 1.65 are fine. GTSAM 4.0 needs Boost ≥ 1.43. |
| GTSAM | The README names 4.0.0-alpha2 (CMake ≥ 2.6). It has no `-march=native` option or flag. It bundles Eigen 3.2.10 and uses it by default (`GTSAM_USE_SYSTEM_EIGEN=OFF`). LeGO-LOAM's include order puts PCL's `/usr/include/eigen3` (3.3.4) before GTSAM's bundled Eigen, so `mapOptmization` compiles GTSAM headers against 3.3.4 while `libgtsam` uses 3.2.10. This is a latent ODR risk, but it is what the README build gives (see the open questions). TBB and MKL are pinned OFF. |
| Message generation | `mapOptmization` and `transformFusion` include `cloud_msgs/cloud_info.h` without depending on `cloud_msgs_gencpp`, hence the README's `catkin_make -j1`. The Dockerfile uses `-j1`. |
| package.xml | Declares `ros_bridge`, `sensors_msgs` and `gtsam` as ROS dependencies, which do not exist. `catkin_make` ignores them, but `rosdep install` would fail, so rosdep is not used. |
| Runtime | Frame ids with a leading `/` cause tf2 warnings only; there are no TF lookups. |
| **Verdict** | **Expected to build on Melodic with no build-fix patches** (the README says it was tested on Melodic). Host probes compile all four nodes. Not confirmed by a real image build. |

## 6. Open questions

1. **Scan period (brief Section 5.0).** For LeGO-LOAM in LiDAR-only mode, keep `scanPeriod = 0.1` for all datasets. A measured 0.25 s would break the de-skew interpolation because of the hard-coded `10` (Section 1). Nahuel to confirm.
2. **Pullally horizontal resolution.** If the Pullally scans are 0.4° (900 columns), the README's "New Lidar" section says `Horizon_SCAN`/`ang_res_x` (`utility.h:64-65`) must match. These are sensor settings that patch 0001 does not expose. A 0003 patch with the same `#ifdef` pattern would be needed. Decide after the inventory.
3. **Input cloud format.** Check in the inventory that every dataset's `/velodyne_points` has a `ring` field and `is_dense = true`. Otherwise LeGO-LOAM shuts down at the first scan (`imageProjection.cpp:173-177`).
4. **GTSAM Eigen.** Use the bundled Eigen 3.2.10 (README default, chosen) or system Eigen 3.3.4 (`GTSAM_USE_SYSTEM_EIGEN=ON`), which removes the mixed-Eigen risk but deviates from the README and is untested for alpha2? This is a build arg in the Dockerfile.
5. **Rows c and e in table T3.** `edgeFeatureNum` and `surfFeatureNum` are dead constants. Report them as "not effective in the code" in T3, and in the response letter for the submitted P1–P3.
6. **Build type.** The brief asks for Release (`-O3 -DNDEBUG`). The README's `catkin_make` sets no build type (`-O3`, asserts on). This is recorded; no action is proposed.
7. **End of playback with sim time.** The `ros::Rate(200)` loops in featureAssociation and mapOptmization (`:1874`, `:1537`) sleep on `/clock`. When `rosbag play` stops publishing `/clock`, the last one or two scans may never be processed. The infrastructure agent should decide whether to keep `/clock` running for a few seconds after playback.

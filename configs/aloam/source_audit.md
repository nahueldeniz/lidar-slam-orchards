# A-LOAM source audit

**Pinned source:** `HKUST-Aerial-Robotics/A-LOAM`, branch `devel`, commit `e51f88c12b483ce42dfa219783712c7234dc8c08` (2019-03-28). `devel` exists and is the default and only branch. Local clone: `env/src/aloam/A-LOAM`.

**Ceres version:** the README names none ("Follow Ceres Installation"). The repository's `docker/Dockerfile` pins `CERES_VERSION="1.12.0"` (on `ros:kinetic-perception`, with PCL 1.8.0 built from source).

All file:line references are to the **unpatched** upstream files. "FE" means `alaserOdometry`. "BE" means `alaserMapping`. `ascanRegistration` is the shared pre-processing stage.

## 1. Brief table 5.4: verification

| Row | Parameter | Definition | Default (unit) | Where it is used (traced) | Module | Brief | Status |
|---|---|---|---|---|---|---|---|
| a | `DISTANCE_SQ_THRESHOLD` | `src/laserOdometry.cpp:65` (`constexpr double`) | 25 (m²; a gate of 5 m) | `laserOdometry.cpp:305, 310` (edge), `:393, 399` (plane): gate for the closest point and initial bound for the support points | FE | 25, FE | ✔ |
| b | `mapping_line_resolution` | ROS parameter, `laserMapping.cpp:902` (`float`, code default 0.4); VLP-16 launch line 12 = **0.2** | 0.2 (m) | voxel leaf of the corner map and the current corner features: `:905` (set), `:543`, `:793` (applied) | BE | 0.2 launch / 0.4 code, BE | ✔ |
| c | `mapping_plane_resolution` | ROS parameter, `laserMapping.cpp:903` (code default 0.8); launch line 13 = **0.4** | 0.4 (m) | `:906` (set), `:548`, `:798` (applied) | BE | 0.4 launch / 0.8 code, BE | ✔ |
| d | `NEARBY_SCAN` | `src/laserOdometry.cpp:66` (`constexpr double`) | 2.5 (scan lines) | `:319, 345` (edge), `:405, 433` (plane), compared with integer ring ids, so it acts as ±2 rings | FE | 2.5, FE | ✔ (note the integer behaviour) |
| e | Huber loss, odometry | `src/laserOdometry.cpp:284`, literal `new ceres::HuberLoss(0.1)` | 0.1 (m) | the odometry problem, both inner iterations (`:278-284`) | FE | 0.1, FE | ✔ |
| f | Huber loss, mapping | `src/laserMapping.cpp:565`, literal `new ceres::HuberLoss(0.1)` | 0.1 (m) | the scan-to-map problem, both inner iterations (`:562-565`) | BE | 0.1, BE (proposed) | ✔ (the only robust loss in laserMapping, so the proposed mapping is consistent) |

**Huber occurrences.** A search for `HuberLoss` and `LossFunction` in `src/` and `include/` finds exactly:

- two active lines: `laserOdometry.cpp:284` and `laserMapping.cpp:565`;
- two commented-out null-loss alternatives: `:283` and `:564`.

`lidarFactor.hpp` and `kittiHelper.cpp` contain none. Both active occurrences are handled identically by patch 0001: an override hook with default 0.1 and a named constant (`ODOM_HUBER_DELTA`, `MAPPING_HUBER_DELTA`). The Ceres scale a is a residual magnitude (inlier if the squared residual s ≤ a²), so it is in metres here.

**Fixed values from the VLP-16 launch.** All match the brief ✔:

| Parameter | Launch line | Value | Code default |
|---|---|---|---|
| `scan_line` | 3 | 16 | 16 (`scanRegistration.cpp:466`) |
| `mapping_skip_frame` | 6 | 1 | 2 (`laserOdometry.cpp:191`) |
| `minimum_range` | 9 | 0.3 | 0.1 (`scanRegistration.cpp:468`) |
| `mapping_line_resolution` | 12 | 0.2 | 0.4 |
| `mapping_plane_resolution` | 13 | 0.4 | 0.8 |

All are global ROS parameters, because the nodes read them through a public `NodeHandle`.

### Scan-period constants

| Constant | Where | Use | Effect in D |
|---|---|---|---|
| `scanPeriod = 0.1` | `scanRegistration.cpp:60` (`const double`) | `:239`: `intensity = scanID + scanPeriod·relTime` (sweep fraction encoded into the intensity) | none; only decoded under `DISTORTION` |
| `SCAN_PERIOD = 0.1` | `laserOdometry.cpp:64` (`constexpr double`) | `:116` (`TransformToStart`), `:377`, `:475` (factor interpolation), all inside `if (DISTORTION)` | **none**: `#define DISTORTION 0` (`:59`) makes these branches dead and s = 1.0 for every point |
| `systemDelay = 0` | `scanRegistration.cpp:62` | number of initial scans skipped | none |
| `10 / skipFrameNum` | `laserOdometry.cpp:193` | `printf` only | none |

Consequences:

- **With `DISTORTION 0` (upstream), the scan period has no effect on any output.** The brief's rule to build with the measured period is harmless for A-LOAM but does nothing. Patch 0002 prints a `ROS_WARN` if `SCAN_PERIOD ≠ 0.1` while `DISTORTION` is 0.
- **With `DISTORTION 1`,** s = (scanPeriod · relTime) / SCAN_PERIOD = relTime whenever the two constants are equal. Patch 0001 sets both from one macro, so they are always equal. The actual value would still not matter.
- **No other hidden scan-period literal exists**, unlike LeGO-LOAM's `10`.

## 2. Topics

### Subscriptions

| Node | Topic | Type | Queue | Where |
|---|---|---|---|---|
| ascanRegistration | `/velodyne_points` | `sensor_msgs/PointCloud2` | 100 | `scanRegistration.cpp:478` |
| alaserOdometry | `/laser_cloud_sharp`, `/laser_cloud_less_sharp`, `/laser_cloud_flat`, `/laser_cloud_less_flat`, `/velodyne_cloud_2` | internal | 100 | `laserOdometry.cpp:195-203` |
| alaserMapping | `/laser_cloud_corner_last`, `/laser_cloud_surf_last`, `/laser_odom_to_init`, `/velodyne_cloud_3` | internal | 100 | `laserMapping.cpp:908-914` |

**Input cloud requirements:**

- Only x, y and z are read (`pcl::PointXYZ`, `scanRegistration.cpp:132-133`).
- NaNs are removed (`:136`).
- The ring id is computed from the elevation angle (`:166-177`), so no `ring` field and no `is_dense` flag are needed.

**LiDAR only by construction.** No node subscribes to an IMU, GNSS or odometry topic. `sensor_msgs/Imu.h` is included at `scanRegistration.cpp:51` and `laserOdometry.cpp:47` but never used. Nothing is needed beyond playing only the LiDAR topic.

### Outputs (brief Section 6.1)

All three are verified:

| Topic | Type | header.frame_id | child_frame_id | Node | Rate | Stamp |
|---|---|---|---|---|---|---|
| `/laser_odom_to_init` | `nav_msgs/Odometry` | `/camera_init` | `/laser_odom` | alaserOdometry (`:211` advertise, `:511-522` publish) | every scan, including the first, which has the identity pose (`:267-271`): LiDAR rate | `ros::Time().fromSec(timeSurfPointsLessFlat)` (`:514`): the input stamp **±119 ns at most** |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | `/camera_init` | `/aft_mapped` | alaserMapping thread `process()` (`:922` advertise, `:854-865` publish) | once per processed mapping frame. Every scan is offered (`mapping_skip_frame = 1`), but if mapping lags, all queued frames except the newest are **dropped** (`:299-303`, "drop lidar frame in mapping for real time performance"). So the rate is ≤ the LiDAR rate and depends on CPU load. | `fromSec(timeLaserOdometry)` (`:857`): the input stamp ±119 ns |
| `/aft_mapped_to_init_high_frec` | `nav_msgs/Odometry` | `/camera_init` | `/aft_mapped` | alaserMapping callback `laserOdometryHandler` (`:924` advertise, `:197-229`) | one per `/laser_odom_to_init`: LiDAR rate | copied from `/laser_odom_to_init` (`:220`) |

Notes:

- **What `_high_frec` is.** It is the latest map correction T_wmap←wodom composed with the current odometry (`:214-215`). Before the first mapping step the correction is the identity, so it equals the odometry.
- **Unlocked shared state.** The correction is written by the mapping thread (`transformUpdate`, `:148-152`, called at `:734`) and read in the callback thread without a lock. This is upstream behaviour and was not changed.
- **TF.** Only `/camera_init → /aft_mapped` is broadcast (`laserMapping.cpp:886`). alaserOdometry broadcasts no TF. Frame ids carry a leading `/`, which the evaluation should strip.
- **Stamp matching for the frame logger.** As for LeGO-LOAM, match stamps with a tolerance (for example 1 µs), because `fromSec(toSec())` changes 99.6 % of stamps by up to 119 ns (simulated).
- **Frame drops make `/aft_mapped_to_init` CPU-dependent,** which adds to run-to-run spread. Each drop prints a line in the console log, so the drops can be counted per run.

## 3. Frames and the conversion to the LiDAR pose

**A-LOAM removed LOAM's axis swap.**

- `scanRegistration.cpp:162-164` copies x, y and z unchanged.
- All poses are Eigen quaternions and vectors of the raw LiDAR coordinates: `q_w_curr`/`t_w_curr`, `laserOdometry.cpp:93-94` and `laserMapping.cpp:144`.

The frames are therefore:

- `camera_init` is the **LiDAR frame at the first scan** (the name is historical);
- `laser_odom` and `aft_mapped` are the **current LiDAR frame**.

The launch file publishes no static TF.

**Conversion (brief Section 10):** T_W←L(t) = [R(q_t), p_t] directly; the extra rotation is the identity.

- W is the LiDAR frame at the first scan (x forward, y left, z up for an upright VLP-16).
- W is z-up only as far as the LiDAR was level at the start.
- Suggested check: roll and pitch close to 0 at the start on flat ground, and an XY shape that matches the ground truth after alignment.

## 4. Patches and their verification

Patch files are in `env/patches/aloam/`. They apply in order on `e51f88c` with `git apply`; the `# reason:` first line is skipped.

- **0001-buildtime-constant-overrides.patch** (build configuration only):
  - Wraps `scanPeriod`, `SCAN_PERIOD`, `DISTANCE_SQ_THRESHOLD` and `NEARBY_SCAN` in `#ifdef <NAME>_OVERRIDE … #else <upstream line verbatim> #endif`. One macro, `SCAN_PERIOD_OVERRIDE`, sets both scan-period constants.
  - Adds `ODOM_HUBER_DELTA` and `MAPPING_HUBER_DELTA` (default 0.1) and uses them in place of the two literals.
  - Adds the CMake cache variable `SLAMBENCH_DEFS`. This is needed because `CMakeLists.txt:5` overwrites `CMAKE_CXX_FLAGS`, so `-DCMAKE_CXX_FLAGS=…` would be silently ignored. Unknown names stop CMake with an error.
- **0002-log-effective-constants.patch** (logging only):
  - Each node prints one `[slambench-constants]` `ROS_INFO` line with its compile-time constants and ROS parameters.
  - alaserOdometry also prints `DISTORTION` and the warning described in Section 1.

Checks performed on 2026-10-12 (host Ubuntu 22.04, GCC 11.4, Eigen 3.3.4 headers, Ceres 1.12.0 headers):

1. **The patches apply cleanly** to a fresh clone at `e51f88c`. The result is byte-identical to the local clone, where the patches are applied in the working tree.
2. **Preprocessed sources without overrides, against upstream:**
   - `scanRegistration.cpp` is identical;
   - `laserOdometry.cpp` and `laserMapping.cpp` differ only by the added `constexpr double …_HUBER_DELTA = 0.1;` and `HuberLoss(<name>)` in place of `HuberLoss(0.1)`.
3. **Values.** A compiled harness gives the upstream values without overrides (0.1, 0.1, 25, 2.5, 0.1, 0.1) and the overridden values with them.
4. **Compilation.** All three nodes pass `g++ -fsyntax-only -Wformat=2` with and without overrides. This used stub ROS headers and host PCL 1.12; there were no errors or format warnings.
5. **Log lines.** The three `main()` functions were run against printing stubs. Their start-up lines are as expected (see the `EXPECT_*` values in `configs/aloam/D/constants.env`).
6. **Ceres probe.** Ceres 1.12.0 (`libceres` target) compiled against Eigen 3.3.4. A-LOAM's `lidarFactor.hpp` factors, with `EigenQuaternionParameterization` and `HuberLoss`, compiled, linked and evaluated.

Not checked: a real Melodic build.

## 5. Build feasibility on ROS Melodic / Ubuntu 18.04

| Item | Finding |
|---|---|
| C++ standard and flags | `CMakeLists.txt:4-6` forces `CMAKE_BUILD_TYPE Release`, `CMAKE_CXX_FLAGS "-std=c++11"` and `-O3 -Wall -g`. Fine with GCC 7.5. |
| OpenCV | `#include <opencv/cv.h>` (`scanRegistration.cpp:44`) and `kittiHelper` (cv_bridge) are fine with OpenCV 3.2. They would fail with OpenCV 4 (Noetic). |
| Ceres | Uses `ceres::LocalParameterization` and `EigenQuaternionParameterization` (`laserOdometry.cpp:285-286`, `laserMapping.cpp:566-567`). `EigenQuaternionParameterization` first appeared in **Ceres 1.12.0** (its version history), and `LocalParameterization` was removed in Ceres 2.2. The usable range is therefore 1.12.0 ≤ Ceres < 2.2. Ceres 1.12.0 adds no `-march=native` on Linux: its `GCC_VERSION` test is only set in the macOS branch, and the probe's compile flags confirm this. |
| PCL / Eigen | PCL 1.8.1 is fine. `catkin_package(DEPENDS EIGEN3 PCL)` (`CMakeLists.txt:35`) gives a catkin warning, because `find_package(Eigen3)` is commented out. This is a warning, not an error. |
| **Verdict** | **Expected to build on Melodic with no build-fix patches.** The README lists Ubuntu 18.04 and ROS Melodic. Not confirmed by a real image build. |

## 6. Open questions

1. **Ceres version.**
   - **1.12.0** is chosen: it is the repository docker pin and the minimum that has `EigenQuaternionParameterization`.
   - **1.13.0** is the Ubuntu 18.04 package and fixes Jet/Eigen 3.3 issues according to its release notes. The A-LOAM probe did not hit those issues.
   - **1.14.0** was the latest release when A-LOAM `e51f88c` was committed.

   Nahuel to confirm 1.12.0. It is a Docker build arg.
2. **Scan period.** It has no effect with `DISTORTION 0` (upstream). D keeps 0.1 s. Confirm that `DISTORTION` stays 0: it is not a tuning candidate in brief Section 5.4, and patch 0001 does not expose it.
3. **Frame dropping in alaserMapping** (`:299-303`) makes `/aft_mapped_to_init` depend on CPU load. Should the run manifest record the number of dropped frames, counted from the console log?
4. **Submitted resolutions.** The submitted P1–P3 reported line/plane resolutions of 0.4/0.8 m, which are the code defaults. The VLP-16 launch, the base of D, sets 0.2/0.4 m. It is unknown whether the 2024 runs loaded these launch parameters.
5. **End of playback with sim time.** The `ros::Rate(100)` loop of alaserOdometry (`:218`) sleeps on `/clock`, so the last scan may stay unprocessed when `/clock` stops. This is the same issue as LeGO-LOAM's and is for the infrastructure agent to decide.

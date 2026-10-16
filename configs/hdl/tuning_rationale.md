# DRAFT: hdl Graph SLAM tuning rationale (brief Section 5.0, steps 1–2)

**Status: DRAFT, not frozen.** Nahuel must approve these levels before any tuning run. The levels are listed in `tuning_levels.yaml`; configuration D is `D/hdl_D.launch`, and the source checks are in `D/README.md`.

Source: `koide3/hdl_graph_slam` @ `95b8dce` (master). The base launch is `hdl_graph_slam_400.launch`, the README's outdoor template.

## Sources used for justification

- **hdl README, "Parameter tuning guide":** "The mapping quality largely depends on the parameter setting. In particular, scan matching parameters have a big impact on the result." It adds: "Copy a template launch file (hdl_graph_slam_501.launch for indoor, hdl_graph_slam_400.launch for outdoor) and tweak parameters in the launch file."
- **hdl README, "Nodelets":** "The input point cloud is first downsampled by *prefiltering_nodelet* … *scan_matching_odometry_nodelet* estimates the sensor pose by iteratively applying a scan matching between consecutive frames … To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph."
- **The upstream launch templates** (`hdl_graph_slam{,_400,_501,_imu,_kitti}.launch`) serve as documented alternative values.
- **The code paths**, traced in `D/README.md` §1.
- **The method paper:** Koide, Miura and Menegatti (2019), *Int. J. Adv. Robot. Syst.* 16(2), cited in the README. It could not be retrieved automatically (the publisher blocks scripted downloads), so **no claim here relies on it**. Any paper citation in the manuscript should be checked against the PDF.

## Selection: 7 parameters from brief Table 5.1

Section 5.0 asks for 4–8 parameters from the candidate table that can change the evaluated outputs: `/odom` (FE), `odom2pub` and the dump (BE). Rows c and d of the table each name **two different ROS parameters**, one in FE and one in BE. They have different meanings, defaults and reach (D/README §1), so they are tuned separately. This answers Reviewer 1's "effect of each parameter" more precisely than a joint change would.

| id | Parameter | Module | D | Levels | Why it can change the output |
|---|---|---|---|---|---|
| a | `downsample_resolution` (prefiltering) | FE+BE | 0.1 m | 0.05, 0.3 | It sets the density of `/filtered_points`, the only input of scan matching (FE), floor detection and the keyframe clouds used for loop closure (BE). The README states that the input "is first downsampled", and that scan-matching parameters have the largest effect. The kitti template uses 0.25 m, so a coarser leaf is a documented option. |
| b | `radius_radius` (radius outlier removal) | FE+BE | 0.5 m | 0.25, 1.0 | It removes isolated returns before matching. In orchards, sparse canopy and branch returns are exactly the points it may remove. It acts on the same cloud as a. |
| c_fe | `keyframe_delta_trans` (FE) | FE+BE | 1.0 m | 0.5, 2.0 | FE registers each scan against the last FE keyframe and replaces that keyframe after this translation (`scan_matching_odometry_nodelet.cpp:241-252`). It trades drift (fewer keyframe changes) against overlap. The templates span 0.25 (501/imu) to 5.0 (kitti). |
| c_be | `keyframe_delta_trans` (BE) | BE | 2.0 m | 1.0, 4.0 | It sets the spacing of graph nodes: the number of odometry edges, floor edges and loop-closure candidates. The templates use 1.0 (501/imu), 2.0 (400) and 5.0 (kitti). It cannot change `/odom`. |
| d_fe | `keyframe_delta_angle` (FE) | FE+BE | 1.0 (half-angle) | 0.5, 2.0 | It is the rotation trigger for replacing the FE keyframe, relevant at headland turns. Because the code compares acos(q.w), D triggers only at 115°. 0.5 triggers at 57°, and 2.0 (the kitti/imu FE value) disables the rotation trigger. |
| d_be | `keyframe_delta_angle` (BE) | BE | 2.0 rad | 1.0, 4.0 | It is the rotation trigger for graph nodes (full angle). 1.0 rad adds nodes in turns; 4.0 disables the trigger (> π). |
| f | `reg_max_correspondence_distance` | FE+BE | 2.0 m | 1.0, 3.0 | **It is used by FAST_GICP:** `registrations.cpp:33` → `fast_gicp_impl.hpp:139` rejects nearest neighbours beyond it. It gates correspondences in odometry (FE) and in loop-closure alignment (BE). The generic template and the code default use 2.5 m. |

**Level rule** (shared with KISS): ×0.5 and ×2 of D. A level is replaced by a submitted P1–P3 value that lies on the same side of the default within a factor of 3, choosing the one closest to ×0.5 or ×2. This puts the following submitted values in use, for continuity with the paper: P3 for a, P2 for b, P1 for c_fe and d_fe, P2 for c_be and d_be, and P2 for f. Not used: a P2 = 0.5 (×5), and c/d P1 = 0.5 for the BE parameters (×0.25).

**Physically sensible limits**, also in the YAML:

- a ∈ [0.05, 0.5] m. Below about 0.05 m the leaf is at the VLP-16 range-noise scale and barely reduces the cloud. Above about 0.5 m, the 20 nearest neighbours that FAST_GICP uses for each covariance (`reg_correspondence_randomness` = 20) span several metres.
- b ∈ [0.25, 1.0] m and ≥ 2 × a in any combined configuration, so that each voxel point can still have two neighbours.
- c_fe ∈ [0.25, 5] m and c_be ∈ [1, 5] m (the upstream template range).
- d_fe ≤ 2.0 and d_be ≤ 4.0. Larger values change nothing; they only keep the trigger disabled.
- f ∈ [0.5, 5] m. Motion per scan is about 0.12 m: CitrusFarm averages about 1.2 m/s at 10 Hz (for example `cf_3`: 2035 m in 28:27), and Pullally about 0.3–0.5 m/s at the stated 4 Hz. With the constant-pose initial guess, the residual offset that the gate must cover is about one scan of motion, so 0.5 m keeps a margin.

**Expected checks:**

- For the BE-only parameters (c_be, d_be), the FE output (`/odom`) must equal D within the run-to-run spread. This is a free consistency check, like the floor ablation (5.2).
- In step 5 (interactions), verify b ≥ 2 × a if both a and b change.

## Excluded

- **`registration_method`:** fixed to FAST_GICP by the brief.
- **`sensor_height`:** sensor setting, per dataset (C4).
- **`tilt_deg` (proposed):** this is the LiDAR mounting pitch, which is a sensor setting like `sensor_height`. The submitted P2/P3 (30°, 15°) on a level sensor would make the floor-normal check (10°) reject almost every plane, which is the same as switching floor edges off; Section 5.2 already tests that. It is passed per dataset in `hdl_D.launch`.
- **`reg_max_correspondence_distance`** is *not* excluded, because it is used by FAST_GICP (see f).
- **Parameters outside Table 5.1** are not chosen, because Section 5.0 restricts the choice to the candidate tables:
  - loop closure: `distance_thresh`, `accum_distance_thresh`, `min_edge_interval`, `fitness_score_thresh`;
  - floor: `height_clip_range`, `floor_pts_thresh`, `floor_edge_stddev`;
  - registration: `reg_correspondence_randomness`, `reg_transformation_epsilon`.

  Several of them, especially the loop-closure gates, probably influence BE more than b does (see Open questions).

## Budget

7 parameters × 2 levels × 2 sequences = 28 one-at-a-time runs. Adding D (once on `cf_4`, three times on `cf_5`) gives 32 runs, about 6.8 h of playback, plus cool-down.

## Open questions

1. Approve treating `tilt_deg` as a per-dataset setting (not tuned), and give the pitch for each dataset.
2. Approve tuning the FE and BE keyframe parameters separately (7 parameters) rather than the brief's joint mapping (5 parameters: a, b, c, d, f). The joint alternative scales both nodelets by the same factor: c ∈ {0.5×, 2×} of (1.0, 2.0) and d likewise.
3. Downsampling levels: the draft uses 0.05 / 0.3. Should the submitted P2 = 0.5 be added as a third level for continuity? This costs 2 extra runs.
4. Should a loop-closure gate (for example `fitness_score_thresh`: 2.5 in `_400`, 0.5 in the generic template) be added to the candidate table? Loops (cf_3, cf_7) are where BE differs from FE.

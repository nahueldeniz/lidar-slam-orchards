# Frames and transforms (brief Section 10)

Every evaluated pose is the pose of the **LiDAR frame** in the method's world frame, T_W_L. The conversion for each output is declared in `configs/methods.yaml` (`eval_outputs`) as T_W_L = left · T_out · right, with rotation matrices written out explicitly, and applied by `eval/slameval/outputs.py`. Positions are compared with the ground truth after a rigid alignment (Section 11.2), so any constant rotation or offset between the method's world frame and the ground-truth frame is removed; the `left` matrix only matters for making z point up before the horizontal/vertical split. The `right` matrix matters for the lever arm and the heading error.

## Ground truth

### CitrusFarm

- Source: `ground_truth/<seq>/gt.bag`, topic `/gps/fix/odometry`, `nav_msgs/Odometry`, `header.frame_id = rtk_path_frame`, empty `child_frame_id`; 10 Hz; orientation all zero (not populated); pose covariance all zero. Positions start at (0, 0, 0). The reference point is the GPS receiver centre.
- **Frame check against the raw RTK** (`cf_7`, using the `/piksi/navsatfix_best_fix` messages that the 2024 A-LOAM output bag happens to contain): NavSatFix converted to local ENU (pymap3d, origin at the first fix), associated with `gt.bag` by header stamp, rigid fit gt → ENU:
  - rotation (z, y, x) = (90.0°, 0.001°, 0.000°); translation (0.075, −0.004, 0.005) m;
  - RMS 3D difference after the fit: **0.046 m** (10 117 samples); with a translation-only fit: 39.7 m.
  - So `rtk_path_frame` is a level frame rotated by 90° about z from ENU (x ≈ north, y ≈ west, z up). z is up, so the horizontal/vertical error split is valid without a correction. The check is repeated for every sequence by `scripts/process_gt.py` once the base bags are available (`gt/<seq>/gt_quality.json`, `rtk.enu_consistency`).
- Raw RTK for `cf_7` (same source): `status = 2` for all 10 117 messages; `position_covariance_type = 1` (approximated) with a constant σ_E = σ_N = 0.07 m and σ_U = 0.10 m in every message. A constant value suggests a driver setting rather than a receiver accuracy estimate; to be confirmed against the `piksi_multi_rtk_ros` driver configuration and `/piksi/debug/receiver_state`.
- Lever arm, LiDAR → GPS receiver centre, in the LiDAR frame: (−0.220, 0.000, 0.153) m (`04-lidar-gps-result.txt`, CAD). LiDAR in `base_link`: (0.040, 0.000, 0.3787) m, identity rotation (`05-baselink-lidar-result.txt`, CAD), so the LiDAR x axis is the robot's forward axis (`lidar_forward_axis = (1, 0, 0)` for the heading error).

### Pullally

Pending the inventory (OPEN_QUESTIONS.md Q1, Q6, Q8).

## Methods

Pending the source verification of each method (output frame_id / child_frame_id, axis conventions, static transforms). Each entry will give the numerical check required by the brief: roll and pitch near zero at the start on flat ground, and an XY shape matching the ground truth after alignment.

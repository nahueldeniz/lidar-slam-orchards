# CitrusFarm ground truth: first look at `gt.bag`

This is a first look on 12 October 2026, not the Section 9 processing.

- **Produced by:** `datasets/tools/gt_overview.py`, run as `.venv/bin/python -I datasets/tools/gt_overview.py datasets/citrusfarm/ground_truth datasets/citrusfarm_gt_overview.json`, with rosbags 0.11.5 and the `ROS1_NOETIC` type store. The full numbers are in `datasets/citrusfarm_gt_overview.json`.
- **Input files:** downloaded on 12 October 2026 by `download_citrusfarm.sh small`. The official MD5s were verified, and the SHA-256 sums are in `citrusfarm/SHA256SUMS`.

## Summary

Every `gt.bag` holds exactly one connection: `/gps/fix/odometry`, of type `nav_msgs/Odometry` (rosbags name `nav_msgs/msg/Odometry`). There are no other topics and no `/tf`.

| Seq | Folder | Count | `frame_id` | `child_frame_id` | Header span (s) | Rate (Hz) | dt median / min / max (s) | Gaps > 0.2 s | Orientation | Pose cov. | Twist |
|---|---|---:|---|---|---:|---:|---|---:|---|---|---|
| cf_2 | 02_13B_Jackal | 4447 | `rtk_path_frame` | *(empty)* | 444.56 | 10.001 | 0.1000 / 0.0341 / 0.1599 | 0 | not populated (0, 0, 0, 0) | all zero | all zero |
| cf_3 | 03_13B_Jackal | 17079 | `rtk_path_frame` | *(empty)* | 1707.70 | 10.001 | 0.0998 / 0.0419 / 0.1586 | 0 | not populated (0, 0, 0, 0) | all zero | all zero |
| cf_4 | 04_13D_Jackal | 7694 | `rtk_path_frame` | *(empty)* | 769.26 | 10.001 | 0.1001 / 0.0434 / 0.1626 | 0 | not populated (0, 0, 0, 0) | all zero | all zero |
| cf_5 | 05_13D_Jackal | 7593 | `rtk_path_frame` | *(empty)* | 759.16 | 10.001 | 0.1002 / 0.0448 / 0.1529 | 0 | not populated (0, 0, 0, 0) | all zero | all zero |
| cf_6 | 06_14B_Jackal | 3024 | `rtk_path_frame` | *(empty)* | 302.26 | 10.001 | 0.1001 / 0.0044 / 0.2062 | 1 (0.206 s at t = 278.07 s) | not populated (0, 0, 0, 0) | all zero | all zero |
| cf_7 | 07_14B_Jackal | 10139 | `rtk_path_frame` | *(empty)* | 1013.73 | 10.001 | 0.1002 / 0.0119 / 0.1931 | 0 | not populated (0, 0, 0, 0) | all zero | all zero |

- **Rate:** (count − 1) / header-stamp span. The 1st–99th percentiles of dt are about 0.065–0.135 s for every sequence. No dt is ≤ 0.
- **Orientation:** the quaternion is (x, y, z, w) = (0, 0, 0, 0) in every message. This is not a valid rotation, not the identity. Section 9's TUM conversion must write `0 0 0 1` and must not pass these quaternions to evo.
- **Covariance:** pose and twist covariances are all zero, and the twist is all zero.

## Time, length and comparison with Teng et al. (2023), Table 4

UTC comes from the header stamps. Local time is PDT (UTC−7), Riverside, California.

| Seq | First stamp (UTC) | Local | Paper time of day | Span (min:s) | Paper duration | 3D path (m) | 2D path (m) | Paper distance (m) | End − start (m, x/y/z) |
|---|---|---|---|---|---|---:|---:|---:|---|
| cf_2 | 2023-07-18 21:51:21.917 | 14:51 | 5 pm | 7:24.6 | 07:24 | 524.60 | 521.45 | 524.60 | 0.05 / 217.23 / −2.68 |
| cf_3 | 2023-07-17 02:17:20.972 | 16 Jul 19:17 | 7 pm | 28:27.7 | 28:27 | 2035.22 | 2027.39 | 2035.21 | 0.49 / 0.27 / −0.03 |
| cf_4 | 2023-07-18 21:25:34.226 | 14:25 | 4 pm | 12:49.3 | 12:49 | 896.15 | 890.88 | 896.15 | −1.06 / 0.01 / 0.03 |
| cf_5 | 2023-07-17 00:05:13.474 | 16 Jul 17:05 | 6 pm | 12:39.2 | 12:39 | 865.34 | 859.32 | 865.33 | −1.01 / 0.82 / −0.00 |
| cf_6 | 2023-07-18 21:29:35.938 | 14:29 | 4 pm | 5:02.3 | 05:02 | 356.58 | 355.18 | 356.57 | −37.76 / −0.43 / −0.08 |
| cf_7 | 2023-07-17 01:31:20.239 | 16 Jul 18:31 | 6 pm | 16:53.7 | 16:53 | 1212.75 | 1208.14 | 1212.74 | 0.15 / −0.51 / 0.04 |

- **Durations and distances agree with Table 4 and with brief Section 4.1.** The paper's distances equal the 3D polyline length of `gt.bag`, to within 0.01 m.
- **Loop column:** the end−start offsets agree with it. cf_3, 4, 5 and 7 end within about 1.3 m of their start; cf_2 and cf_6 do not.
- **Origin:** every trajectory starts at exactly (0, 0, 0), so the frame is a local frame with its origin at the first fix.
- **Position extents (m):**

  | Seq | x | y | z |
  |---|---|---|---|
  | cf_2 | [0.00, 46.69] | [−0.70, 217.42] | [−3.27, 0.26] |
  | cf_3 | [−4.76, 47.30] | [−102.69, 115.35] | [−1.85, 1.65] |
  | cf_4 | [−56.60, 2.74] | [−0.17, 39.82] | [−0.53, 0.23] |
  | cf_5 | [−58.53, 4.07] | [−40.25, 3.83] | [−0.25, 0.57] |
  | cf_6 | [−38.68, 0.00] | [−82.31, 0.52] | [−0.10, 0.71] |
  | cf_7 | [−41.98, 0.17] | [−86.44, 1.59] | [−0.10, 0.79] |

- **Point described:** per the dataset website (Calibration page), the GPS-RTK data refer to the centre of the GPS receiver, so `gt.bag` positions are those of the receiver, not the LiDAR. The lever arm is in `citrusfarm_lidar_height_calibration.md`.

## Header stamps, bag time and `gt.csv`

- **Header stamps vs bag record time:** header stamps are 0.4–32 ms earlier than the bag record time (median about 1 ms) in every sequence.
- **`gt.csv` format:** three comment lines (`# ground truth trajectory`, `# file: '<folder>/gt.bag'`, `# timestamp,tx,ty,tz,qx,qy,qz,qw`), then one row per message. The row count matches `gt.bag`, and the positions are identical (maximum difference 0).
- **`gt.csv` timestamp:** **the CSV timestamp is the bag record time, not the header stamp.** It matches bag time to within 2.4 × 10⁻⁷ s (float formatting), but differs from the header stamp by up to 32 ms (cf_5). The quaternion columns are all 0.
- **Consequence:** use the `gt.bag` header stamps (brief Section 10 uses header stamps for the estimates), and do not mix in the CSV times.

## Observations

1. **Irregular spacing.** Intervals vary between about 0.03 and 0.16 s around the nominal 0.1 s, plus one interval of 0.0044 s (cf_6). This suggests the header stamps are ROS receive times, not GNSS epoch times. This is an inference: the post-processing script that produced `gt.bag` is not published.
2. **Recording clock on 18 July 2023.** cf_4 (21:25:34–21:38:23 UTC) and cf_6 (21:29:35–21:34:38 UTC) overlap in time, although the same robot recorded them in different fields. The base-bag file names for 01_13B (14-26-48), 04_13D (14-25-34) and 06_14B (14-29-35) also all start within 4 min of each other on 18 July. The 18 July stamps therefore cannot be absolute wall-clock time. The 16 July sequences (cf_3, cf_5, cf_7) agree with the paper's time of day to within an hour; cf_2, cf_4 and cf_6 do not.
3. **Consistency with the base bags.** For all six sequences, the first `gt.bag` stamp agrees, to the second, with the start time in the first base-bag file name. Ground truth and LiDAR therefore appear to share the robot's ROS clock. This must be checked against the base bags in the inventory (Section 4.3).

## Open questions

1. **Axes of `rtk_path_frame`.** Is it ENU, NED or rotated? Is z up? The website says only "post-processed data (via WGS84)". To be settled in the Section 9 consistency check, by converting `/piksi/navsatfix_best_fix` to ENU (pymap3d, origin at the first fix) and comparing. This needs the base bags.
2. **Absolute time on 18 July.** cf_2, cf_4 and cf_6 have a recording clock that is not absolute time (Observation 2). This does not matter for relative evaluation if LiDAR and ground truth share the clock, but no absolute-time reasoning (for example GNSS time or sun position) should use these stamps.
3. **Header stamps.** Are the `gt.bag` header stamps the receive time of `/piksi/navsatfix_best_fix` or the GNSS epoch? The dataset authors have not stated this. It matters only at the 10–30 ms level (about 1–3 cm at 1 m/s).

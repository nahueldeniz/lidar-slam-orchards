# DLO tuning rationale (DRAFT, not frozen)

**Status: DRAFT.** Section 5.0 steps 1–2 only. Nothing here has been run. Levels are in `configs/dlo/tuning_levels.yaml`; both files must be reviewed by Nahuel and then hashed and frozen before any tuning run.

Source: `vectr-ucla/direct_lidar_odometry` at `11528c02e89a1619ee3c42ba39ecaa992bd20bf4` (master, 19 May 2023; `cfg/dlo.yaml` version 1.4.3). Base configuration: `configs/dlo/D/` (upstream values with `dlo/imu: false`; `adaptiveParams: true`).

References:

- **[DLO]** K. Chen, B. T. Lopez, A. Agha-mohammadi, A. Mehta, "Direct LiDAR Odometry: Fast Localization With Dense Point Clouds", *IEEE RA-L* 7(2):2000–2007, 2022 (arXiv:2110.00605v3).
- **[README]** repository `README.md` at the pinned commit.

## Selection rule

Candidates come from the brief's Table 5.5. All DLO parameters are front-end, so all of them can change `/robot/dlo/odom_node/odom`. Each one below is used in code on every scan or keyframe. Seven parameters are selected; the brief allows up to eight.

## Selected parameters

| ID | Parameter (`params.yaml`) | D | Levels | Why it can matter (source) |
|---|---|---|---|---|
| a | `preprocessing/cropBoxFilter/size` | 1.0 m | 0.5, 2.0 | [DLO] Sec. II-B: one of only two preprocessing filters; it removes returns from the robot itself. In code it is the **half** edge of the removed cube (`odom.cc:123-125`), so D removes a 2 m cube, not the 1 m³ box described in the paper. It also changes the input to the spaciousness metric, which drives the adaptive keyframe threshold. |
| b | `preprocessing/voxelFilter/scan/res` | 0.25 m | 0.15 (P1), 0.5 | [DLO] Sec. II-B: a 0.25 m voxel grid "to lightly downsample the data … while maintaining dominate structures". It sets the density of every scan used by scan-to-scan and scan-to-map GICP, and it also feeds the spaciousness metric. |
| c | `preprocessing/voxelFilter/submap/res` | 0.5 m | 0.3 (P1), 1.0 | [DLO] Sec. II-E: the submap is the concatenation of keyframe clouds, and this leaf is applied to each keyframe cloud when it is stored (`odom.cc:1165-1168`). It sets the density of the scan-to-map target. |
| d2 | `keyframe/threshR` | 45° | 22.5, 90 | [DLO] Sec. II-E-2: keyframes are placed by a translational threshold (adaptive) and a rotational threshold, which the paper holds fixed at 30°. `adaptiveParams` does not change it. In code it creates a keyframe only when at most one keyframe lies within 1.5 × the distance threshold (`odom.cc:1150-1158`), so its effect may be small. That is itself a useful result for Reviewer 1. |
| e1 | `submap/keyframe/knn` | 10 | 5, 20 (P2) | [DLO] Sec. II-E-1 and III-A-1: the submap is the K nearest keyframes plus the convex-hull keyframes; the paper used 10 + 10 and reports a large APE reduction from keyframe-based submaps. |
| e2 | `submap/keyframe/kcv` | 10 | 5, 20 (P2) | As e1: the L nearest convex-hull keyframes ([DLO] Sec. II-E-1). |
| e3 | `submap/keyframe/kcc` | 10 | 5, 20 (P2) | Concave-hull keyframes. They are **not described in [DLO]**; they appear only in the code (`odom.cc:1291-1301`, hull alpha = adaptive distance threshold). Included because the brief lists `kcc` with `knn` and `kcv` (row e), and the submitted P2 changed all three. |

How the levels were chosen (brief Section 5.0, step 2): ×0.5 and ×2 for continuous parameters, rounded for integers. Where a submitted P1–P3 value differs from D and is physically sensible, it replaces the nearer of ×0.5 and ×2: 0.15 and 0.3 (P1) for b and c, and 20 (P2) for e1–e3. The submitted 1° and 5° for d2 were not used: the brief marks their assignment to `threshR` only as "proposed", and 1° would add a keyframe for almost any heading noise. The paper's 30° is a documented alternative and could replace 22.5° if Nahuel prefers it.

Row e is varied **one key at a time** (e1, e2, e3), not as the joint 20/20/20 of P2, so that the effect of each is separable. If Nahuel prefers the joint change for continuity with P2, replace e1–e3 with one grouped parameter at levels {5/5/5, 20/20/20}; this saves 8 runs.

Physically sensible limits are in the YAML. The most important one is technical: PCL 1.8.1's `VoxelGrid` silently returns the **unfiltered** cloud when the leaf is so small that the voxel index would overflow 32 bits (about 0.1 m for a full-range VLP-16 scan). For this reason the voxel levels stay at or above 0.15 m.

## Not tuned, and why

- **`keyframe/threshD` (row d1): excluded because it is inactive.** With `adaptiveParams: true` (fixed, brief C5), `setAdaptiveParams()` runs on every scan before the keyframe test. It overwrites `keyframe_thresh_dist_` with 10 m if spaciousness > 20 m, 5 m if in (10, 20] m, 1 m if in (5, 10] m and 0.5 m if ≤ 5 m (`odom.cc:1193-1209`; [DLO] Eq. 6), and it sets the concave-hull alpha to the same value. The YAML value 5.0 is used only by the constructor's initial hull alpha (`odom.cc:98`), which is overwritten before any hull is computed. `adaptiveParams` changes nothing else: `threshR`, the submap sizes and the voxel sizes are unaffected. Spaciousness is the median point range of the preprocessed scan, low-pass filtered with weights 0.95/0.05 ([DLO] Sec. II-E-2; `odom.cc:995-1015`). Patch `env/patches/dlo/0002` logs the threshold actually used at each keyframe.
- **GICP parameters (row f):** not varied, as the brief proposes. Three of them are inactive anyway: `ransac/iterations`, `ransac/outlierRejectionThresh` and `euclideanFitnessEpsilon` are passed to `pcl::Registration` but never read by NanoGICP's own Levenberg–Marquardt loop (`include/nano_gicp/impl/lsq_registration_impl.hpp:89-115`).
- **IMU and gravity parameters:** LiDAR only.
- **`mapNode/*`:** these affect only the visualisation map node.

## Interactions to keep in mind when reading the sensitivity results

- Parameters a and b also change spaciousness, so they can move the adaptive keyframe threshold between bins. The `[DLO_KF]` log lines show whether a level changed the keyframe spacing as well as the registration input.
- Parameter c changes only keyframes created after start-up. The first keyframe is filtered with the same leaf (`odom.cc:492-495`).

## Open questions (DLO tuning)

1. Bounds of the crop box (a) depend on the base_link→LiDAR geometry of the Jackal (CitrusFarm `05-baselink-lidar-result.txt`) and of the Go1 (Pullally, not documented). Confirm that 0.5 m still removes all robot returns on both platforms.
2. For d2, use 22.5° (×0.5, current) or the paper's 30°?
3. Vary e1–e3 separately (current) or as one group like P2?
4. Keep the DLO mapping node in D and T (current, as upstream)? It does not change odometry, but it adds CPU and memory: every second it converts the whole accumulated map to a ROS message (`map.cc` publish timer), even when nobody subscribes.
5. NanoGICP uses `omp_get_max_threads()` threads, which means all 16 logical CPUs on the benchmark laptop unless `OMP_NUM_THREADS` is set. Decide whether to leave this unset (upstream behaviour) and record it.

# SLAMesh tuning rationale (DRAFT, not frozen)

**Status: DRAFT.** Section 5.0 steps 1–2 only. Nothing here has been run. Levels are in `configs/mesh/tuning_levels.yaml`; both files must be reviewed by Nahuel and then hashed and frozen before any tuning run.

Source: `RuanJY/SLAMesh` at `50b4356e8a24c4fe2e2dc1be37d682e703843314` (master, 5 January 2025). Base configuration: `configs/mesh/D/` (upstream `online.yaml`, with `visualisation_type: 0` and the code defaults written out explicitly).

References:

- **[SLAMesh]** J. Ruan, B. Li, Y. Wang, Y. Sun, "SLAMesh: Real-time LiDAR Simultaneous Localization and Meshing", *ICRA 2023*, pp. 3546–3552 (arXiv:2303.05252v1).
- **[README]** repository `README.md` at the pinned commit, Sections 3.4, 3.5 and 4.1.

## Selection rule

Candidates come from the brief's Table 5.7. SLAMesh has one output, `/lidar_odometry`, which is the scan-to-mesh-map registration result. Any parameter that changes the input points, the cell reconstruction or the registration can therefore change it. Seven parameters from the table are selected. An eighth (h2), which is outside the table, is proposed for Nahuel's decision.

## Selected parameters

| ID | Parameter | D | Levels | Why it can matter (source) |
|---|---|---|---|---|
| b | `range_min` | 0.5 m | 1.0 (P2/P3), 3.0 (P1) | Removes near points before meshing (`map.cpp:60-74`). Upstream changed the default from 3.0 to 0.5 on 2025-01-05 (commit 5f56fd1). The 2024 runs therefore probably used 3.0, the same as P1. In orchards the nearest canopy can lie inside 3 m. |
| c | `register_times` | 5 | 3, 7 (P3) | Maximum number of iterations of the loop *overlap → match → solve → re-reconstruct* ([SLAMesh] Sec. III-C; `map.cpp:1084-1122`). The loop stops earlier if the update falls below 1e-5. The cross-cell query length shrinks to 0 in the last iteration ([SLAMesh] Fig. 3). |
| d | `num_margin_old_cell` | −1 | 100 (P3), 500 (P1) | [README] 4.1 recommends 500 for odometry accuracy and −1 for mesh consistency ("implicit loop closure effect"). 500 was also the upstream default until 2025-01-05. Units are scan steps, so the time window depends on the LiDAR rate. |
| e | `grid` | 1.2 m | 1.0 (P2/P3), 2.4 | The cell edge of the GP map ([SLAMesh] Sec. III-B). The paper used 1.5–1.6 m and reports "a wide tolerance for parameters", with good accuracy still at 3 m cells (Sec. IV-E). The input voxel leaf is `grid/num_test`, so this also changes downsampling. |
| f | `min_points_num_to_gp` | 3 | 2, 5 (P2/P3) | **Confirmed: minimum number of points for Gaussian-process reconstruction of a cell, not ground-plane detection.** A cell is reconstructed only if it holds *more* than this number of (voxel- and range-filtered) points (`map.cpp:170-173, 267-270`). With a sparse VLP-16 this decides how many distant cells enter the map. |
| g | `num_test` | 6 | 3, 10 (P3) | The number of GP test locations per cell side ([SLAMesh] Sec. IV-A: "number of predictions in each cell n_j = 6²"). It sets the vertex density used for matching and also the input voxel leaf `grid/num_test`. |
| i | `variance_sensor` | 0.02 | 0.01, 0.1 (P2/P3) | The GP observation noise ([SLAMesh] Eqs. 1–2, σ_in). **In code it is used as a standard deviation**: it is squared before being added to the kernel diagonal (`cell.cpp:170`). The paper gives σ_in² = 0.02. It sets how strongly the reconstructed surface follows each point, and the vertex uncertainties that gate matching. |
| h2 | `residual_combination` (**proposed, outside Table 5.7**) | false | true | [SLAMesh] Sec. III-C combines the point-to-mesh residuals inside each layer, and the ablation in Sec. IV-E reports that this "also reduces the errors". The upstream default was `true` until 2025-01-05, and the 2024 runs probably used it. Include only if Nahuel approves; it adds 4 runs. |

How the levels were chosen (brief Section 5.0, step 2): ×0.5 and ×2 for continuous parameters, rounded for integers. Where a submitted P1–P3 value differs from D and is physically sensible, it replaces the nearer of ×0.5 and ×2. For `num_margin_old_cell`, which is a categorical "never / after N steps" switch, the two documented finite values are used: 500 ([README], P1, the old default) and 100 (P3).

Coupling to report with the results: `grid` (e) and `num_test` (g) both change the input voxel leaf `grid/num_test` (`slamesher_node.cpp:521`). A change in either is therefore a change in both the map resolution and the downsampling.

## Not tuned, and why

- **`variance_min` (row h): inactive in D and T.** It appears only in the point-to-point solver `Map::computeT` (`map.cpp:949-950`). With `point2mesh: true` (upstream `online.yaml`), every registration step calls `Map::computeTPointToMesh`, which reads the value into a local variable and never uses it (`map.cpp:995`). Varying it would give identical trajectories.
- **`range_max` (row a):** P1–P3 did not vary it. ×2 (120 m) would cross the code's 100 m switch, above which no range filtering is done at all and `range_min` is ignored too (`map.cpp:64-78`). That would not be a one-factor change.
- **`num_thread`, `visualisation_type`:** runtime only. GP reconstruction is per cell and independent; visualisation only reads the map. Both are recorded.
- **`full_cover`:** [README] 4.1 recommends `false` for odometry accuracy, but it is outside Table 5.7 and the budget is at 8. It could replace a lower-priority parameter if Nahuel prefers.
- **`variance_register`, `variance_map_update`, `cross_cell_overlap_length`, `point2mesh`:** these are outside Table 5.7. `point2mesh` would switch to a different registration formulation.

## Open questions (SLAMesh tuning)

1. Approve or drop h2 (`residual_combination: true`)?
2. Bounds of `range_min` depend on the robot geometry (Jackal; Go1 not documented) and on the distance to the canopy.
3. With `num_test: 10`, or with `grid: 1.0` and `num_test: 6`, SLAMesh may process scans slower than real time. Scans are queued, not dropped, but processing stops at shutdown. The protocol's maximum 60 s post-playback wait could therefore cut the end of the trajectory. Check that the last output stamp matches the last scan stamp, or allow a longer wait for SLAMesh.
4. `num_margin_old_cell` is in scan steps. Should Pullally (4 Hz) use the same step counts as CitrusFarm (10 Hz), as the current design does?

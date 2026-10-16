# One-at-a-time sensitivity design (Plan A)

Decided by Nahuel on 12 October 2026 to fit the 5-day deadline: every method is evaluated with its repository defaults (configuration D); there is no tuned configuration T. Reviewer 1's questions ("how each parameter affected the results" and "why the final values were chosen") are answered by a one-at-a-time sensitivity analysis on a held-out sequence, and by the choice of defaults, which cannot be biased by tuning on test data.

## Design

- **Sequence:** CitrusFarm 05 (`cf_5`, field two, semi-dense, lawn-mower path, 865 m, 12.7 min). No test sequence uses field two.
- **Runs:** D three times (run-to-run spread) and D with one parameter changed to each of two levels, once each. 4 parameters × 2 levels × 6 methods = 48 runs, plus 18 D runs.
- **Reported per level:** ΔRMSE, ΔCPU and ΔRSS relative to D (F11, T3), with the run-to-run spread of D for comparison.
- **Levels** (fixed before any run; from the reviewed drafts in `configs/<method>/tuning_levels.yaml`): ×0.5 and ×2 around the default, replaced by a submitted P1–P3 value on the same side of the default within a factor of 3, and limited to physically sensible values.

## Rule for choosing four parameters per method

1. Only parameters that are active in the code at the pinned commit and can change the evaluated output.
2. First, parameters that the submitted configurations P1–P3 varied (continuity with the submitted Fig. 4, which Reviewer 1 asked about).
3. Remaining places go to parameters that cover the method's modules not yet covered (pre-processing, front end, back end), following the emphasis of the method's paper or README.

## Selected parameters

| Method | Parameter (key) | Module | Default | Levels | Reason |
|---|---|---|---|---|---|
| hdl | downsample resolution (`downsample_resolution`) | FE+BE | 0.1 m | 0.05, 0.3 | varied in P1–P3 |
| hdl | max. correspondence distance (`reg_max_correspondence_distance`) | FE+BE | 2.0 m | 1.0, 3.0 | varied in P1–P3 |
| hdl | keyframe delta translation, odometry (`keyframe_delta_trans`, scan-matching nodelet) | FE+BE | 1.0 m | 0.5, 2.0 | varied in P1–P3 (row c, front-end part) |
| hdl | keyframe delta translation, graph (`keyframe_delta_trans`, graph nodelet) | BE | 2.0 m | 1.0, 4.0 | varied in P1–P3 (row c, back-end part) |
| lego | segment theta (`segmentTheta`) | pre-processing | 60° | 10°, 30° | varied in P1–P3; ≥ 90° disables clustering, so no ×2 level |
| lego | nearest-feature search distance (`nearestFeatureSearchSqDist`) | FE | 25 m² | 10, 50 | varied in P1–P3 |
| lego | edge threshold (`edgeThreshold`) | FE | 0.1 | 0.05, 0.2 | feature extraction (paper) |
| lego | mapping process interval (`mappingProcessInterval`) | BE | 0.3 s | 0.15, 0.6 | only back-end parameter in the table |
| aloam | nearby scan (`NEARBY_SCAN`) | FE | 2.5 | 1.5, 4 | varied in P1–P3 (compared with integer rings: ±2 → ±1, ±3) |
| aloam | Huber loss, odometry | FE | 0.1 m | 0.05, 0.2 | varied in P1–P3 |
| aloam | Huber loss, mapping | BE | 0.1 m | 0.05, 0.2 | varied in P1–P3 |
| aloam | plane resolution (`mapping_plane_resolution`) | BE | 0.4 m | 0.2, 0.8 | back-end map density (planar features dominate on the ground) |
| dlo | scan voxel size | FE | 0.25 m | 0.15, 0.5 | varied in P1–P3 |
| dlo | sub-map voxel size | FE | 0.5 m | 0.3, 1.0 | varied in P1–P3 |
| dlo | keyframe rotation threshold (`threshR`) | FE | 45° | 22.5°, 90° | varied in P1–P3 (row d2) |
| dlo | sub-map keyframes, k-nearest / convex / concave (joint) | FE | 10 | 5, 20 | varied jointly in P1–P3 (row e) |
| kiss | initial threshold | FE | 2.0 m | 1.0, 4.0 | varied in P1–P3 |
| kiss | max. points per voxel | FE | 20 | 10, 50 | varied in P1–P3 |
| kiss | voxel size | FE | 1.0 m | 0.5, 2.0 | varied in P1–P3 |
| kiss | minimum range | FE | 5.0 m | 2.5, 0.5 | varied in P1–P3 (all used 0.5 m); ×2 = 10 m would discard nearly all tree returns |
| mesh | minimum range (`range_min`) | FE | 0.5 m | 1.0, 3.0 | varied in P1–P3; default changed 3.0 → 0.5 in January 2025 |
| mesh | margin for old cells (`num_margin_old_cell`) | FE | −1 | 100, 500 | varied in P1–P3; default changed 500 → −1 in January 2025 |
| mesh | cell (grid) size (`grid`) | FE | 1.2 m | 1.0, 2.4 | varied in P1–P3; key parameter in the SLAMesh paper |
| mesh | GP test points per cell side (`num_test`) | FE | 6 | 3, 10 | varied in P1–P3; sets the input voxel size with `grid` |

## Not selected (all varied in P1–P3 or in the brief's tables)

- **hdl:** outlier-removal radius and the two keyframe angle thresholds (the defaults effectively make keyframes translation-driven); `tilt_deg` is a mounting setting (0° for CitrusFarm), not a tuning parameter.
- **lego:** `edgeFeatureNum` and `surfFeatureNum` are never read by the code (hard-coded 2 and 4); `surfThreshold` mirrors `edgeThreshold`.
- **aloam:** `DISTANCE_SQ_THRESHOLD` and the line resolution (not varied in P1–P3; the module is already covered).
- **dlo:** crop-box size (not varied in P1–P3); `threshD` is inactive with adaptive keyframing.
- **kiss:** minimum motion threshold (varied only in P1).
- **mesh:** registration iterations, minimum points for GP reconstruction and sensor variance (module covered; time budget); `variance_min` is inactive with point-to-mesh registration; `residual_combination` is outside the brief's table (OPEN_QUESTIONS.md Q11).

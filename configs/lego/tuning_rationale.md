# DRAFT: LeGO-LOAM tuning rationale (brief Section 5.0, steps 1 and 2)

Status: **DRAFT**, not frozen. Levels are in `configs/lego/tuning_levels.yaml`. Source commit `896a7a9` (master).

## Sources cited

- **[P]** T. Shan and B. Englot, "LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain", IROS 2018. Preprint shipped in the repository as `Shan_Englot_IROS_2018_Preprint.pdf`.
- **[R]** Repository `README.md` at `896a7a9`.
- **[C]** Source code at `896a7a9`. File and line numbers refer to the unpatched upstream files.

## Selection

The candidate table (brief Section 5.3) has seven rows. Five can change the evaluated outputs and are selected. Two cannot change anything and are excluded:

| Row | Constant | Selected | Reason |
|---|---|---|---|
| a | `mappingProcessInterval` | yes | BE; sets how often scan-to-map refinement runs |
| b | `segmentTheta` | yes | segmentation before feature extraction; affects FE and BE |
| c | `edgeFeatureNum` | **no** | Declared at `utility.h:120` but never referenced. `featureAssociation.cpp:709` hard-codes 2 ([C]). |
| d | `edgeThreshold` | yes | FE feature extraction |
| e | `surfFeatureNum` | **no** | Declared at `utility.h:121` but never referenced. `featureAssociation.cpp:747` hard-codes 4 ([C]). |
| f | `surfThreshold` | yes | FE feature extraction |
| g | `nearestFeatureSearchSqDist` | yes | FE correspondence gate |

Also not tuned:

- `scanPeriod`, which is a sensor setting (brief Section 5.0).
- `loopClosureEnableFlag`, which stays `false` as the brief requires.

All five selected parameters reach the headline output `/integrated_to_init`. That output fuses odometry (FE) with mapping (BE) in `transformFusion.cpp`.

## Parameters

### a. `mappingProcessInterval`: 0.3 s → levels 0.15 s and 0.6 s (BE)

- **Why it matters.** [P] Sec. III-A, III-E and Fig. 1(b): lidar mapping refines the odometry pose against a surrounding map "at a lower frequency" (2 Hz in Fig. 1(b), against 10 Hz odometry). Transform integration then fuses the two estimates. This interval sets how often drift is corrected (`mapOptmization.cpp:1499`), so it can change `/aft_mapped_to_init` and `/integrated_to_init`, but not `/laser_odom_to_init`.
- **Levels.** ×0.5 and ×2, following the brief's rule. P1–P3 all used 0.3 s, so no submitted value differs from the default.
- **Physical limits.** Features reach mapping only on every second scan (`featureAssociation.cpp:1792`). At 10 Hz the effective mapping periods are therefore 0.2 s, 0.4 s (D) and 0.6 s, which are three distinct levels. At 4 Hz (if Pullally is 4 Hz), 0.15 s and 0.3 s both give 0.5 s.

### b. `segmentTheta`: 60° → levels 10° and 30° (FE+BE)

- **Why it matters.** [P] Sec. III-B: an image-based segmentation groups range-image points into clusters. Clusters with fewer than 30 points are dropped, because "small objects, e.g., tree leaves, may form trivial and unreliable features". [P] Sec. IV-A shows that this is LeGO-LOAM's main advantage in vegetated terrain, and an orchard is such terrain. `segmentTheta` is the angle threshold of that segmentation (`imageProjection.cpp:423`). The code comment at `utility.h:113` says "decrese this value may improve accuracy" ([C]). Segmentation runs before feature extraction, so it affects both FE and BE outputs.
- **Levels.** The submitted P1 (10°) and P2 (30°, which is also ×0.5), for continuity with the submitted paper.
- **Physical limits.** The segmentation angle computed at `imageProjection.cpp:421` is `atan2(d2·sin α, d1 − d2·cos α)` with d1 ≥ d2, so it is always below 90°. A threshold of 90° or more (×2 = 120°) would put every point in its own cluster and discard all non-ground points. No upper level is therefore physically meaningful. Both levels lie below the default; see the open questions.
- **Direction of the effect.** A smaller θ merges more neighbours, so fewer points are discarded. A larger θ discards more points.

### d. `edgeThreshold` and f. `surfThreshold`: 0.1 → levels 0.05 and 0.2 each (FE)

- **Why it matters.** [P] Sec. III-C: a roughness threshold c_th separates edge features (c > c_th) from planar features (c < c_th). [P] Sec. IV-A discusses exactly this trade-off in vegetation: "we can increase c_th to extract more stable edge features ... but this change may result in an insufficient number of useful edge features if the robot enters a relatively clean environment. Similarly, decreasing c_th will also give rise to a lack of useful planar features". Orchard rows combine foliage, trunks and open ground, so this trade-off applies directly. The code uses two thresholds with equal defaults (`featureAssociation.cpp:705` for edges, `:740` for flat points). Each is tuned separately.
- **Levels.** ×0.5 and ×2 for each. P1–P3 all used 0.1.
- **Unit.** The code's roughness is the square of the unnormalised sum of range differences over ±5 neighbours (`featureAssociation.cpp:626-633`), so its unit is m². This differs from [P] eq. (1), which is normalised and dimensionless.
- **Interaction.** Edges are taken only from non-ground points and flat points only from ground points (`featureAssociation.cpp:706`, `:741`). The two thresholds therefore never compete for the same point, even if T combines them.

### g. `nearestFeatureSearchSqDist`: 25 m² → levels 10 m² and 50 m² (FE)

- **Why it matters.** [P] Sec. III-D defers the correspondence search to LOAM. This constant is the squared-distance gate for the nearest edge and planar correspondences between consecutive scans (`featureAssociation.cpp:1057, 1061, 1168, 1172`). Too small a gate drops correspondences in fast turns. Too large a gate admits wrong associations between similar-looking trees in neighbouring rows. It is FE only: mapping uses its own fixed 1 m² gate (`mapOptmization.cpp:1101, 1183`).
- **Levels.** P1 = 10 m² (gate 3.2 m), for continuity, and ×2 = 50 m² (gate 7.1 m). Both are far larger than the motion between scans (about 0.14 m at 1.35 m/s and 10 Hz), so a small sensitivity is plausible. That is still informative for the response to Reviewer 1.

## Run budget

There are 5 parameters × 2 levels = 10 single-change images. Each runs once on `cf_4` and once on `cf_5`, giving 20 runs. Measuring the D spread adds 3 runs on `cf_5`.

## Open questions

1. **Segment theta has no upper level.** Both `segmentTheta` levels lie below the default, because ×2 is impossible (angles ≥ 90° disable clustering). Should an upper level such as 75° be added instead of P1 = 10°, so that the parameter is tested in both directions?
2. **`edgeFeatureNum` and `surfFeatureNum` are dead constants.** The submitted paper reported them as parameters. Changing them has no effect, and making them effective would require editing `extractFeatures()`, which is not an allowed patch. Confirm that they are reported as "not a parameter in the code" rather than tuned.
3. **Paper versus code.** [P] gives n_𝔽e = 40 and n_𝔽p = 80. The code uses 20 less-sharp points per sub-image (`:713`) and all remaining non-edge points after voxel filtering. [P] uses a 100 m surrounding map, whereas the code uses `surroundingKeyframeSearchRadius = 50` (`utility.h:129`). This is for the record only; nothing is changed.
4. **Mapping interval at 4 Hz.** If Pullally is 4 Hz, level a = 0.15 s gives the same effective mapping period as D. This needs no action, because tuning uses CitrusFarm at 10 Hz, but it should be noted when T is transferred to Pullally.

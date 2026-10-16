# DRAFT: A-LOAM tuning rationale (brief Section 5.0, steps 1 and 2)

Status: **DRAFT**, not frozen. Levels are in `configs/aloam/tuning_levels.yaml`. Source commit `e51f88c` (devel).

## Sources cited

- **[R]** A-LOAM `README.md` at `e51f88c`: "A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver". The README describes no parameters.
- **[L]** A-LOAM launch files at `e51f88c`, which document per-sensor values:
  - `aloam_velodyne_VLP_16.launch`: line 0.2 m, plane 0.4 m, minimum range 0.3 m.
  - `aloam_velodyne_HDL_64.launch`: line 0.4 m, plane 0.8 m, minimum range 5 m.
- **[Z]** J. Zhang and S. Singh, "LOAM: Lidar Odometry and Mapping in Real-time", RSS 2014. This is the method A-LOAM implements. The paper is not in the repository; the section references below need checking against the published paper (see the open questions).
- **[C]** Source code at `e51f88c`. Line numbers refer to the unpatched files.
- **[Ceres]** Ceres Solver 1.12.0 `HuberLoss(a)`: ρ(s) = s for s ≤ a², and 2a√s − a² otherwise, where s is the squared residual norm. The scale a is therefore a residual magnitude, in metres for A-LOAM's point-to-line and point-to-plane residuals.

## Selection

All six rows of the candidate table (brief Section 5.4) can change the evaluated outputs, so all six are selected:

- three FE parameters: a, d and e;
- three BE parameters: b, c and f.

FE parameters change `/laser_odom_to_init` and, through it, both full outputs. BE parameters change only `/aft_mapped_to_init` and `/aft_mapped_to_init_high_frec`.

Also not tuned:

- the scan period, which is a sensor setting;
- `scan_line`, `minimum_range` and `mapping_skip_frame`, which brief Section 5.4 fixes at the VLP-16 launch values.

## Parameters

### a. `DISTANCE_SQ_THRESHOLD`: 25 m² → levels 12.5 m² and 50 m² (FE)

- **Why it matters.** [Z] Sec. V-B (finding feature-point correspondences) finds, for each edge or planar point of the current sweep, its closest point in the previous sweep. `DISTANCE_SQ_THRESHOLD` rejects candidates whose squared distance exceeds it, both for the closest point and for the second and third support points (`laserOdometry.cpp:305, 310, 393, 399`).
  - A smaller gate drops correspondences when inter-scan motion is large.
  - A larger gate admits wrong matches, for example to the next tree in a row.
- **Levels.** ×0.5 (gate 3.5 m) and ×2 (gate 7.1 m). P1–P3 all used 25.

### b. `mapping_line_resolution`: 0.2 m → levels 0.1 m and 0.4 m (BE)

- **Why it matters.** [Z] Sec. VI (mapping) down-samples the map with a voxel grid filter. A-LOAM uses this value as the leaf size for the edge (corner) map and for the current corner features before scan-to-map matching (`laserMapping.cpp:902, 905`; filters applied at `:543` and `:793`). It sets the density and cost of the map that refines odometry.
- **Levels.** ×0.5 and ×2. The ×2 level of 0.4 m is also documented elsewhere:
  - it is the HDL-64 launch value [L];
  - it is the code default (`laserMapping.cpp:902`);
  - it is the value reported for the submitted P1–P3.

  The ×2 level therefore covers both the alternative documented upstream and continuity with the submitted paper.

### c. `mapping_plane_resolution`: 0.4 m → levels 0.2 m and 0.8 m (BE)

- **Why it matters.** This is the same mechanism as b, for the planar (surface) map (`laserMapping.cpp:903, 906`; filters applied at `:548` and `:798`). Planar features dominate on open orchard ground.
- **Levels.** ×0.5 and ×2. The ×2 level of 0.8 m is the HDL-64 launch value [L], the code default and the submitted P1–P3.

### d. `NEARBY_SCAN`: 2.5 → levels 1.5 and 4 (FE)

- **Why it matters.** [Z] Sec. V-B takes the second point of an edge correspondence from the scan lines next to that of the closest point, and the planar support points from the same and neighbouring scan lines. `NEARBY_SCAN` bounds how many ring indices away those support points may lie (`laserOdometry.cpp:319, 345, 405, 433`). With only 16 rings at 2° spacing, this sets how far apart in elevation the line or plane support points can be.
- **Discrete behaviour.** Ring ids are integers, so the comparison `id > closest + NEARBY_SCAN` makes 2.5 act as ±2 rings. Any value in [2, 3) is equivalent.
- **Levels.**
  - 1.5 (±1 ring): one step down. ×0.5 = 1.25 behaves identically.
  - 4 (±4 rings): the submitted P1, and ×2 of the effective default of 2 rings.
- **Physical limits.** Values below 1 make edge correspondences impossible, because the second point must lie on another ring.

### e. Huber scale in odometry: 0.1 m → levels 0.05 m and 0.2 m (FE)

- **Why it matters.** [Z] Sec. V-C (motion estimation) down-weights feature points with large residuals, using bisquare weights in the original LOAM. A-LOAM replaces that weighting with a Ceres Huber loss of scale 0.1 m (`laserOdometry.cpp:284`, one active occurrence; the commented-out alternative at line 283 is a null loss). The scale sets the residual above which a correspondence counts as an outlier. In foliage, many residuals are large.
- **Levels.** ×0.5 = 0.05 m (equal to the submitted P3) and ×2 = 0.2 m. The submitted P1 of 0.125 m is only 25 % above the default and was not used, to keep a clear contrast; see the open questions. The VLP-16 range accuracy is ±3 cm (stated in the LeGO-LOAM paper, Sec. II), so 0.05 m is about the smallest physically sensible scale.

### f. Huber scale in mapping: 0.1 m → levels 0.05 m and 0.2 m (BE)

- **Why it matters.** This is the same robust loss, in the scan-to-map optimisation (`laserMapping.cpp:565`, one active occurrence; the commented-out alternative at line 564 is a null loss).
- **Levels.** ×0.5 and ×2. The submitted P1 of 0.75 m (×7.5) lies outside the ×0.5–×2 band. At that scale almost every residual is in the quadratic region, so the loss is nearly non-robust. It is not used as a level; see the open questions.

**Note.** Each Huber loss appears exactly once (active) per file. Both are now compile-time constants (`ODOM_HUBER_DELTA`, `MAPPING_HUBER_DELTA`) and are handled in the same way: an `#ifdef` override hook with upstream default 0.1, printed at start-up.

## Run budget

There are 6 parameters × 2 levels = 12 single changes, each run once on `cf_4` and once on `cf_5`: 24 runs. Measuring the D spread adds 3 runs on `cf_5`.

Images needed: D plus 8 build-arg images. Parameters b and c change only the launch file and reuse the D image.

## Open questions

1. **Huber levels versus submitted values.** For the odometry Huber scale, should the submitted P1 = 0.125 m replace ×2 = 0.2 m for continuity? For the mapping Huber scale, should P1 = 0.75 m be added as a third, documented "near non-robust" level?
2. **Section references to [Z].** The LOAM paper is not in the repository, so the section numbers above follow its published structure as recalled. Check them against the paper before they are cited in the response letter.
3. **Submitted resolutions.** The submitted P1–P3 list line/plane resolutions of 0.4/0.8 m, which are the code defaults. The VLP-16 launch, which is the base of D, uses 0.2/0.4 m. Either the 2024 runs used the code defaults, which would mean the launch parameters were not loaded, or the table reported code defaults. Its 2024 configuration files are not available.

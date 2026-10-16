# DRAFT: KISS-ICP tuning rationale (brief Section 5.0, steps 1–2)

**Status: DRAFT, not frozen.** Nahuel must approve these levels before any tuning run. The levels are listed in `tuning_levels.yaml`; configuration D is `D/kiss_D.launch`, and the source checks are in `D/README.md`.

Source: `PRBonn/kiss-icp` v0.3.0 (`460c6f2`), ROS 1 node, launch `ros/launch/odometry.launch`. KISS-ICP is odometry only, so every parameter affects the only output, FE (`/kiss/odometry`).

## Sources used for justification

- **Paper:** Vizzo, Guadagnino, Mersch, Wiesmann, Behley and Stachniss (2023), "KISS-ICP: In Defense of Point-to-Point ICP – Simple, Accurate, and Robust Registration If Done the Right Way", *IEEE RA-L* 8(2). The PDF linked from the README was checked:
  - Sec. III-B: double down-sampling with voxel sizes αv and βv.
  - Sec. III-C: voxel-grid local map with voxel size v, at most Nmax points per voxel, and voxels removed beyond rmax.
  - Sec. III-D: adaptive threshold, with σt computed from deviations larger than δmin, "In our experiments, we set this threshold δmin to 0.1 m", and τt = 3σt.
  - Sec. III-E: Geman-McClure kernel with scale σt/3.
  - Sec. III-F and **Table I** ("All seven parameters of our approach"): τ0 = 2 m, δmin = 0.1 m, Nmax = 20, v = 0.01 rmax, α = 0.5, β = 1.5, γ = 10⁻⁴. The paper states "We use the same parameters for all experiments". It does not treat rmax as a system parameter, but says it "might also be adapted to the specific environment".
- **README:** KISS-ICP "just works on most of the cases without tunning any parameter". `config/README.md`: "In most of the cases it should be enough to use the default parameters 'as is'. If you need extra tunning …".
- **`config/advanced.yaml`** (upstream example) uses `min_range: 0.0` and `deskew: True`. It is a documented alternative for `min_range`.
- **The code paths**, traced in `D/README.md` §1.

## Selection: all 5 rows of brief Table 5.6

All five rows can change the output. The four paper parameters exposed in ROS (τ0, δmin, Nmax, v) are the full user-facing parameter set of the paper's Table I; α, β and γ are compile-time constants. The fifth row, `min_range`, is a pre-filter that is not in the paper. With the launch default of 5 m, however, it discards every return within 5 m of the sensor. In an orchard, that is where the adjacent tree rows and most ground returns are, so it is likely the most influential setting for these data.

| id | Parameter | D | Levels | Why it can change the output |
|---|---|---|---|---|
| a | `initial_threshold` (τ0) | 2.0 m | 1.0, 4.0 | It is the correspondence gate (3τ0) and kernel scale (τ0/3) until the robot has moved 5δmin and the first deviation larger than δmin is seen. Its effect is limited to start-up, but a bad start offsets the whole trajectory. |
| b | `min_motion_th` (δmin) | 0.1 m | 0.05, 0.2 | It decides which model deviations enter σ (paper Eq. 9–10), and therefore the gate τ = 3σ for the whole run. Slow, smooth platforms (Jackal ~1.2 m/s; Go1 0.3–0.5 m/s) produce many small deviations. |
| c | `max_points_per_voxel` (Nmax) | 20 | 10, 50 | It sets the density of the local map used for nearest-neighbour association (Sec. III-C). |
| d | `voxel_size` (v) | 1.0 m | 0.5, 2.0 | It sets the map resolution and both scan down-samplings (0.5v, 1.5v). This is the main resolution knob. Orchard structure (trunks, canopy edges) is finer than the 1 m default, which was designed for 0.01·rmax with rmax = 100 m. |
| e | `min_range` | 5.0 m | 2.5, 0.5 | It removes near returns before everything else (`Preprocessing.cpp:69`). |

**Level rule** (shared with hdl): ×0.5 and ×2 of D. A level is replaced by a submitted P1–P3 value that lies on the same side of the default within a factor of 3, choosing the one closest to ×0.5 or ×2. As a result:

- a uses P1/P2 = 1.0 (which is also ×0.5).
- c uses P1/P2 = 50 instead of 40.
- b (P1 = 0.5, ×5) and d (P = 0.25 and 0.1, ×0.25 and ×0.1) keep the ×0.5/×2 levels, because the submitted values fall outside the factor-3 window.

**Exception, `min_range`:** the ×2 level (10 m) is replaced by the submitted 0.5 m. The draft therefore uses two levels below the default (2.5 m and 0.5 m). Reasons:

1. The purpose of `min_range` is to drop returns from the platform itself and near-field artefacts. A 10 m minimum would remove the adjacent tree rows and the ground, which is the structure the method registers against. That is outside physically sensible limits for a ground robot between rows.
2. The upstream example `config/advanced.yaml` uses 0.0, so a near-zero value is a documented alternative.
3. 0.5 m is the value of all three submitted configurations.

The lower limit of 0.5 m assumes no part of either robot lies more than 0.5 m from the LiDAR origin. This is **to be checked** (Jackal sensor mast; Go1 mount).

**Physically sensible limits**, also in the YAML:

- τ0 ∈ [0.5, 5] m: below about 0.5 m the first registrations may lack correspondences; above 5 m (gate 15 m) they admit gross mismatches.
- δmin ∈ [0.02, 0.5] m.
- Nmax ∈ [5, 100].
- v ∈ [0.25, 2] m: below 0.25 m the voxel is finer than the VLP-16 ring spacing beyond about 7 m (2° vertical resolution), and the nearest-neighbour search only looks in the 3×3×3 voxel block around a point, so correspondences between rings start to be lost (and run time grows); above 2 m the registration cloud (1.5v = 3 m) keeps only a few points per tree.
- `min_range` ∈ [0.5, 5] m.

**Interactions to watch in step 5:**

- v and Nmax together set the local-map density.
- `min_range` and v: with v = 2 m and `min_range` = 0.5 m, near-field points dominate.
- Because δ_rot scales with rmax (fixed at 100 m), δmin acts mostly on rotation deviations: 0.1 m corresponds to about 0.06° of rotation.

## Fixed (not tuned)

- `max_range` = 100 m: brief; it is a sensor value (paper Sec. III-F). It also drives δ_rot. `voxel_size` is now explicit, so it no longer follows `max_range`.
- `deskew` = false; `visualize` = false (brief).

## Budget

5 parameters × 2 levels × 2 sequences = 20 one-at-a-time runs. Adding D (once on `cf_4`, three times on `cf_5`) gives 24 runs, about 5.1 h of playback, plus cool-down.

## Open questions

1. Approve the `min_range` exception (levels 2.5 m and 0.5 m instead of 2.5 m and 10 m), and confirm that 0.5 m clears both robot bodies.
2. Should the submitted `voxel_size` 0.25 m (P1/P2) be added as an extra level for continuity? It is ×0.25 and falls outside the factor-3 window. Cost: 2 runs.
3. Should the submitted `min_motion_th` 0.5 m (P1) be added as an extra level? Cost: 2 runs.

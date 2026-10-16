# CitrusFarm: LiDAR height above ground from calibration and robot description

This feeds hdl `sensor_height` for CitrusFarm (brief Section 5.1, C4). Nahuel approves the value used. Written on 12 October 2026.

**Result (calibration route): LiDAR origin ≈ 0.442 m above flat ground** (0.0635 m base_link height from the Jackal URDF + 0.3787 m from `05-baselink-lidar-result.txt`). The plausible range from the Jackal sources is 0.441–0.444 m, before tyre deflection, soil sinkage and robot tilt. The data-based RANSAC estimate (last section) is still to come.

## 1. CitrusFarm calibration files (exact contents)

Downloaded from the official S3 bucket on 12 October 2026. The official MD5s match, and the SHA-256 sums are in `datasets/citrusfarm/SHA256SUMS`. Local copies are in `datasets/citrusfarm/calibration/`.

`Calibration/results/04-lidar-gps-result.txt` (421 bytes):

```
parent frame: lidar (Velodyne)
child frame: GPS-RTK (origin located at the center of the GPS receiver on the robot)

# This result is measured and computed directly from CAD models of the robot and the mounting board.
x, y, z = -0.220, 0.000, 0.153

# orientation is omitted since it is not meaningful; GPS-RTK only provides 3D positioning data, rather than full 6D poses, because it has only one receiver on the robot.

```

`Calibration/results/05-baselink-lidar-result.txt` (304 bytes):

```
parent frame: base_link (Wheel Odometry frame, located at the bottom center of the robot)
child frame: lidar (Velodyne)

# This result is measured and computed directly from CAD models of the robot and the mounting board.
x, y, z, qx, qy, qz, qw = 0.0400, 0.0000, 0.3787, 0.0000, 0.0000, 0.0000, 1.0000

```

### Frame conventions

Teng et al. (2023, arXiv:2309.15332v2, Section 4.4) state that the seven parameters (x, y, z, qx, qy, qz, qw) "define the transformation of a child reference frame with respect to a parent reference frame". So (x, y, z) is the position of the child frame's origin, expressed in the parent frame, and q is the orientation of the child in the parent (the usual ROS/tf `parent → child` convention, T_parent_child). The paper's Table 3 and the website image `docs/figures/extrinsic_param_summary.png` give the same numbers, with frame names `base_link → velodyne_lidar` and `velodyne_lidar → gps_rtk`, and an identity quaternion for both.

| File | Parent | Child | x | y | z | Rotation | Meaning |
|---|---|---|---|---|---|---|---|
| 04 | lidar (Velodyne) | GPS-RTK, "centre of the GPS receiver" | −0.220 | 0.000 | +0.153 | omitted in the file; identity in Table 3 | The receiver point is 0.220 m behind (−x) and 0.153 m above (+z) the LiDAR origin, on its x–z plane |
| 05 | base_link (bottom centre of the robot base) | lidar (Velodyne) | +0.0400 | 0.0000 | +0.3787 | identity | The LiDAR origin is 0.040 m ahead (+x) of and 0.3787 m above (+z) base_link, with axes parallel to base_link |

Axis directions: base_link follows REP-103 in the Jackal URDF (x forward, y left, z up). Because the rotation is identity, the LiDAR axes are the same. The CitrusFarm frame figure (website Calibration page; paper Fig. 1b) shows the frames, but the numbers come only from the table.

### Check against the brief

- **Lever arm LiDAR → GPS: (−0.220, 0.000, 0.153) m. Agrees.** It is expressed in the LiDAR frame, so brief Section 10's p_ant = p_lidar + R_lidar · l with l = (−0.220, 0.000, 0.153) is consistent with the file.
- **LiDAR 0.3787 m above base_link. Agrees.** The brief omits the +0.040 m forward (x) offset. That offset does not affect the height on level ground, but it matters for any base_link ↔ LiDAR pose conversion.
- **base_link "at the bottom centre of the robot base". Agrees** with the file, the website ("located at the center bottom of the robot base") and the paper (Section 3, wheel odometry).

## 2. Height of base_link above the ground (Clearpath Jackal description)

**Source:** [github.com/jackal/jackal](https://github.com/jackal/jackal), file `jackal_description/urdf/jackal.urdf.xacro`.

- **melodic-devel** at commit [`0d8d76f96bd52102b69a3b9cb735fd5f9e15f695`](https://github.com/jackal/jackal/blob/0d8d76f96bd52102b69a3b9cb735fd5f9e15f695/jackal_description/urdf/jackal.urdf.xacro) (13 May 2022). This is the branch that matches the CitrusFarm robot's Ubuntu 18.04 / ROS Melodic (Teng et al., Section 3).
- **noetic-devel** at commit [`4ddf9b578bb7abce1115c8dc59d8b7f86aa9268c`](https://github.com/jackal/jackal/blob/4ddf9b578bb7abce1115c8dc59d8b7f86aa9268c/jackal_description/urdf/jackal.urdf.xacro) (24 May 2024, default branch). It is identical in every line cited below.
- The same values are also on kinetic-devel (`6bbc110`) and indigo-devel (`1a9136e`).

| Line(s) | Content | Value |
|---|---|---|
| 7 | `<xacro:property name="wheel_vertical_offset" value="0.0345" />` | wheel axle 0.0345 m **above** chassis_link |
| 8 | `<xacro:property name="footprint_vertical_offset" value="-0.0655" />` | used only at line 105, the visual mesh origin |
| 10 | `<xacro:property name="wheel_radius" value="0.098" />` | 0.098 m (collision cylinder, line 39) |
| 15 | `<xacro:property name="chassis_height" value="0.184" />` | 0.184 m |
| 82–93 | the four wheel joints: `<origin xyz="${±wheelbase/2} ${±track/2} ${wheel_vertical_offset}" …/>`, parent `chassis_link` (line 63) | axle at z = +0.0345 m in chassis_link |
| 95–101 | `<link name="base_link">`; joint `base_link_joint` (fixed), `<origin xyz="0 0 0" rpy="0 0 0" />`, parent `base_link`, child `chassis_link` | base_link ≡ chassis_link |
| 105 | chassis visual: `<origin xyz="0 0 ${footprint_vertical_offset}" rpy="${PI/2} 0 ${PI/2}"/>`, mesh `jackal-base.stl` | – |

**There is no `base_footprint` link or `base_footprint → base_link` joint** in `jackal.urdf.xacro`, or anywhere in the repository on melodic-devel or noetic-devel (checked with `grep -rI base_footprint` over every text file at both commits). The brief's instruction to look at that joint therefore cannot be followed literally. The height follows from the wheel geometry instead.

**Derivation.** On flat ground, with rigid wheels at the nominal radius, the contact plane is one wheel radius below the axle:

z_ground (in base_link) = wheel_vertical_offset − wheel_radius = 0.0345 − 0.098 = **−0.0635 m**

So **base_link is 0.0635 m above the ground** (URDF parameters).

**Cross-checks** (computed with `datasets/tools/stl_bbox.py` on the melodic-devel meshes):

- **Base mesh.** After the line-105 origin, `jackal-base.stl` spans z = 0.000 to 0.184 m in chassis_link, equal to `chassis_height`. So base_link is at the very bottom of the chassis body, which agrees with CitrusFarm's "bottom center of the robot base". In the raw mesh the bottom is at +0.0655 m, so `footprint_vertical_offset` only moves the mesh. It is not a footprint frame.
- **Wheel mesh.** The outer radius of `jackal-wheel.stl` is 0.0998–0.1000 m. With that radius, base_link would be 0.1000 − 0.0345 = 0.0655 m above the ground. This equals |`footprint_vertical_offset`|, which suggests the base mesh was authored with its origin on the ground for the 0.1 m mesh tyre.
- **Clearpath manual.** The [Jackal user manual](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal) gives ground clearance 65 mm and "194 mm diameter wheels". If the chassis bottom (= base_link) is the lowest point, the height is 0.065 m. A 0.097 m radius gives 0.097 − 0.0345 = 0.0625 m.

| Source | base_link height above ground (m) |
|---|---:|
| URDF parameters (`wheel_radius` 0.098, `wheel_vertical_offset` 0.0345) | **0.0635** |
| URDF wheel mesh radius (0.100) | 0.0655 |
| Manual: wheel diameter 194 mm | 0.0625 |
| Manual: ground clearance 65 mm | 0.065 |

## 3. LiDAR height above ground (calibration route)

h_lidar = h_base_link + z(base_link → lidar) = 0.0635 + 0.3787 = **0.4422 m**

The range over the four base_link values is 0.4412–0.4442 m (−1.0 / +2.0 mm). The +0.040 m x offset and the identity rotation do not change the height on level ground.

Derived only for reference: the GPS receiver point (file 04) is then at 0.4422 + 0.153 = 0.595 m above the ground.

Assumptions behind 0.442 m. Each can matter more than the ±2 mm spread between sources.

- **Ground and tyres.** Flat, firm ground; tyres at nominal radius with no deflection under load (robot about 17 kg plus a sensor payload of several kg) and no sinkage into soil or grass.
- **Wheels.** The CitrusFarm Jackal uses stock wheels. Not stated by the dataset authors.
- **Attitude.** The robot is level (no pitch or roll).
- **LiDAR origin.** The CAD "lidar (Velodyne)" origin is the origin of the `/velodyne_points` coordinates (the VLP-16 driver frame). Not stated by the dataset authors.

## 4. Do the CitrusFarm paper or website state mounting heights?

**No.** The website (Calibration and Download pages at commit `0e58efe`) and the paper (arXiv:2309.15332v2; ISVC 2023, LNCS, pp. 571–582) give only the relative extrinsics: Table 3 / `extrinsic_param_summary.png`, identical to the two files above. They say that base_link is at the bottom centre of the robot base, and that the wheel-odometry and GPS extrinsics come from CAD. Neither gives any height of a sensor, or of base_link, above the ground.

Note: the brief and task refer to "arXiv 2309.15334". That identifier is a different paper (C3Net, Lee, Lim and Kim, 2023). The CitrusFarm preprint is **arXiv:2309.15332**, as linked from the dataset website.

## 5. Data-based estimate (RANSAC ground plane): placeholder

To do when the base bags are available (brief Section 5.1). For each sequence: fit a RANSAC plane to each of the first 100 `/velodyne_points` scans, take the distance from the LiDAR origin to the plane, and report the median, IQR and the inlier ratio per sequence.

| Seq | Scans used | Median height (m) | IQR (m) | Plane tilt (deg) | Notes |
|---|---|---|---|---|---|
| cf_2 | – | – | – | – | pending base bags |
| cf_3 | – | – | – | – | pending base bags |
| cf_4 | – | – | – | – | pending base bags |
| cf_5 | – | – | – | – | pending base bags |
| cf_6 | – | – | – | – | pending base bags |
| cf_7 | – | – | – | – | pending base bags |

Comparison with 0.442 m: pending.

## Open questions

1. **No `base_footprint` frame.** The Jackal description has no `base_footprint`, so the base_link height (0.0625–0.0655 m) is derived from the wheel geometry and the manual, not read from a joint. Nahuel should confirm that this derivation is acceptable for C4.
2. **Robot configuration.** Which `jackal_description` version, wheels and tyres the CitrusFarm robot used is unknown (stock assumed). Tyre deflection and soft orchard soil would lower the real height by an unknown amount. The RANSAC estimate should settle this.
3. **LiDAR origin.** Does the CAD "lidar (Velodyne)" origin coincide with the VLP-16 measurement origin (the `velodyne` frame of `/velodyne_points`)? This is not stated in the files or the paper.
4. **GPS reference point.** File 04's child is "the center of the GPS receiver". RTK positions refer to the antenna phase centre, and the SwiftNav Duro receiver and its antenna are separate units. Does the lever arm point to the antenna (reference point or phase centre) or to the receiver enclosure? Ask the dataset authors if centimetre-level lever-arm accuracy matters.
5. **arXiv identifier.** Correct the brief's arXiv identifier for Teng et al. (2023) to 2309.15332.

# Licences

This repository is distributed under two licences, chosen by the authors, plus the terms of the third-party material it contains.

## 1. The authors' code: GPL-3.0-or-later

[`LICENSE`](LICENSE). This covers:
- `scripts/`, `tools/`, `eval/` and `demo/`;
- `env/docker/` (the Dockerfiles and the frame logger `env/docker/infra/slambench_tools`);
- `env/collect_machine.py`, the `Makefile` and `datasets/download_citrusfarm.sh`.

GPL-3.0 was chosen because the SLAMesh configuration files (`configs/mesh/`) are derived from GPL-3.0 files, and GPL-3.0 can include the BSD- and MIT-licensed material listed in section 3. The frame logger in the published infrastructure image declares `MIT` in its `package.xml`, because the image was built before the licence was chosen. The authors, who hold the copyright, also grant that copy under the MIT licence.

## 2. The authors' data and documentation: CC BY-SA 4.0

[`LICENSE-DATA`](LICENSE-DATA), Creative Commons Attribution-ShareAlike 4.0 International. This covers:
- `gt/` (see section 3);
- `datasets/` (registry, file lists, height estimates);
- `demo/paper_cf_6.csv`;
- the documentation.

CC BY-SA is required for the processed CitrusFarm ground truth, which is adapted from a CC BY-SA 4.0 dataset.

## 3. Third-party material, which keeps its own terms

- **Configuration files** in `configs/<method>/<config>/` are copies or edited copies of upstream launch and parameter files:

  | Method | Licence |
  |---|---|
  | hdl Graph SLAM | BSD 2-Clause |
  | LeGO-LOAM | BSD 3-Clause |
  | A-LOAM | BSD 3-Clause (Carnegie Mellon University / Southwest Research Institute notice) |
  | DLO | MIT |
  | KISS-ICP | MIT |
  | SLAMesh | GPL-3.0 |

- **Patches** in `env/patches/` contain upstream source lines and keep the licence of the patched project.
- **Processed ground truth** in `gt/`:
  - `gt/cf_*` is adapted from CitrusFarm (H. Teng, Y. Wang, X. Song and K. Karydis, 2023; https://ucr-robotics.github.io/Citrus-Farm-Dataset/), CC BY-SA 4.0;
  - `gt/pl_1` is derived from Pullally run 1 (https://github.com/RAL-UC/Pullally_Dataset), MIT licence.

  Changes made: `gt/README.md`.

These licences are as stated in each upstream repository at the pinned commits (`env/src_pins_*.json`).

## 4. Docker images

The images `ghcr.io/nahueldeniz/lidar-slam-orchards/*` are not covered by this repository's licences. They contain:
- Ubuntu 18.04;
- ROS 1 Melodic;
- the upstream methods, each under its own licence (README, "Images and licences").

SLAMesh is GPL-3.0. Its image carries the complete source at the pinned commit (`/ws/src/SLAMesh`), and `env/docker/mesh/Dockerfile` rebuilds it.

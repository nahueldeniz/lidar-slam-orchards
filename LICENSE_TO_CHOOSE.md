# LICENCE NOT YET CHOSEN: PLACEHOLDER

**This repository has no licence yet.** Until the authors replace this file with a `LICENSE` file, no rights to reuse it are granted beyond what applicable law allows. Do not publish the repository with this placeholder in place.

**Proposed** (to be confirmed by the authors):

- **Code: GPL-3.0-or-later.** The SLAMesh configuration files (`configs/mesh/`) are derived from GPL-3.0 files, and GPL-3.0 can include the BSD- and MIT-licensed material listed below.
- **Data and results: CC BY-SA 4.0.** The processed CitrusFarm ground truth (`gt/cf_*`) is adapted from a CC BY-SA 4.0 dataset and must keep that licence.

What the choice has to cover:

1. **The authors' own code.** Covers `scripts/`, `tools/`, `eval/`, `demo/`, `env/docker/` (Dockerfiles and the frame logger), `env/collect_machine.py`, the `Makefile` and `datasets/download_citrusfarm.sh`. The frame logger's `env/docker/infra/slambench_tools/package.xml` has a `<license>` placeholder that must match the choice. Note: the published infrastructure image was built before that placeholder was introduced, so the `package.xml` inside it still says `MIT`.
2. **The authors' data and documentation.** Covers `gt/` (see 3), `datasets/` (registry, file lists, height estimates), `demo/paper_cf_6.csv` and the documentation.
3. **Files derived from third-party sources, which keep their own terms:**
   - **Configuration files** in `configs/<method>/<config>/`. These are copies or edited copies of upstream launch and parameter files:
     - hdl Graph SLAM: BSD 2-Clause;
     - LeGO-LOAM: BSD 3-Clause;
     - A-LOAM: BSD 3-Clause (Carnegie Mellon University / Southwest Research Institute notice);
     - DLO: MIT;
     - KISS-ICP: MIT;
     - SLAMesh: GPL-3.0.
   - **Patches** in `env/patches/`: these contain upstream source lines.
   - **Processed ground truth** in `gt/`:
     - `gt/cf_*` is adapted from CitrusFarm (Teng et al., 2023), CC BY-SA 4.0, and must stay under CC BY-SA 4.0;
     - `gt/pl_1` is derived from Pullally run 1, whose release is under the MIT licence;
     - see `gt/README.md`.

4. **The Docker images** (`ghcr.io/nahueldeniz/lidar-slam-orchards/*`) are not covered by this repository's licence. They contain Ubuntu 18.04, ROS 1 Melodic and the upstream methods, each under its own licence (README, "Images and licences"). SLAMesh is GPL-3.0: its image carries the complete source at the pinned commit (`/ws/src/SLAMesh`), and the Dockerfile `env/docker/mesh/Dockerfile` rebuilds it.

The licences listed in 3 and 4 are as stated in each upstream repository at the pinned commits (`env/src_pins_*.json`), and were checked in the `LICENSE` files and `package.xml` files inside the images. Check them again before publishing.

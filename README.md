# LiDAR SLAM in orchards: six methods, one command

Benchmark code, frozen configurations and a ready-to-run demonstration for the article

> N. Deniz, P. Nazate-Burgos, L. Fu, M. Torres-Torriti and F. Auat Cheein, "Comparing six LiDAR-based simultaneous localisation and mapping approaches on public orchard datasets to assess the feasibility of GNSS-free autonomous machinery", *Precision Agriculture* (under revision; the link will be added on publication).

Six open-source LiDAR SLAM methods (hdl Graph SLAM, LeGO-LOAM, A-LOAM, DLO, KISS-ICP and SLAMesh) were run with a scripted protocol on two orchard datasets (CitrusFarm and Pullally), using LiDAR data only, in ROS 1 Melodic with one Docker image per method. This repository holds the code, the configurations and a one-command demo that runs all six methods on one CitrusFarm sequence and scores them as in the paper.

The recorded outputs of all 156 runs of the paper (trajectories, manifests, resource and frame logs), the aggregated results, figures and tables are in the Zenodo package, **https://doi.org/10.5281/zenodo.23405930**. They are not in this repository.

Contents:

1. [Quickstart](#1-quickstart)
2. [What the demo does, its output and its run time](#2-what-the-demo-does-its-output-and-its-run-time)
3. [Demo options](#3-demo-options)
4. [Run on your own bag](#4-run-on-your-own-bag)
5. [Images and licences](#5-images-and-licences)
6. [Rebuild the images from the Dockerfiles](#6-rebuild-the-images-from-the-dockerfiles)
7. [Regenerate the paper's results from the Zenodo package](#7-regenerate-the-papers-results-from-the-zenodo-package)
8. [Repository layout](#8-repository-layout)
9. [Data, licences and citation](#9-data-licences-and-citation)
10. [Troubleshooting](#10-troubleshooting)

**Command status.** Every command below was run while preparing this repository (Ubuntu 22.04, Docker 29.7.2, Python 3.10.12, uv 0.11.16), unless it is marked **[untested]**. Downloads from the internet (`git clone` from GitHub, `docker pull` from ghcr.io, the CitrusFarm and Zenodo downloads) could not be tested before publication: the demo was tested from a local clone, with the published images present as local tags of the same builds, and with the official CitrusFarm file already downloaded (its MD5 checked by the demo).

---

## 1. Quickstart

You need a Linux x86-64 machine with:

- Docker Engine, usable by your user without `sudo` ([post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/));
- `git`, GNU `make`, `curl` and [uv](https://docs.astral.sh/uv/) (for the Python evaluation environment);
- about 12 GB of free disk for the Docker images, 2.5 GB for the demo bag and 0.1 GB for the outputs;
- 4 or more CPU cores and 2 GB or more of free memory (the demo was tested on 16 logical CPUs);
- port 11311 free (no ROS 1 master running).

```bash
git clone https://github.com/nahueldeniz/lidar-slam-orchards.git   # [untested] until published
cd lidar-slam-orchards
make venv          # Python environment .venv from env/requirements-eval.lock.txt (uv)
make demo-check    # checks Docker, disk and port; gets the images and the demo bag; no runs
make demo          # the six methods on CitrusFarm 06, then evaluation and report (about 40 min)
```

`make demo-check` pulls the seven images (about 1.3 GB to download; **[untested]** until they are published) and downloads the CitrusFarm bag (2.49 GB, resumable; the download from the dataset's server is **[untested]**, the resume and MD5 logic was tested on a local copy). You can skip it: `make demo` does the same checks first.

A two-minute check that everything works, with one method and the first 60 s of the bag (not comparable with the paper):

```bash
make demo METHODS=kiss DURATION=60
```

The results are in `demo_output/results.md`.

---

## 2. What the demo does, its output and its run time

`make demo` (the same as `./demo/run_demo.sh`) runs these steps:

1. **Prerequisites:** Docker reachable without `sudo`, port 11311 free, free disk, the Python environment.
2. **Images:** for each image in `demo/images.conf` it uses the registry name if present, else the local build tag (`slambench/...`, from `make images`), else it pulls the registry name.
3. **Data:** the original CitrusFarm bag of sequence 06 (`06_14B_Jackal/base_2023-07-18-14-29-35_0.bag`, 302 s, 2,486,606,484 bytes) from the dataset's official server, into `demo_data/`. The download resumes if interrupted (run the demo again), and the file is checked against the official MD5 (`demo/config.sh`).
4. **Runs:** each method once, one after the other, with `scripts/run_one.py --demo`: a fresh `roscore` with simulated time, the method container (configuration D of the paper, checked against `configs/FROZEN.json`), a recorder and a frame logger, then `rosbag play` of the LiDAR topic `/velodyne_points` only, at real-time rate, exactly as in the paper's runs. After playback the runner waits until the output has been quiet for 10 s (at most 60 s); for hdl Graph SLAM it then calls the keyframe dump service. The demo profile leaves out what was specific to the benchmark laptop: CPU pinning, the idle-machine, power-setting and data-drive checks, the RAPL power logger and the staging copy (`scripts/run_one.py`, docstring).
5. **Evaluation:** `scripts/evaluate_runs.py` with the paper's code and settings (`eval/slameval`, `eval/config/eval.yaml`): association by header stamps, rigid alignment over the whole run (alignment A), 3D position error, LiDAR-to-antenna lever arm compensated, headline output of each method (`eval/config/outputs.yaml`). LeGO-LOAM's rotated output frames are converted as declared in `configs/methods.yaml` (`eval_outputs`).
6. **Report:** `demo/report.py` writes the files below and compares each method with the paper's three runs (`demo/paper_cf_6.csv`, written by `demo/make_paper_reference.py` from the Zenodo package). A run whose playback fell more than 0.5 s behind the bag (busy or frozen machine) is marked "playback fault" and not compared.

Output, in `demo_output/` (a previous `demo_output/` is moved aside to `demo_output.<date-time>`, never overwritten):

| File | Content |
|---|---|
| `results.md` | summary: machine, settings, table of demo vs paper, figure, how to read the numbers |
| `results.csv` | the same per method, with run status, coverage, processed-scan ratio, latency, CPU, memory and run time |
| `trajectories_cf_6.png`, `.pdf` | top view, one panel per method, aligned trajectory over the ground truth |
| `runs/<method>/cf_6/D/rep1/` | everything a run records: `manifest.json`, `outputs.bag`, `<output>.tum`, `frames.csv`, `resources.csv`, logs |
| `eval/` | the evaluation tables of the paper's format (`results_long.csv`, `segments.csv`, `heading.csv`, `run_diagnostics.csv`, `resources_summary.csv`) |
| `logs/` | the console output of each step |

**Run time.** About 5.7 min per method (302 s of playback and about 40 s to start and stop the containers), and about 10 s for the evaluation and the report: 34 min 53 s for the six methods on the benchmark laptop. The first run also pulls the images (about 1.3 GB) and downloads the bag (2.49 GB).

**Expected results.** The paper's values and three full demo runs on the benchmark laptop, each from a clean clone of this repository while the laptop was in normal desktop use: runs 1 and 2 without CPU pinning, run 3 pinned as in the paper (`CPUSET_METHOD=0-11 CPUSET_INFRA=12-15`). Configuration D, alignment A, 3D RMSE in m, lever arm compensated; processed-scan ratio of run 3:

| Method | Paper: mean ± SD | Paper: the 3 runs | Demo run 1 | Demo run 2 | Demo run 3 (pinned) | Processed-scan ratio |
|---|---|---|---|---|---|---|
| LeGO-LOAM | 0.471 ± 0.000 | 0.471, 0.471, 0.471 | 0.480 | 0.441 | 0.368 | 0.999 |
| DLO | 0.582 ± 0.074 | 0.632, 0.617, 0.497 | 1.076 | 0.663 | 0.765 | 0.999 |
| SLAMesh | 0.590 ± 0.150 | 0.696, 0.418, 0.655 | 0.585 | 0.661 | 0.522 | 1.000 |
| A-LOAM | 1.162 ± 0.043 | 1.114, 1.172, 1.199 | 1.151 | 1.226 | 1.096 | 1.000 |
| hdl Graph SLAM | 4.351 ± 0.000 | 4.351, 4.351, 4.351 | 4.352 | 4.351 | 4.351 | 1.000 |
| KISS-ICP | 4.849 ± 6.970 | 12.881, 1.270, 0.395 | 11.198 | 0.643 | 0.887 | 1.000 |

On a laptop in normal use, LeGO-LOAM and DLO vary between runs more than in the paper, whose runs were made on an idle machine: LeGO-LOAM gave 0.37 to 0.48 m (paper 0.471 m) and DLO 0.42 to 1.08 m in the demo runs and in two further DLO checks (paper 0.50 to 0.63 m). Under real-time playback their results depend on how the scans are scheduled on a loaded machine, and DLO's NanoGICP registration uses one OpenMP thread per CPU of the container, so its result also depends on the CPU pinning. For the closest comparison with the paper, run the demo on an otherwise idle machine and, on a similar hybrid CPU, pin it as in the paper: `make demo CPUSET_METHOD=0-11 CPUSET_INFRA=12-15`.

Real-time results depend on the machine: the bag is played at its recorded rate, and a slower or busier machine makes a method skip or delay scans, which can change its trajectory (the processed-scan ratio in `results.md` shows this). Several methods also vary from run to run on the same machine; in the paper KISS-ICP gave 0.40, 1.27 and 12.88 m on this sequence. Differences within the paper's run-to-run spread are expected. The demo is one run on one sequence: it shows that the pipeline works on your machine, not a re-evaluation of the paper.

---

## 3. Demo options

Options are `make` variables (or environment variables for `./demo/run_demo.sh`); the defaults are in `demo/config.sh`.

| Variable | Default | Meaning |
|---|---|---|
| `METHODS` | `hdl lego aloam dlo kiss mesh` | methods to run, in this order |
| `DURATION` | empty (whole bag) | play only the first N seconds (quick check; not comparable with the paper) |
| `OUT` | `demo_output` | output folder |
| `DEMO_DATA_DIR` | `demo_data` | folder for the demo bag (an existing copy there is used, after its MD5 check) |
| `SKIP_CHECKSUM` | empty | `1` skips the MD5 check of an existing demo bag |
| `BAG`, `SEQ` | empty, `cf_6` | your own bag and its sequence entry (Section 4) |
| `CPUSET_METHOD`, `CPUSET_INFRA` | empty (no pinning) | pin the method and infrastructure containers (`docker --cpuset-cpus`), e.g. `0-11` and `12-15` as in the paper |

```bash
make demo METHODS="lego dlo"                                # a subset
make demo DEMO_DATA_DIR=/data/citrusfarm OUT=/tmp/demo1     # bag already downloaded elsewhere
make demo METHODS=dlo CPUSET_METHOD=0-11 CPUSET_INFRA=12-15 # pinned as in the paper
./demo/run_demo.sh --check                                  # = make demo-check
```

**CPU pinning.** The paper pinned the method container to the 12 logical CPUs of the performance cores of its Intel i7-13620H and the infrastructure container to the 4 efficiency cores. The demo does not pin by default, because CPU numbers differ between machines. On a machine like the paper's, pinning reproduces its set-up. It matters for DLO, whose NanoGICP registration uses one OpenMP thread per CPU of the container and sums in an order that depends on the thread count (`configs/dlo/source_verification.md`).

---

## 4. Run on your own bag

The methods run with configuration D, which was set for the Velodyne VLP-16 (10 Hz) of the CitrusFarm Jackal. Another sensor may need other settings (`configs/<method>/D/`); LeGO-LOAM and A-LOAM have their sensor parameters compiled in (`configs/<method>/D/constants.env`), so they need a rebuilt image (Section 6).

1. Add an entry for your recording to `datasets/sequences.yaml` (copy the `cf_6` block). For example:

   ```yaml
     my_orchard:
       dataset: own
       role: own
       lidar_topic: /velodyne_points      # sensor_msgs/PointCloud2 topic in your bag
       bags: []                           # the file is given with BAG=
       gt_nominal_period_s: 0.1           # ground-truth period, if you have ground truth
       lever_arm_lidar_to_antenna_m: null # [x, y, z] in the LiDAR frame, if known
       lidar_forward_axis: [1, 0, 0]
       sensor:                            # hdl Graph SLAM floor detection and frame
         sensor_height: 0.43              # LiDAR height above the ground (m)
         tilt_deg: 0
         lidar_frame_id: velodyne         # header.frame_id of the LiDAR messages
         scan_period: 0.1
   ```

2. Optional: ground truth as `gt/my_orchard/gt_tum.txt`, one line per sample, `t x y z qx qy qz qw`, with `t` on the same clock as the LiDAR header stamps and positions in a local metric frame (for example East-North-Up). Without it, the trajectories are written (`runs/<method>/my_orchard/D/rep1/<output>.tum`) but not scored.

3. Run:

   ```bash
   make demo SEQ=my_orchard BAG=/path/to/recording.bag METHODS="kiss dlo"
   ```

This was tested with an entry like the one above, without ground truth, on the CitrusFarm bag (`METHODS=kiss DURATION=20`). A recording split into several bags can be run with `scripts/run_one.py --demo --bag first.bag --bag second.bag ...` (one `--bag` per file, in chronological order), then `scripts/evaluate_runs.py --runs <out-root>`; **[untested]** with more than one file.

---

## 5. Images and licences

The images are published in the GitHub container registry. `demo/images.conf` is the one place that names them:

| Key | Image | Size (`docker images`) | Built from |
|---|---|---|---|
| `infra` | `ghcr.io/nahueldeniz/lidar-slam-orchards/infra:melodic` | 3.29 GB | `env/docker/infra/Dockerfile`: roscore, `rosbag play/record`, frame logger |
| `hdl` | `ghcr.io/nahueldeniz/lidar-slam-orchards/hdl:melodic` | 3.37 GB | `env/docker/hdl/Dockerfile` |
| `lego` | `ghcr.io/nahueldeniz/lidar-slam-orchards/lego:D` | 3.42 GB | `env/docker/lego/Dockerfile`, constants of `configs/lego/D/constants.env` |
| `aloam` | `ghcr.io/nahueldeniz/lidar-slam-orchards/aloam:D` | 3.48 GB | `env/docker/aloam/Dockerfile`, constants of `configs/aloam/D/constants.env` |
| `dlo` | `ghcr.io/nahueldeniz/lidar-slam-orchards/dlo:melodic` | 3.51 GB | `env/docker/dlo/Dockerfile` |
| `kiss` | `ghcr.io/nahueldeniz/lidar-slam-orchards/kiss:melodic` | 3.68 GB | `env/docker/kiss/Dockerfile` |
| `mesh` | `ghcr.io/nahueldeniz/lidar-slam-orchards/mesh:melodic` | 3.81 GB | `env/docker/mesh/Dockerfile` |
| `base` | `ghcr.io/nahueldeniz/lidar-slam-orchards/base:melodic` | 3.29 GB | `env/docker/base/Dockerfile` (shared base; not needed by the demo) |

All images share the 3.3 GB base (`ros:melodic-perception-bionic` plus build tools), which `docker images` counts in every line. Together the seven demo images take about 4.7 GB on disk and about 1.3 GB to download. They are the builds used for the paper's runs: their image IDs are those recorded in `env/images.json`, and every run manifest records the ID it ran with. LeGO-LOAM and A-LOAM read their parameters at compile time, so there is one image per configuration; only configuration D is published. The run checks that the image label `slambench.constants_sha256` matches `configs/<method>/D/constants.env`.

Sources are pinned: each Dockerfile fetches the upstream repositories at the commits in its `ARG` lines and checks them, the pins and their reasons are in `env/src_pins_*.json`, and the few build-fix and logging-only patches are in `env/patches/` (each with a one-line reason).

**Licences of the code inside the images.** The images are not covered by this repository's licence; each component keeps its own. As stated in the `LICENSE` and `package.xml` files inside the images:

| Image | Method (pinned commit) | Licence | Other components built from source |
|---|---|---|---|
| `hdl` | hdl_graph_slam `95b8dce` | BSD 2-Clause | ndt_omp `6f4e5f3` (BSD 2-Clause), fast_gicp `0e7ec14` (BSD 3-Clause); g2o from the ROS apt package (`package.xml`: BSD; some optional g2o parts are LGPL) |
| `lego` | LeGO-LOAM `896a7a9` | BSD 3-Clause | GTSAM 4.0.0-alpha2 (BSD, upstream licence file; not checked inside the image) |
| `aloam` | A-LOAM `e51f88c` | BSD 3-Clause (Carnegie Mellon University / Southwest Research Institute notice) | Ceres Solver 1.12.0 (BSD 3-Clause, upstream) |
| `dlo` | direct_lidar_odometry `11528c0` | MIT | none |
| `kiss` | KISS-ICP v0.3.0 `460c6f2` | MIT | Sophus 1.22.11 (MIT), oneTBB 2021.8.0 (Apache 2.0), robin-map 1.2.1 (MIT); CMake 3.25.3 binary (BSD 3-Clause, build tool) |
| `mesh` | SLAMesh `50b4356` | **GPL-3.0** | mesh_tools `95433f1` (BSD 3-Clause), Ceres Solver 2.1.0 (BSD 3-Clause, upstream); lvr2 from the ROS apt package (BSD 3-Clause) |
| `infra` | frame logger `slambench_tools` (the authors' code) | GPL-3.0-or-later; the `package.xml` inside the published image says MIT, which the authors also grant for that copy (`LICENSES.md`) | none |

Every image also contains Ubuntu 18.04 and ROS 1 Melodic packages (ROS mostly BSD 3-Clause; PCL BSD 3-Clause; Ubuntu packages under many licences, including the GPL), whose terms are in `/usr/share/doc/*/copyright` inside the image. SLAMesh is distributed under the GPL-3.0: its complete source at the pinned commit is in the image (`/ws/src/SLAMesh`), and `env/docker/mesh/Dockerfile` rebuilds it.

---

## 6. Rebuild the images from the Dockerfiles

**[untested]** here (the published images are the original builds). This needs Docker, network access to GitHub and the Ubuntu and ROS package servers, and about 15 GB of free disk.

```bash
make images        # base, infrastructure and the six methods (one image per compile-time configuration)
# or step by step:
docker build -t slambench/base:melodic env/docker/base
docker build -t slambench/infra:melodic -f env/docker/infra/Dockerfile env
.venv/bin/python scripts/build_images.py kiss          # one method; logs in env/build_logs/
.venv/bin/python scripts/build_images.py lego --configs D
```

The demo uses the local build tags (`slambench/...`, third column of `demo/images.conf`) when the registry names are not present, so `make demo` runs on your own builds without retagging. Rebuilding is not guaranteed to give bit-identical images: the base image is referenced by tag (the digest used, `sha256:3b7f84af20714319a212c825025e2e42da1a4a2c387feed2db5188e8bab57fd9`, is in the Zenodo package's `env/build_logs/base.log`), and Ubuntu and ROS apt packages are only partly pinned. To build one method without building the base, pull the published base and tag it: `docker pull ghcr.io/nahueldeniz/lidar-slam-orchards/base:melodic && docker tag ghcr.io/nahueldeniz/lidar-slam-orchards/base:melodic slambench/base:melodic` **[untested]**.

---

## 7. Regenerate the paper's results from the Zenodo package

The paper's numbers, figures and tables come from the recorded run outputs, which are in the Zenodo package (DOI 10.5281/zenodo.23405930, about 180 MB zipped). The package contains the same code as this repository (without `demo/`) and its own README. Download and unpack it **[untested]**, then, in the package folder:

```bash
make venv                             # .venv from env/requirements-eval.lock.txt
make test                             # 29 unit and pipeline tests
make results OUT=/tmp/regen           # eval CSVs, figures F0-F11 and tables T1-T7 into /tmp/regen
make check-results OUT=/tmp/regen     # compare with the shipped eval/ and tables/
```

ZENODO_TEST_NOTE

To repeat the measured campaign itself (156 runs, about two days of machine time, the full datasets and the benchmark laptop's settings), see Sections 4 to 6 of the package README (`make batch1`, `make batch2`; **[untested]** here).

---

## 8. Repository layout

```
Makefile               entry point: venv, demo, demo-check, test; campaign targets (images, batch1, ...)
demo/                  run_demo.sh (the demo), config.sh (settings), images.conf (image names),
                       report.py, paper_cf_6.csv and make_paper_reference.py (paper values),
                       push_images.sh (maintainers: publish the images)
scripts/               run_one.py (one run; --demo profile), evaluate_runs.py, make_figures.py,
                       make_tables.py, build_images.py, campaign scripts (run_batch.py, ...)
tools/                 host-side resource, power and host-state loggers
eval/slameval/         evaluation package; eval/tests/ unit and pipeline tests; eval/config/ settings
configs/               frozen configurations (D, the hdl floor ablation and the 48 sensitivity levels),
                       FROZEN.json (their SHA-256), methods.yaml (method registry), source audits
datasets/              sequence registry (sequences.yaml), CitrusFarm file list and download script
gt/                    processed ground truth of the six sequences (gt_tum.txt, gt_quality.json)
env/                   Dockerfiles (env/docker/), patches, source pins, lock file, run settings,
                       images.json (image IDs of the campaign)
schedules/             run order of the paper's two batches
```

Not in this repository (all in the Zenodo package): run outputs (`runs/`, `baselines/`), aggregated results (`eval/*.csv`), figures, tables, campaign logs, build logs and the benchmark laptop's description (`env/machine.json`).

---

## 9. Data, licences and citation

**Licences of this repository.** The authors' code is under GPL-3.0-or-later (`LICENSE`; the SLAMesh configuration files derive from GPL-3.0 files), and the authors' data and documentation are under CC BY-SA 4.0 (`LICENSE-DATA`; the processed CitrusFarm ground truth must keep that licence). Configuration files, patches and ground truth derived from third-party sources keep their own terms, listed in `LICENSES.md`.

**CitrusFarm.** The demo uses the CitrusFarm dataset: H. Teng, Y. Wang, X. Song and K. Karydis, "Multimodal Dataset for Localization, Mapping and Crop Monitoring in Citrus Tree Farms", *Advances in Visual Computing (ISVC 2023)*, https://doi.org/10.1007/978-3-031-47969-4_44; website https://ucr-robotics.github.io/Citrus-Farm-Dataset/; licence CC BY-SA 4.0 (https://creativecommons.org/licenses/by-sa/4.0/). The demo downloads the original bag from the dataset's own server; this repository redistributes no bag. The processed ground truth in `gt/cf_*` is adapted from CitrusFarm and shared under CC BY-SA 4.0; the changes are listed in `gt/README.md`.

**Pullally.** `gt/pl_1` is derived from Pullally run 1 (Nazate-Burgos, Torres-Torriti, Huang and Auat Cheein, 2026, *Computers and Electronics in Agriculture* 247, 111687, https://doi.org/10.1016/j.compag.2026.111687; MIT licence); see `gt/README.md`.

**How to cite:** the article above and the Zenodo package (`CITATION.cff`).

---

## 10. Troubleshooting

- **"cannot reach the Docker daemon":** start Docker and add your user to the `docker` group (link in Section 1); the demo never uses `sudo`.
- **"port 11311 is in use":** a ROS 1 master is running (for example a `roscore` of your own); stop it. The containers use `--network host`.
- **A ROS 2 (or other) `PYTHONPATH`** in your shell does not matter: the Makefile and `demo/run_demo.sh` unset it.
- **A method falls behind real time:** its processed-scan ratio in `results.md` is below 1. Close other programs and run it again, for example `make demo METHODS=mesh`.
- **"playback fault" in `results.md`:** the bag was not replayed in real time; playback fell more than 0.5 s behind the bag timeline (the paper's criterion for setting a run aside, `max_playback_lag_s` in `env/run_settings.yaml`). The value is not compared with the paper. This happened once while the demo was being tested: the laptop froze for 8 minutes under memory pressure from other programs (swap full), and playback fell 519 s behind. Free memory and run the method again.
- **The demo was interrupted:** run it again. The bag download resumes, the containers left over (`sb_infra`, `sb_method`) are removed, and the previous output folder is moved aside.
- **A run failed:** `demo_output/logs/run_<method>.log`, and in `demo_output/runs/<method>/cf_6/D/rep1/`: `manifest.json` (`status`, `error`), `console.log` (method output) and `infra.log`.

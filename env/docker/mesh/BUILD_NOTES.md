# SLAMesh build feasibility on ROS Melodic (not built yet)

Target: `slambench/base:melodic` = `ros:melodic-perception-bionic` (Ubuntu 18.04, CMake 3.10.2, GCC 7.5, PCL 1.8.1, Eigen 3.3.4, Boost 1.65). Source: SLAMesh @ `50b4356`.

## Verdict: own build on Melodic, with no patches. Do not use the official image for measured runs.

SLAMesh's README says it was tested on Ubuntu 18.04 with Melodic, and its own `docker/Dockerfile` is `FROM osrf/ros:melodic-desktop-full`. The code is C++14 (`CMakeLists.txt:4`). Dependencies and how they are pinned:

| Dependency | Upstream recipe (`docker/Dockerfile`, README 2.1) | Problem with the recipe today | Pinned choice |
|---|---|---|---|
| Ceres Solver | `git clone -b 2.1.0`, `cmake .. && make install` (glog, gflags, ATLAS) | none (tag) | Tag 2.1.0 = `f68321e7…`, Release, static (upstream default), tests/examples/benchmarks off. Needs CMake ≥ 3.10 and Eigen ≥ 3.3, both OK. The build fails if LAPACK is not found, because SLAMesh uses `DENSE_QR` with `LAPACK` (`map.cpp:972-973`). |
| lvr2 | `git clone https://github.com/uos/lvr2.git` (default branch, **unpinned**), built from source | Default branch is now `main` (25.2.x, `cmake_minimum_required(3.16)`), so it **cannot configure with CMake 3.10.2** | `ros-melodic-lvr2=20.11.3-1bionic.20211011.173739`: the frozen Melodic binary of tag 20.11.3 (`9bb03a3`, = branch `master`). It provides `LVR2Config.cmake` version 2, which satisfies `find_package(LVR2 2 REQUIRED)`, and its apt dependencies cover the README's list (VTK 6, OpenCV 3.2, HDF5, Qt5, GDAL, FLANN, lz4). |
| mesh_tools | `git clone -b noetic` (unpinned), `rosdep install`, `catkin_make` of all packages | builds the RViz plugin and other tools that SLAMesh does not need | Branch `noetic` @ `95433f1`; only `mesh_msgs` and `mesh_msgs_conversions` are built. Their only differences from release 1.1.0, which is also packaged for Melodic, are a "fix no return types" commit and a CMake minimum-version change. |
| PCL, Eigen, OpenCV, Boost | from ROS | none | base image versions (PCL 1.8.1, Eigen 3.3.4) |
| MPI | via lvr2 deps | `mesh_msgs_conversions` calls `find_package(MPI REQUIRED)` | `libopenmpi-dev=2.1.1-8` |

Other build observations:

- `CMakeLists.txt:6` sets `catkin_DIR` to `/opt/ros/kinetic/...`, which does not exist. Upstream builds with `catkin_make`, where catkin has already been found; the Dockerfile does the same. If configuration fails on this line, a one-line build-fix patch (delete it) is justified.
- `pcl/visualization/pcl_visualizer.h` is included (`src/cell.h:43`), so PCL's VTK 6 headers are needed. They come with `libpcl-dev` on 18.04, and the README warns against VTK 7 on Melodic.
- No C++17 or PCL ≥ 1.10 API was found in SLAMesh's sources.
- Peak RAM: Ceres and SLAMesh are built with `-j4` for the 16 GB laptop.
- Disk: lvr2's binary dependencies (Qt5, VTK 6 Qt, GDAL, HDF5, OpenMPI) are the largest addition, roughly 1–1.5 GB over the base (estimate, not measured).

## Official image `pleaserun/rjy_slam_work:slamesh_18.04`: reference only

Docker Hub API and registry metadata only; nothing was pulled.

- Digest `sha256:33832e01972f12f894f2766f393cc387ed04ac1dcbbd3a2eba52218c5406ee27`, 11 layers, **2.32 GB compressed** (2,323,259,706 bytes), last pushed 2023-08-14T07:12:51Z.
- Provenance: a third-party "ROS Melodic desktop-full + XFCE + VNC + SSH + Firefox" image built 2020-04-15, plus **one `docker commit` layer of about 1.15 GB with no recorded commands**. Its entrypoint starts a VNC desktop (`./startup.sh`).
- Following the README, SLAMesh itself is cloned and built inside the container by the user. The image therefore does not identify the SLAMesh commit used in 2024.

Reasons for the own build: (1) the brief requires every method image to share `slambench/base:melodic`; (2) the official image cannot be audited or pinned below the image level, and carries a desktop and VNC stack that would add background load during measured runs; (3) its size (2.3 GB compressed, several GB unpacked) is a large share of the ~14 GB of free disk; (4) every component of the own build is pinned and documented. The official image could still serve as a cross-check, if disk allows, by building SLAMesh @ `50b4356` inside it and comparing one run. That would need Nahuel's approval because of the disk use.

## Noetic

Not needed: everything above exists for Melodic. As written, the upstream recipe fails on 18.04 only because lvr2 is unpinned; pinning lvr2 to 20.11.3 resolves it.

## Open questions

1. The same ROS apt-key concern as for DLO (`env/docker/dlo/BUILD_NOTES.md`) applies, because `ros-melodic-lvr2` comes from packages.ros.org.
2. lvr2 as the Melodic binary (current choice) or built from source at tag 20.11.3? The binary is the same release built by the ROS build farm; a source build would add 30+ minutes and needs the same dependencies.
3. Which BLAS/LAPACK backs Ceres (ATLAS, as upstream, or another provider already in the base) can change the last bits of the results. The Dockerfile records the `liblapack.so.3` alternative in `/ws/build_versions.txt`.

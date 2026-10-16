# DLO build feasibility on ROS Melodic (not built yet)

Target: `slambench/base:melodic` = `ros:melodic-perception-bionic` (Ubuntu 18.04, CMake 3.10.2, GCC 7.5, PCL 1.8.1, Eigen 3.3.4, Boost 1.65). Source: `direct_lidar_odometry` @ `11528c0`.

## Verdict: build on Melodic with one build-fix patch. Noetic is not needed.

The README lists "CMake >= 3.16.3, PCL >= 1.10.0, Eigen >= 3.3.7". These are the versions on Ubuntu 20.04, where the authors tested. The same README says DLO was "tested extensively on both Ubuntu 18.04 Bionic with ROS Melodic and Ubuntu 20.04 Focal with ROS Noetic". Checked item by item:

| Requirement | What the code actually uses | Available on Melodic? |
|---|---|---|
| CMake ≥ 3.16.3 | `cmake_minimum_required(VERSION 3.10.0)` (`CMakeLists.txt:10`). The only newer command is `add_compile_definitions(HAS_CPUID)` (line 77, added by the "Patch for ARM architecture" commit of 2022-06-30), which needs **CMake ≥ 3.12**. On 3.10.2 it stops configuration ("Unknown CMake command"). | **No → patch 0001** replaces it with `add_definitions(-DHAS_CPUID)`, which gives the same macro on the same targets. The macro only enables printing the CPU brand string. |
| PCL ≥ 1.10 | `CropBox`, `VoxelGrid`, `removeNaNFromPointCloud`, `ConvexHull`/`ConcaveHull` with `setKeepInformation` and `getHullPointIndices`, `pcl::Registration` with `KdTreeReciprocalPtr`, `setSearchMethodSource/Target(…, true)`, and virtual `setInputSource/Target`/`computeTransformation` overrides. All are present in PCL 1.8.1 (checked against the `pcl-1.8.1` headers). The code uses `boost::make_shared` for PCL pointers, which is right for PCL < 1.11. No `pcl::make_shared`, `pcl::shared_ptr`, `pcl::index_t`, `std::filesystem` or C++17. | Yes |
| C++ standard | `-std=c++14` (`CMakeLists.txt:13`) | Yes (GCC 7.5) |
| OpenMP ≥ 4.5 | `#pragma omp parallel for … reduction … schedule(guided, 8)` | Yes (GCC 7 libgomp, OpenMP 4.5) |
| Eigen ≥ 3.3.7 | Standard dense Eigen only | Yes (3.3.4); no 3.3.5+ feature found |
| ROS deps | `roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `pcl_ros`, `message_generation` (`tf2_ros` comes through `pcl_ros`) | Yes (base image) |

External evidence: DLO issue #20 (July 2022, after the ARM patch) shows a Melodic user (PCL 1.8, Boost 1.65.1) compiling all sources and failing only at link time, because of a conda Boost/ICU conflict. That user's CMake was evidently ≥ 3.12.

Remaining risk: the code has not been compiled here (no image builds allowed). If compilation fails on Melodic for an unforeseen reason, add a further build-fix patch, or as a last resort fall back to `ros:noetic-perception-focal`, where DLO builds unpatched. Record the reason if that happens.

One behavioural note, the same on Melodic and Noetic: PCL's `VoxelGrid` (1.8.1 and 1.10) returns the cloud **unfiltered** if the leaf is too small for a 32-bit voxel index (about 0.1 m for a full-range VLP-16 scan). The D and T levels stay at or above 0.15 m.

## Why not Noetic

- Melodic is the paper's platform (brief Section 3), and the brief allows Noetic only if a method cannot be built on Melodic.
- One CMake-only patch is enough; no source change is needed.
- Installing Kitware CMake or building PCL 1.10 from source would add risk and image size for no gain.

## Image contents (Dockerfile)

`FROM slambench/base:melodic`; pinned `libpcl-dev=1.8.1+dfsg1-2ubuntu2.18.04.1` and `libeigen3-dev=3.3.4-4` (already in the base; the pin fails loudly if they differ); depth-1 fetch of commit `11528c0…`; patches 0001 (build) and 0002 (logging); `catkin build` in Release in `/ws`; versions written to `/ws/build_versions.txt`. No RViz. Expected image growth is small (about 50–100 MB over the base).

## Open questions

1. The base image and the ROS apt key. The ROS signing key `F42ED6FBAB17C654` published in `ros/rosdistro/master/ros.key` has a self-signature dated 2025-04-30 that sets its expiry to 2030-06-01. A `ros:melodic` image built before then may hold an older copy of the key with an earlier expiry. If `apt-get update` then fails with EXPKEYSIG for packages.ros.org, the base Dockerfile (owned by another agent) must refresh the key. This Dockerfile's `apt-get` step uses the same sources.

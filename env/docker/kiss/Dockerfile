# KISS-ICP (paper ID `kiss`) v0.3.0, ROS 1 wrapper, on the shared ROS 1 Melodic base.
# Build context: $R/env   e.g.  docker build -f docker/kiss/Dockerfile -t slambench/kiss:melodic .
# Pins and their justification: env/src_pins_hdl_kiss.json. Not built yet (disk budget); see open questions there.
#
# Why this image deviates from the base toolchain (Ubuntu 18.04: CMake 3.10.2, GCC 7.5):
#   * CMake >= 3.25: ros/CMakeLists.txt and cpp/kiss_icp/CMakeLists.txt require 3.16, and
#     3rdparty/sophus/sophus.cmake and 3rdparty/tsl_robin/tsl_robin.cmake use FetchContent_Declare(... SYSTEM ...),
#     a CMake 3.25 keyword (upstream CI also installs CMake 3.25.x for its ROS 1 Noetic job).
#   * GCC >= 8: the ROS 1 node is compiled with cxx_std_20 (ros/CMakeLists.txt:60) and uses a C++20
#     template-parameter lambda (ros/ros1/Utils.hpp, ExtractTimestampsFromMsg); the core library needs C++17.
#     g++-8 comes from the official Ubuntu 18.04 archive (bionic-updates/universe) and uses the system
#     libstdc++6 8.4.0 that Melodic already runs on, so no PPA and no runtime-library upgrade is needed.
#   * No source patches are needed.
FROM slambench/base:melodic

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ARG KISS_SHA=460c6f2cc3616c122c42006898036df9c20b636f
ARG CMAKE_VERSION=3.25.3
ARG CMAKE_SHA256=d4d2ba83301b215857d3b6590cd4434a414fa151c5807693abe587bd6c03581e

# GCC 8 toolchain from the Ubuntu archive (pinned build), and curl for the tarball downloads.
RUN apt-get update && apt-get install -y --no-install-recommends \
        gcc-8=8.4.0-1ubuntu1~18.04 g++-8=8.4.0-1ubuntu1~18.04 curl \
    && rm -rf /var/lib/apt/lists/*

# Kitware CMake binary release, checksum-verified; used only inside this image's build step.
RUN set -e; cd /tmp \
    && curl -fsSL -o cmake.tgz "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz" \
    && echo "${CMAKE_SHA256}  cmake.tgz" | sha256sum -c - \
    && mkdir -p /opt/cmake && tar -xzf cmake.tgz -C /opt/cmake --strip-components=1 && rm cmake.tgz

# Third-party sources that KISS-ICP v0.3.0 would otherwise download with FetchContent at configure time.
# The URLs are exactly those in cpp/kiss_icp/3rdparty/*/*.cmake; the SHA-256 sums were computed on 2026-10-12.
# Eigen is NOT fetched: the system Eigen 3.3.4 (shared with every other method) is used, which is upstream's
# default (USE_SYSTEM_EIGEN3=ON) and what its ROS 1 CI used on Noetic (Eigen 3.3.7).
RUN set -e; mkdir -p /opt/kiss_deps && cd /opt/kiss_deps; \
    get() { curl -fsSL -o "$1.tgz" "$2" && echo "$3  $1.tgz" | sha256sum -c - \
            && mkdir "$1" && tar -xzf "$1.tgz" -C "$1" --strip-components=1 && rm "$1.tgz"; }; \
    get sophus    https://github.com/nachovizzo/Sophus/archive/refs/tags/1.22.11.tar.gz   0a8a2c39fed766ab958b274c0cf2858e7812c20f83d04b8bbf9b61b840f8d126; \
    get tbb       https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz eee380323bb7ce864355ed9431f85c43955faaae9e9bce35c62b372d7ffd9f8b; \
    get robin_map https://github.com/Tessil/robin-map/archive/refs/tags/v1.2.1.tar.gz     2b54d2c1de2f73bea5c51d5dcbd64813a08caf1bfddcfdeee40ab74e9599e8e3

# KISS-ICP at tag v0.3.0 (last release with the ROS 1 wrapper; ROS 1 was removed in v0.4.0, PR #280).
# The whole repository is needed: ros/CMakeLists.txt builds ../cpp/kiss_icp in-tree.
RUN set -e; mkdir -p /ws/src/kiss-icp && cd /ws/src/kiss-icp && git init -q \
    && git remote add origin https://github.com/PRBonn/kiss-icp \
    && git fetch -q --depth 1 origin "${KISS_SHA}" && git checkout -q FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${KISS_SHA}"

# Build in Release with CMake 3.25.3 and GCC 8, fully offline:
#  - USE_SYSTEM_* pin the dependency choice instead of depending on what happens to be installed;
#  - FETCHCONTENT_SOURCE_DIR_<NAME> points FetchContent at the checksum-verified sources above;
#  - gcc-ar-8/gcc-ranlib-8 handle oneTBB's -flto objects with the GCC 8 LTO plugin;
#  - the build directory is kept: the node's RPATH points at the oneTBB library built there.
RUN source /opt/ros/melodic/setup.bash \
    && export PATH=/opt/cmake/bin:${PATH} \
    && cd /ws \
    && catkin_make -j4 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER=/usr/bin/gcc-8 -DCMAKE_CXX_COMPILER=/usr/bin/g++-8 \
        -DCMAKE_AR=/usr/bin/gcc-ar-8 -DCMAKE_RANLIB=/usr/bin/gcc-ranlib-8 \
        -DUSE_CCACHE=OFF \
        -DUSE_SYSTEM_EIGEN3=ON -DUSE_SYSTEM_SOPHUS=OFF -DUSE_SYSTEM_TBB=OFF -DUSE_SYSTEM_TSL-ROBIN-MAP=OFF \
        -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
        -DFETCHCONTENT_SOURCE_DIR_SOPHUS=/opt/kiss_deps/sophus \
        -DFETCHCONTENT_SOURCE_DIR_TBB=/opt/kiss_deps/tbb \
        -DFETCHCONTENT_SOURCE_DIR_TESSIL=/opt/kiss_deps/robin_map \
    && test -x /ws/devel/lib/kiss_icp/odometry_node \
    && ! (ldd /ws/devel/lib/kiss_icp/odometry_node | grep -q 'not found')

# Record what was built, for the run manifests.
RUN set -e; echo "kiss-icp $(git -C /ws/src/kiss-icp rev-parse HEAD)" > /ws/BUILD_INFO.txt; \
    (ldd /ws/devel/lib/kiss_icp/odometry_node | grep -E 'tbb|stdc' || true) >> /ws/BUILD_INFO.txt; \
    (dpkg-query -W g++-8 libeigen3-dev ros-melodic-roscpp ros-melodic-tf2-ros || true) >> /ws/BUILD_INFO.txt; \
    /opt/cmake/bin/cmake --version | head -n1 >> /ws/BUILD_INFO.txt; \
    echo "sophus 1.22.11 (nachovizzo), oneTBB v2021.8.0, robin-map v1.2.1, Eigen system" >> /ws/BUILD_INFO.txt

# Run-time launch files (configuration D and derived T) are mounted from configs/kiss at run time.
WORKDIR /ws

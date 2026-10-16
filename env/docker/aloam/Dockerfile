# A-LOAM (paper ID `aloam`) on the shared ROS 1 Melodic base. Written, NOT built (disk budget).
# Build context: $R/env
#   Configuration D (upstream constants):
#     docker build -f docker/aloam/Dockerfile -t slambench/aloam:D .
#   One configuration per image tag, e.g. a sensitivity level:
#     docker build -f docker/aloam/Dockerfile --build-arg NEARBY_SCAN=4 -t slambench/aloam:d-4 .
#   mapping_line_resolution / mapping_plane_resolution are ROS parameters (launch file), not build args.
# Pins: env/src_pins_lego_aloam.json. Source audit, frames and open questions: configs/aloam/source_audit.md.
FROM slambench/base:melodic

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned sources: A-LOAM devel HEAD (the default and only branch).
ARG ALOAM_SHA=e51f88c12b483ce42dfa219783712c7234dc8c08
# Ceres: the README names no version ("Follow Ceres Installation"); the repository's own docker/Dockerfile pins
# CERES_VERSION=1.12.0 (Kinetic). 1.12.0 is used here; see open questions (1.13.0 = Ubuntu 18.04 package,
# 1.14.0 = latest release at the pinned A-LOAM commit). 1.12.0 compiled against Eigen 3.3.4 in a host probe.
ARG CERES_TAG=1.12.0
ARG CERES_COMMIT=029799d757b4ed2be5af64899178928f18cb6e28
ARG CERES_JOBS=4

# Ceres dependencies as in A-LOAM's docker/Dockerfile (glog, ATLAS/LAPACK, SuiteSparse; gflags comes with glog),
# plus the ROS packages A-LOAM's CMakeLists asks for (already in ros:melodic-perception; listed explicitly).
# PCL: Melodic's PCL 1.8.1 from the base image (upstream docker built PCL 1.8.0 from source on Kinetic).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
        ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-tf ros-melodic-rosbag \
        ros-melodic-pcl-ros ros-melodic-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# Ceres 1.12.0 from source, verified against the pinned commit. Upstream docker ran a plain `cmake ..`
# (Ceres defaults to Release); tests and examples are switched off only to save build time (they do not
# change libceres). Ceres 1.12.0's '-march=native' branch is not taken on Linux (GCC_VERSION is only set in
# its macOS branch), so libceres and A-LOAM are compiled for the same instruction set.
RUN set -e; \
    git clone -q --depth 1 --branch "${CERES_TAG}" https://github.com/ceres-solver/ceres-solver.git /opt/ceres-src; \
    test "$(git -C /opt/ceres-src rev-parse HEAD)" = "${CERES_COMMIT}"; \
    mkdir /opt/ceres-src/build && cd /opt/ceres-src/build; \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF; \
    make -j"${CERES_JOBS}" install; \
    ldconfig; \
    cd / && rm -rf /opt/ceres-src

# A-LOAM at the pinned commit only (shallow fetch by SHA). One catkin package: aloam_velodyne.
RUN set -e; \
    mkdir -p /ws/src/A-LOAM && cd /ws/src/A-LOAM; \
    git init -q && git remote add origin https://github.com/HKUST-Aerial-Robotics/A-LOAM; \
    git fetch -q --depth 1 origin "${ALOAM_SHA}" && git checkout -q FETCH_HEAD; \
    test "$(git rev-parse HEAD)" = "${ALOAM_SHA}"

# Patches, applied in numerical order (each file starts with a '# reason:' line, which git apply skips):
#   0001 build-time override hooks (DISTANCE_SQ_THRESHOLD, NEARBY_SCAN, both Huber scales, scan period)
#   0002 logging only: start-up ROS_INFO line '[slambench-constants]' with effective constants and parameters
COPY patches/aloam /patches/aloam
RUN set -e; for p in /patches/aloam/0*.patch; do \
        echo "applying $p"; git -C /ws/src/A-LOAM apply --verbose "$p"; \
    done

# Compile-time constants of this configuration (brief Section 5.4 and the scan period).
# Declared after the slow layers so every configuration reuses the cached Ceres layer.
# Empty = no override = upstream value. Units: m^2, scan lines, m, m, s.
# SCAN_PERIOD sets both scanPeriod (scanRegistration.cpp) and SCAN_PERIOD (laserOdometry.cpp), keeping them equal.
ARG DISTANCE_SQ_THRESHOLD=
ARG NEARBY_SCAN=
ARG ODOM_HUBER_DELTA=
ARG MAPPING_HUBER_DELTA=
ARG SCAN_PERIOD=
LABEL slambench.method="aloam" \
      slambench.aloam_sha="${ALOAM_SHA}" \
      slambench.ceres="${CERES_TAG} ${CERES_COMMIT}" \
      slambench.constants="DISTANCE_SQ_THRESHOLD=${DISTANCE_SQ_THRESHOLD};NEARBY_SCAN=${NEARBY_SCAN};ODOM_HUBER_DELTA=${ODOM_HUBER_DELTA};MAPPING_HUBER_DELTA=${MAPPING_HUBER_DELTA};SCAN_PERIOD=${SCAN_PERIOD}"

# Build. A-LOAM's CMakeLists forces CMAKE_BUILD_TYPE=Release with '-O3 -Wall -g' and overwrites CMAKE_CXX_FLAGS,
# which is why overrides travel in the SLAMBENCH_DEFS cache variable added by patch 0001.
RUN set -e; source /opt/ros/melodic/setup.bash; cd /ws; \
    defs=""; \
    add() { if [ -n "$2" ]; then defs="${defs:+$defs }$1=$2"; fi; }; \
    add DISTANCE_SQ_THRESHOLD_OVERRIDE "${DISTANCE_SQ_THRESHOLD}"; \
    add NEARBY_SCAN_OVERRIDE           "${NEARBY_SCAN}"; \
    add ODOM_HUBER_DELTA_OVERRIDE      "${ODOM_HUBER_DELTA}"; \
    add MAPPING_HUBER_DELTA_OVERRIDE   "${MAPPING_HUBER_DELTA}"; \
    add SCAN_PERIOD_OVERRIDE           "${SCAN_PERIOD}"; \
    echo "SLAMBENCH_DEFS='${defs}'"; \
    catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DSLAMBENCH_DEFS="${defs}"; \
    echo "SLAMBENCH_DEFS=${defs}" > /ws/SLAMBENCH_DEFS.txt; \
    rm -rf /ws/build

# Record what was built, for the run manifests (binary hashes identify the configuration).
RUN set -e; { \
        echo "aloam_velodyne $(git -C /ws/src/A-LOAM rev-parse HEAD) $(git -C /ws/src/A-LOAM diff --stat | tail -n1)"; \
        cat /ws/SLAMBENCH_DEFS.txt; \
        sha256sum /patches/aloam/*.patch; \
        sha256sum /ws/devel/lib/aloam_velodyne/*; \
        echo "ceres ${CERES_TAG} ${CERES_COMMIT}"; \
        (dpkg-query -W libgoogle-glog-dev libsuitesparse-dev libatlas-base-dev libpcl-dev libeigen3-dev ros-melodic-pcl-ros || true); \
        g++ --version | head -n1; cmake --version | head -n1; \
    } > /ws/BUILD_INFO.txt; cat /ws/BUILD_INFO.txt

# Run-time launch files (configs/aloam/<config>/aloam_velodyne_VLP_16.launch) are mounted at run time; no RViz.
WORKDIR /ws

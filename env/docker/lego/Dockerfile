# LeGO-LOAM (paper ID `lego`) on the shared ROS 1 Melodic base. Written, NOT built (disk budget).
# Build context: $R/env
#   Configuration D (upstream constants):
#     docker build -f docker/lego/Dockerfile -t slambench/lego:D .
#   One configuration per image tag, e.g. a sensitivity level:
#     docker build -f docker/lego/Dockerfile --build-arg SEGMENT_THETA_DEG=30 -t slambench/lego:b-30 .
# Pins: env/src_pins_lego_aloam.json. Source audit, frames and open questions: configs/lego/source_audit.md.
FROM slambench/base:melodic

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned sources: LeGO-LOAM master HEAD and the GTSAM tag named in the LeGO-LOAM README (4.0.0-alpha2).
ARG LEGO_SHA=896a7a95a8bc510b76819d4cc48707e344bad621
ARG GTSAM_TAG=4.0.0-alpha2
ARG GTSAM_COMMIT=c82fe1fde2fc988b6bde5e4798b66129bbb5da19
# README builds GTSAM with a plain `cmake ..`, i.e. GTSAM's bundled Eigen 3.2.10 (OFF). LeGO-LOAM itself then
# compiles against the system Eigen 3.3.4 pulled in by PCL. Kept OFF to follow the README; see open questions.
ARG GTSAM_USE_SYSTEM_EIGEN=OFF
ARG GTSAM_JOBS=4

# Build dependencies of GTSAM (Boost 1.65 from Ubuntu 18.04) and the ROS packages LeGO-LOAM's CMakeLists asks for.
# Most ROS packages are already in ros:melodic-perception; listing them makes the dependency explicit.
# Exact installed versions are written to /ws/BUILD_INFO.txt below.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libboost-serialization-dev libboost-system-dev libboost-filesystem-dev libboost-thread-dev \
        libboost-program-options-dev libboost-date-time-dev libboost-timer-dev libboost-chrono-dev \
        libboost-regex-dev \
        ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-tf \
        ros-melodic-pcl-ros ros-melodic-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# GTSAM 4.0.0-alpha2 from source, verified against the pinned commit.
# Flags pinned explicitly (the README's `cmake ..` would pick TBB/MKL up only if present):
#   TBB and MKL OFF (not installed in the base; pinned for reproducibility), tests/examples/unstable/wrap OFF
#   (not linked by LeGO-LOAM; they do not change libgtsam). Build type Release (GTSAM's own default).
#   GTSAM 4.0.0-alpha2 has no -march=native option and adds no -march flags.
RUN set -e; \
    git clone -q --depth 1 --branch "${GTSAM_TAG}" https://github.com/borglab/gtsam.git /opt/gtsam-src; \
    test "$(git -C /opt/gtsam-src rev-parse HEAD)" = "${GTSAM_COMMIT}"; \
    mkdir /opt/gtsam-src/build && cd /opt/gtsam-src/build; \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DGTSAM_USE_SYSTEM_EIGEN=${GTSAM_USE_SYSTEM_EIGEN} \
             -DGTSAM_WITH_TBB=OFF -DGTSAM_WITH_EIGEN_MKL=OFF -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF \
             -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
             -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WRAP=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
             -DGTSAM_BUILD_PYTHON=OFF; \
    make -j"${GTSAM_JOBS}" install; \
    ldconfig; \
    cd / && rm -rf /opt/gtsam-src

# LeGO-LOAM at the pinned commit only (shallow fetch by SHA). The repository holds two catkin packages:
# LeGO-LOAM/ (lego_loam) and cloud_msgs/.
RUN set -e; \
    mkdir -p /ws/src/LeGO-LOAM && cd /ws/src/LeGO-LOAM; \
    git init -q && git remote add origin https://github.com/RobustFieldAutonomyLab/LeGO-LOAM; \
    git fetch -q --depth 1 origin "${LEGO_SHA}" && git checkout -q FETCH_HEAD; \
    test "$(git rev-parse HEAD)" = "${LEGO_SHA}"

# Patches, applied in numerical order (each file starts with a '# reason:' line, which git apply skips):
#   0001 build-time override hooks for the utility.h constants (no override = upstream lines compiled)
#   0002 logging only: start-up ROS_INFO line '[slambench-constants]' with the effective constants
COPY patches/lego /patches/lego
RUN set -e; for p in /patches/lego/0*.patch; do \
        echo "applying $p"; git -C /ws/src/LeGO-LOAM apply --verbose "$p"; \
    done

# Compile-time constants of this configuration (brief Section 5.3 and the scan period).
# Declared here, after the slow layers, so every configuration reuses the cached GTSAM layer.
# Empty = no override = upstream value. Units: s, deg, (curvature), (curvature), m^2, s.
ARG MAPPING_PROCESS_INTERVAL=
ARG SEGMENT_THETA_DEG=
ARG EDGE_THRESHOLD=
ARG SURF_THRESHOLD=
ARG NEAREST_FEATURE_SEARCH_SQ_DIST=
ARG SCAN_PERIOD=
LABEL slambench.method="lego" \
      slambench.lego_sha="${LEGO_SHA}" \
      slambench.gtsam="${GTSAM_TAG} ${GTSAM_COMMIT} system_eigen=${GTSAM_USE_SYSTEM_EIGEN}" \
      slambench.constants="MAPPING_PROCESS_INTERVAL=${MAPPING_PROCESS_INTERVAL};SEGMENT_THETA_DEG=${SEGMENT_THETA_DEG};EDGE_THRESHOLD=${EDGE_THRESHOLD};SURF_THRESHOLD=${SURF_THRESHOLD};NEAREST_FEATURE_SEARCH_SQ_DIST=${NEAREST_FEATURE_SEARCH_SQ_DIST};SCAN_PERIOD=${SCAN_PERIOD}"

# Build in Release. -j1 as the README requires for a first build: imageProjection/featureAssociation depend on
# cloud_msgs_gencpp, but mapOptmization/transformFusion also include the generated header without that dependency.
# Note: README's catkin_make sets no build type (flags '-std=c++11 -O3'); Release adds '-O3 -DNDEBUG'.
RUN set -e; source /opt/ros/melodic/setup.bash; cd /ws; \
    defs=""; \
    add() { if [ -n "$2" ]; then defs="${defs:+$defs }$1=$2"; fi; }; \
    add MAPPING_PROCESS_INTERVAL_OVERRIDE       "${MAPPING_PROCESS_INTERVAL}"; \
    add SEGMENT_THETA_DEG_OVERRIDE              "${SEGMENT_THETA_DEG}"; \
    add EDGE_THRESHOLD_OVERRIDE                 "${EDGE_THRESHOLD}"; \
    add SURF_THRESHOLD_OVERRIDE                 "${SURF_THRESHOLD}"; \
    add NEAREST_FEATURE_SEARCH_SQ_DIST_OVERRIDE "${NEAREST_FEATURE_SEARCH_SQ_DIST}"; \
    add SCAN_PERIOD_OVERRIDE                    "${SCAN_PERIOD}"; \
    echo "SLAMBENCH_DEFS='${defs}'"; \
    catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DSLAMBENCH_DEFS="${defs}"; \
    echo "SLAMBENCH_DEFS=${defs}" > /ws/SLAMBENCH_DEFS.txt; \
    rm -rf /ws/build

# Record what was built, for the run manifests (binary hashes identify the configuration).
RUN set -e; { \
        echo "lego_loam $(git -C /ws/src/LeGO-LOAM rev-parse HEAD) $(git -C /ws/src/LeGO-LOAM diff --stat | tail -n1)"; \
        cat /ws/SLAMBENCH_DEFS.txt; \
        sha256sum /patches/lego/*.patch; \
        sha256sum /ws/devel/lib/lego_loam/*; \
        echo "gtsam ${GTSAM_TAG} ${GTSAM_COMMIT} GTSAM_USE_SYSTEM_EIGEN=${GTSAM_USE_SYSTEM_EIGEN}"; \
        (dpkg-query -W libboost-serialization-dev libpcl-dev libeigen3-dev libopencv-dev ros-melodic-pcl-ros || true); \
        g++ --version | head -n1; cmake --version | head -n1; \
    } > /ws/BUILD_INFO.txt; cat /ws/BUILD_INFO.txt

# Run-time launch files (configs/lego/<config>/run.launch) are mounted at run time; no RViz is installed.
WORKDIR /ws

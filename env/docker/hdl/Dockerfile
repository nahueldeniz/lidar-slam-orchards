# hdl Graph SLAM (paper ID `hdl`) on the shared ROS 1 Melodic base.
# Build context: $R/env   e.g.  docker build -f docker/hdl/Dockerfile -t slambench/hdl:melodic .
# Pins and their justification: env/src_pins_hdl_kiss.json. Not built yet (disk budget); see open questions there.
FROM slambench/base:melodic

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned source commits (see env/src_pins_hdl_kiss.json).
ARG HDL_SHA=95b8dce41c10667dcf0e3e9801dc22754b3bd525
ARG NDT_OMP_SHA=6f4e5f3e09b477c6bb678774a92f9150b14baefd
ARG FAST_GICP_SHA=0e7ec1441c99f7be453db2ea216d5de029387417

# ROS dependencies named in the hdl README for Melodic (geodesy, nmea_msgs, libg2o) plus
# interactive_markers, which hdl's CMakeLists requires and which upstream gets via ros-melodic-rviz
# (RViz itself is not installed). Versions are the final Melodic builds in packages.ros.org (frozen).
# g2o comes from apt (ros-melodic-libg2o 2018.3.25), as in the README and upstream docker/melodic.
# libsuitesparse-dev provides CHOLMOD/CSparse for the default solver lm_var_cholmod.
# OpenMP: GCC 7.5's libgomp is used; libomp-dev (only needed for the upstream clang images) is not installed.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-libg2o=2018.3.25-0bionic.20201015.025147 \
        ros-melodic-geodesy=0.5.3-0bionic.20221025.200855 \
        ros-melodic-nmea-msgs=1.1.0-0bionic.20221025.180322 \
        ros-melodic-interactive-markers=1.11.5-1bionic.20221025.194647 \
        libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Fetch each repository at its pinned commit only (shallow fetch by SHA keeps the image small).
# fast_gicp submodules (thirdparty/Eigen, Sophus, nvbio) are used only with BUILD_VGICP_CUDA=ON, so they are not fetched.
RUN set -e; \
    fetch() { mkdir -p "$1" && cd "$1" && git init -q && git remote add origin "$2" \
              && git fetch -q --depth 1 origin "$3" && git checkout -q FETCH_HEAD \
              && test "$(git rev-parse HEAD)" = "$3"; }; \
    fetch /ws/src/hdl_graph_slam https://github.com/koide3/hdl_graph_slam "${HDL_SHA}"; \
    fetch /ws/src/ndt_omp        https://github.com/koide3/ndt_omp        "${NDT_OMP_SHA}"; \
    fetch /ws/src/fast_gicp      https://github.com/SMRT-AIST/fast_gicp   "${FAST_GICP_SHA}"

# Patches (each file starts with a '# reason:' line, which git apply skips as preamble).
# 0001: logging only, full-precision numbers in the keyframe dump ('data' files).
COPY patches/hdl /patches/hdl
RUN set -e; for p in /patches/hdl/*.patch; do \
        echo "applying $p"; git -C /ws/src/hdl_graph_slam apply --verbose "$p"; \
    done

# Build in Release with the system toolchain (CMake 3.10.2, GCC 7.5, C++14 as set by hdl/ndt_omp).
# -j4 bounds peak RAM (g2o/PCL translation units are large); parallelism does not affect the binaries' behaviour.
RUN source /opt/ros/melodic/setup.bash \
    && cd /ws \
    && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /ws/build

# Record what was built, for the run manifests.
RUN set -e; for d in hdl_graph_slam ndt_omp fast_gicp; do \
        echo "$d $(git -C /ws/src/$d rev-parse HEAD) $(git -C /ws/src/$d diff --stat | tail -n1)"; \
    done > /ws/BUILD_INFO.txt; \
    (dpkg-query -W ros-melodic-libg2o ros-melodic-geodesy ros-melodic-nmea-msgs ros-melodic-pcl-ros \
                   libpcl-dev libeigen3-dev libsuitesparse-dev || true) >> /ws/BUILD_INFO.txt; \
    g++ --version | head -n1 >> /ws/BUILD_INFO.txt; cmake --version | head -n1 >> /ws/BUILD_INFO.txt

# Run-time launch files (configuration D and derived T) are mounted from configs/hdl at run time.
WORKDIR /ws

# Direct LiDAR Odometry (paper ID `dlo`) on the shared ROS 1 Melodic base.
#
# Build context is env/ :
#   docker build -f docker/dlo/Dockerfile -t slambench/dlo:melodic <R>/env
# Not built yet (written only). Verdict and reasons: env/docker/dlo/BUILD_NOTES.md.
#
# Configurations (D/T) are NOT baked in; mount them at run time, e.g.
#   -v <R>/configs/dlo:/configs/dlo:ro
#   roslaunch /configs/dlo/D/dlo_D.launch cfg_dir:=/configs/dlo/D
FROM slambench/base:melodic

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned source (see env/src_pins_dlo_mesh.json): master HEAD, dlo.yaml version 1.4.3.
ARG DLO_URL=https://github.com/vectr-ucla/direct_lidar_odometry.git
ARG DLO_SHA=11528c02e89a1619ee3c42ba39ecaa992bd20bf4

# 1. Dependencies. PCL 1.8.1, Eigen 3.3.4 and libgomp (OpenMP 4.5, GCC 7.5) already come with
#    the base (ros-melodic-pcl-ros). Pin them explicitly so the build fails loudly if the base
#    ever provides different versions. No RViz is installed.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libpcl-dev=1.8.1+dfsg1-2ubuntu2.18.04.1 \
        libeigen3-dev=3.3.4-4 \
    && rm -rf /var/lib/apt/lists/*

# 2. Source at the pinned commit. A depth-1 fetch of the exact SHA avoids the ~160 MB of GIFs
#    in the history; the repository has no Git LFS assets.
RUN mkdir -p /ws/src/direct_lidar_odometry \
    && cd /ws/src/direct_lidar_odometry \
    && git init -q \
    && git remote add origin "${DLO_URL}" \
    && git fetch -q --depth 1 origin "${DLO_SHA}" \
    && git checkout -q --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${DLO_SHA}"

# 3. Patches, applied in order (each file starts with a one-line '# reason:'):
#    0001 build fix: add_compile_definitions() -> add_definitions() for CMake 3.10.2.
#    0002 logging only: ROS_INFO of the keyframe thresholds actually used ([DLO_KF], [DLO_KF_THRESH]).
COPY patches/dlo/ /tmp/patches/dlo/
RUN cd /ws/src/direct_lidar_odometry \
    && for p in /tmp/patches/dlo/*.patch; do echo "applying ${p}"; git apply --verbose "${p}" || exit 1; done \
    && git diff --stat > /ws/dlo_patches_applied.txt \
    && sha256sum /tmp/patches/dlo/*.patch >> /ws/dlo_patches_applied.txt

# 4. Build in Release (the package's CMakeLists also forces CMAKE_BUILD_TYPE=Release, -std=c++14).
RUN source /opt/ros/melodic/setup.bash \
    && cd /ws \
    && catkin init \
    && catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build --no-status --summarize -j4 direct_lidar_odometry \
    && test -x /ws/devel/lib/direct_lidar_odometry/dlo_odom_node \
    && test -x /ws/devel/lib/direct_lidar_odometry/dlo_map_node

# 5. Record what was built, for manifest.json.
RUN { echo "dlo_sha=$(git -C /ws/src/direct_lidar_odometry rev-parse HEAD)"; \
      cmake --version | head -n 1; g++ --version | head -n 1; \
      dpkg-query -W -f='${Package}=${Version}\n' libpcl-dev libeigen3-dev ros-melodic-pcl-ros ros-melodic-roscpp; \
    } > /ws/build_versions.txt \
    && cat /ws/build_versions.txt

WORKDIR /ws

# SLAMesh (paper ID `mesh`) on the shared ROS 1 Melodic base.
#
# Build context is env/ :
#   docker build -f docker/mesh/Dockerfile -t slambench/mesh:melodic <R>/env
# Not built yet (written only). Verdict and reasons: env/docker/mesh/BUILD_NOTES.md.
#
# Follows the dependency recipe of SLAMesh's own docker/Dockerfile (Ceres 2.1.0, lvr2, mesh_tools
# 'noetic' branch, catkin_make) but with every component pinned, and without RViz, VNC or the
# mesh_tools RViz plugin.
#
# Configurations (D/T) are NOT baked in; mount them at run time, e.g.
#   -v <R>/configs/mesh:/configs/mesh:ro
#   roslaunch /configs/mesh/D/slamesh_online_D.launch cfg_dir:=/configs/mesh/D
FROM slambench/base:melodic

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned sources (see env/src_pins_dlo_mesh.json).
ARG SLAMESH_URL=https://github.com/RuanJY/SLAMesh.git
ARG SLAMESH_SHA=50b4356e8a24c4fe2e2dc1be37d682e703843314
ARG MESH_TOOLS_URL=https://github.com/naturerobots/mesh_tools.git
ARG MESH_TOOLS_SHA=95433f1606ffdc8a2eff7e66f791fdce9a231e09
ARG CERES_URL=https://github.com/ceres-solver/ceres-solver.git
ARG CERES_SHA=f68321e7de8929fbcdb95dd42877531e64f72f66

# 1. System dependencies, pinned.
#    - glog, gflags, ATLAS (BLAS/LAPACK): Ceres build deps, as SLAMesh docker/Dockerfile line 16.
#      SLAMesh calls Ceres with DENSE_QR + LAPACK, so LAPACK must be found (checked in step 2).
#    - ros-melodic-lvr2 20.11.3: the frozen ROS Melodic binary of lvr2 tag 20.11.3. SLAMesh's own
#      Dockerfile builds the lvr2 default branch unpinned, which today is 'main' (25.x, needs
#      CMake >= 3.16) and cannot configure on 18.04. The binary pulls in VTK 6, OpenCV 3.2,
#      HDF5, Qt5, GDAL and FLANN, i.e. the list in SLAMesh README section 2.1.
#    - libopenmpi-dev: mesh_msgs_conversions does find_package(MPI REQUIRED).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgoogle-glog-dev=0.3.5-1 \
        libgflags-dev=2.2.1-1 \
        libatlas-base-dev=3.10.3-5 \
        libopenmpi-dev=2.1.1-8 \
        libeigen3-dev=3.3.4-4 \
        libpcl-dev=1.8.1+dfsg1-2ubuntu2.18.04.1 \
        ros-melodic-lvr2=20.11.3-1bionic.20211011.173739 \
    && rm -rf /var/lib/apt/lists/*

# 2. Ceres Solver 2.1.0 from source (SLAMesh README: "version 2.0 or 2.1"), static library as in
#    upstream. Tests, examples and benchmarks are switched off (build time only; the library is
#    unchanged); -j4 limits peak RAM on the 16 GB laptop. Fail if Ceres was configured without
#    LAPACK: SLAMesh asks for DENSE_QR with LAPACK (map.cpp:972-973, 1048-1049), which Ceres
#    rejects at run time when built without it, leaving every registration step unsolved.
RUN set -o pipefail \
    && mkdir -p /opt/src/ceres-solver && cd /opt/src/ceres-solver \
    && git init -q && git remote add origin "${CERES_URL}" \
    && git fetch -q --depth 1 origin "${CERES_SHA}" && git checkout -q --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${CERES_SHA}" \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
                -DBUILD_BENCHMARKS=OFF 2>&1 | tee /opt/ceres_cmake.log \
    && grep -q "Found LAPACK library" /opt/ceres_cmake.log \
    && make -j4 install \
    && ! grep -q '^#define CERES_NO_LAPACK' /usr/local/include/ceres/internal/config.h \
    && cd / && rm -rf /opt/src/ceres-solver

# 3. mesh_tools, ROS 1 'noetic' branch at the pinned commit (SLAMesh README section 2.1). Only the
#    two packages SLAMesh needs are copied into the workspace; the RViz plugin and the HDF5/label
#    tools are not built.
RUN mkdir -p /opt/src/mesh_tools && cd /opt/src/mesh_tools \
    && git init -q && git remote add origin "${MESH_TOOLS_URL}" \
    && git fetch -q --depth 1 origin "${MESH_TOOLS_SHA}" && git checkout -q --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${MESH_TOOLS_SHA}" \
    && cp -r mesh_msgs mesh_msgs_conversions /ws/src/ \
    && echo "mesh_tools_sha=${MESH_TOOLS_SHA}" > /ws/src/MESH_TOOLS_SHA.txt \
    && cd / && rm -rf /opt/src/mesh_tools

# 4. SLAMesh at the pinned commit. No patches: outputs are already stamped with the input scan
#    stamp (see BUILD_NOTES.md), and no build fix is expected on Melodic.
RUN mkdir -p /ws/src/SLAMesh && cd /ws/src/SLAMesh \
    && git init -q && git remote add origin "${SLAMESH_URL}" \
    && git fetch -q --depth 1 origin "${SLAMESH_SHA}" && git checkout -q --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${SLAMESH_SHA}"

# 5. Build with catkin_make, as upstream (docker/Dockerfile line 60), in Release (SLAMesh's
#    CMakeLists also forces Release and -std=c++14).
#    Build fix (build order only): mesh_msgs_conversions does not declare a dependency on the
#    generated mesh_msgs headers, so a parallel catkin_make can compile it first and fail with
#    "mesh_msgs/MeshFaceCluster.h: No such file". Generate the message headers first.
RUN source /opt/ros/melodic/setup.bash \
    && cd /ws \
    && catkin_make -DCMAKE_BUILD_TYPE=Release mesh_msgs_generate_messages_cpp \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 \
    && test -x /ws/devel/lib/slamesh/slamesh

# 6. Result folder used by the launch file's default result_path
#    ($(find slamesh)/../../slamesh_result/kitti = /ws/slamesh_result/kitti); the node also writes
#    <result_path>/<seq>_mesh.ply, so the 'kitti' sub-folder must exist. Mount a run folder here.
RUN mkdir -p /ws/slamesh_result/kitti
# Runs pass result_path:=/tmp/slamesh_result (configs/methods.yaml); containers run as the host
# user, so the folder must be world-writable. The mesh is written at shutdown and discarded.
RUN mkdir -p /tmp/slamesh_result && chmod 1777 /tmp/slamesh_result

# 7. Record what was built, for manifest.json.
RUN { echo "slamesh_sha=$(git -C /ws/src/SLAMesh rev-parse HEAD)"; cat /ws/src/MESH_TOOLS_SHA.txt; \
      echo "ceres_sha=${CERES_SHA}"; grep -E 'CERES_NO_|CERES_USE_' /usr/local/include/ceres/internal/config.h; \
      cmake --version | head -n 1; g++ --version | head -n 1; \
      update-alternatives --query liblapack.so.3-x86_64-linux-gnu 2>/dev/null | grep -E '^Value:' || true; \
      dpkg-query -W -f='${Package}=${Version}\n' ros-melodic-lvr2 libpcl-dev libeigen3-dev \
          libatlas-base-dev libgoogle-glog-dev libgflags-dev libopenmpi-dev ros-melodic-roscpp; \
    } > /ws/build_versions.txt \
    && cat /ws/build_versions.txt

WORKDIR /ws

# Shared ROS 1 Melodic base for all method images and the infrastructure image.
# Built once; every method image starts FROM slambench/base:melodic so that all
# methods share the same OS, ROS, PCL, Eigen and compiler versions.
# The upstream tag is pinned by digest in env/versions.md once pulled.
FROM ros:melodic-perception-bionic

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# The ROS apt signing key shipped in older ros:melodic images has expired, so
# `apt-get update` fails on the ROS source. Drop the ROS source, install the
# tools to fetch the current key from the rosdistro repository, then re-add the
# source with signed-by. Build fix only; it changes no package versions.
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list /etc/apt/sources.list.d/ros-latest.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates gnupg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" \
         > /etc/apt/sources.list.d/ros1-latest.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git cmake wget \
        python-catkin-tools python-pip \
        ros-melodic-tf2-ros ros-melodic-tf2-eigen ros-melodic-tf-conversions \
        ros-melodic-pcl-ros ros-melodic-nav-msgs ros-melodic-geometry-msgs \
        ros-melodic-sensor-msgs ros-melodic-rosbag \
    && rm -rf /var/lib/apt/lists/*

# All catkin workspaces live in /ws; method images add their packages to /ws/src.
RUN mkdir -p /ws/src
WORKDIR /ws

# Sourcing happens in the entrypoint so `docker run <image> roslaunch ...` works.
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

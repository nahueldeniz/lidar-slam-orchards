# Infrastructure image: roscore, rosbag play/record, rosservice calls and the
# frame logger. Runs as a separate container so its CPU and memory are never
# attributed to a method. Build context: revision_rerun/env
FROM slambench/base:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-topic-tools ros-melodic-rosbag ros-melodic-rostopic \
        ros-melodic-rosservice ros-melodic-rosnode \
    && rm -rf /var/lib/apt/lists/*

COPY docker/infra/slambench_tools /ws/src/slambench_tools
RUN source /opt/ros/melodic/setup.bash \
    && catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build --no-status \
    && rm -rf /ws/build /ws/logs

FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-matplotlib \
        python3-numpy \
        python3-scipy \
        python3-yaml \
        ros-humble-nav2-map-server \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-lc"]

ENV ROS_DOMAIN_ID=77 \
    ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

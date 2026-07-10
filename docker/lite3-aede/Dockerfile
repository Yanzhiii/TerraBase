FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-message-filters \
    libeigen3-dev \
    libpcl-dev \
    qtbase5-dev \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/terrabase
COPY . /opt/terrabase

RUN source /opt/ros/humble/setup.bash && \
    cd /opt/terrabase/terra_base_ws && \
    colcon build \
      --merge-install \
      --executor sequential \
      --packages-select \
        lite3_bridge \
        local_planner \
        terrain_analysis \
        terrain_analysis_ext \
        sensor_scan_generation \
        waypoint_rviz_plugin \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN chmod +x /opt/terrabase/docker/lite3-aede/entrypoint.sh

ENTRYPOINT ["/opt/terrabase/docker/lite3-aede/entrypoint.sh"]
CMD ["ros2", "launch", "lite3_bridge", "lite3_aede.launch.py", "input_profile:=raw_smoke", "enable_motion:=false"]

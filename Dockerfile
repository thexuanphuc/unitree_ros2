ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# install common dependencies 
RUN apt update && apt install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install build dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    libeigen3-dev \
    liblcm-dev && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers  \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-lifecycle \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-geometry-msgs && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# Install execution dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-imu-sensor-broadcaster \
    ros-${ROS_DISTRO}-force-torque-sensor-broadcaster \
    ros-${ROS_DISTRO}-robot-state-publisher && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# Install test dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# Colored prompt and ROS source
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' ~/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

COPY . /home/ros2_ws/src/ros2_control_unitree
WORKDIR /home/ros2_ws

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc


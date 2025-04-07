ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full

ENV DEBIAN_FRONTEND noninteractive

COPY unitree_controller/script/install_requirement.sh install_requirements.sh

RUN bash install_requirements.sh

RUN apt-get update \
    && apt-get upgrade -y \
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/ros2_ws/src/ros2_control_unitree

#RUN colcon build

#RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros2_ws

# To bind X servers
#export DISPLAY=:0.0
#xhost +local:docker


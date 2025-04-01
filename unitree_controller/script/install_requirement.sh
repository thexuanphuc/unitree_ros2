#!/bin/bash

# Update package list
sudo apt update

# Install build dependencies
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-eigen3-cmake-module
sudo apt install libeigen3-dev
sudo apt install liblcm-dev

# Install runtime dependencies
sudo apt install ros-humble-pluginlib
sudo apt install ros-humble-realtime-tools
sudo apt install ros-humble-controller-interface
sudo apt install ros-humble-hardware-interface
sudo apt install ros-humble-rclcpp
sudo apt install ros-humble-rclcpp-lifecycle
sudo apt install ros-humble-geometry-msgs

# Install execution dependencies
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-imu-sensor-broadcaster
sudo apt install ros-humble-force-torque-sensor-broadcaster
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-controller-manager
# Install test dependencies
sudo apt install ros-humble-ament-lint-auto
sudo apt install ros-humble-ament-lint-common

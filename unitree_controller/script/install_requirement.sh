#!/bin/bash

# Update package list
sudo apt update -y

# Install build dependencies
sudo apt install ros-humble-ament-cmake -y
sudo apt install ros-humble-eigen3-cmake-module -y
sudo apt install libeigen3-dev -y
sudo apt install liblcm-dev -y

# Install runtime dependencies
sudo apt install ros-humble-pluginlib -y
sudo apt install ros-humble-realtime-tools -y
sudo apt install ros-humble-controller-interface -y
sudo apt install ros-humble-hardware-interface -y
sudo apt install ros-humble-rclcpp -y
sudo apt install ros-humble-rclcpp-lifecycle -y
sudo apt install ros-humble-ros2-control -y
sudo apt install ros-humble-ros2-controllers -y
sudo apt install ros-humble-geometry-msgs -y

# Install execution dependencies
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-joint-state-broadcaster -y
sudo apt install ros-humble-imu-sensor-broadcaster -y
sudo apt install ros-humble-force-torque-sensor-broadcaster -y
sudo apt install ros-humble-gazebo-ros -y
sudo apt install ros-humble-gazebo-ros2-control -y
sudo apt install ros-humble-controller-manager -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-plugins -y

# Install test dependencies
sudo apt install ros-humble-ament-lint-auto -y
sudo apt install ros-humble-ament-lint-common -y

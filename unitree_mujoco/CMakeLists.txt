cmake_minimum_required(VERSION 3.8...3.10)
project(unitree_mujoco)

#===============================================================================
# Compiler Settings
#===============================================================================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#===============================================================================
# Dependencies
#===============================================================================
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)


#===============================================================================
# shared data
#===============================================================================
set(A1_SHARED_SOURCE_DIR "../../a1_data_shared")
set(A1_SHARED_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/a1_shared_data_build")

add_subdirectory(${A1_SHARED_SOURCE_DIR} ${A1_SHARED_BINARY_DIR})

#===============================================================================
# Main Library
#===============================================================================
add_library(unitree_mujoco_aida_plugin SHARED
  src/unitree_mujoco.cpp
)

target_include_directories(unitree_mujoco_aida_plugin PRIVATE
  include
  ${A1_SHARED_SOURCE_DIR}/include
)

ament_target_dependencies(unitree_mujoco_aida_plugin
  hardware_interface
  pluginlib
  rclcpp
)

target_link_libraries(unitree_mujoco_aida_plugin
  a1_shared_data
)

#===============================================================================
# Plugin Configuration
#===============================================================================
pluginlib_export_plugin_description_file(
  hardware_interface unitree_mujoco_aida_plugin.xml
)

#===============================================================================
# Installation
#===============================================================================
install(TARGETS unitree_mujoco_aida_plugin DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES unitree_mujoco_aida_plugin.xml DESTINATION share/${PROJECT_NAME})

#===============================================================================
# Exports
#===============================================================================
ament_export_include_directories(include)
ament_export_libraries(unitree_mujoco_aida_plugin)
ament_export_dependencies(hardware_interface pluginlib rclcpp)

#===============================================================================
# Testing
#===============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
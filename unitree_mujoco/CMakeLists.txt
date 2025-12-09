cmake_minimum_required(VERSION 3.8)
project(unitree_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


set(A1_SHARED_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/a1_shared_data_build")
set(A1_SHARED_SOURCE_DIR "../../a1_data_shared")

add_subdirectory(${A1_SHARED_SOURCE_DIR} ${A1_SHARED_BINARY_DIR})


add_executable(mujoco_bridge src/ros2_mujoco_bridge.cpp)
target_link_libraries(mujoco_bridge 
  Boost::system 
  a1_shared_data
  # common_interface (if using Option A)
  rt # Real-time library is often needed for SHM on Linux
  pthread
)

target_include_directories(mujoco_bridge PRIVATE
   ${A1_SHARED_SOURCE_DIR}/include
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

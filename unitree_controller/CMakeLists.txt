cmake_minimum_required(VERSION 3.5...3.10)
project(unitree_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_hardware REQUIRED)
find_package(unitree_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(unitree_gazebo REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)

# target_link_libraries(${PROJECT_NAME}
#   ${nlohmann_json_LIBRARIES}
#   # ${ament_index_cpp_LIBRARIES}
# )
add_library(${PROJECT_NAME} SHARED
  src/unitree_controller_interface.cpp
  src/unitree_controller.cpp
  src/pd_controller.cpp
  src/mpc_start_push.cpp
  src/mpc_turning.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include 
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  Eigen3
  unitree_hardware
  unitree_gazebo # i am not sure we need this in unitree_controller package, because the controller seems separate from the gazebo interface
  unitree_msgs
  controller_manager
  std_msgs
  nlohmann_json
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "UNITREE_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface unitree_controller_plugin.xml)

# install
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(FILES unitree_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  include
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  ament_index_cpp
  controller_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  Eigen3
  unitree_hardware
  unitree_msgs
  #  i guess we need this to run this package (when others try to call this package)
  builtin_interfaces
  hardware_interface
  pluginlib
  controller_manager
  unitree_gazebo
  std_msgs
  # xtensor
)

ament_package()
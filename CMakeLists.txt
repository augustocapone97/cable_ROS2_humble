# CMakeLists.txt for ROS2
cmake_minimum_required(VERSION 3.5)
project(cable_conv)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include directories
include_directories(
  include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# Add library
add_library(CableVelControlPlugin SHARED src/CableVelControlPlugin.cpp)
ament_target_dependencies(CableVelControlPlugin
  rclcpp
  geometry_msgs
  gazebo_dev
  gazebo_ros
)

# Install resources
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS src/CableController.py DESTINATION lib/${PROJECT_NAME})

# Install targets
install(TARGETS CableVelControlPlugin
  DESTINATION lib/${PROJECT_NAME}
)

# Write the full path to a config file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/cable_model.urdf
  ${CMAKE_BINARY_DIR}/cable_model_with_path.urdf
  @ONLY
)
file(GLOB URDF_FILES "${CMAKE_BINARY_DIR}/*.urdf")

# Force Gazebo to find the plugin at runtime
set(GAZEBO_PLUGIN_PATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")

ament_package()

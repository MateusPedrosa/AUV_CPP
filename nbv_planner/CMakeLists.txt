cmake_minimum_required(VERSION 3.8)
project(nbv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library: octomap_manager
add_library(octomap_manager src/octomap_manager.cpp)
ament_target_dependencies(octomap_manager
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_eigen
)
target_link_libraries(octomap_manager
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)

# Library: nbv_planner
add_library(nbv_planner_lib src/nbv_planner.cpp)
ament_target_dependencies(nbv_planner_lib
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(nbv_planner_lib
  ${OCTOMAP_LIBRARIES}
)

# Executable: nbv_planner_node
add_executable(nbv_planner_node src/nbv_planner_node.cpp)
ament_target_dependencies(nbv_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)
target_link_libraries(nbv_planner_node
  octomap_manager
  nbv_planner_lib
)

# Install
install(TARGETS
  octomap_manager
  nbv_planner_lib
  nbv_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
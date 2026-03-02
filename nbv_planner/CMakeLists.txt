cmake_minimum_required(VERSION 3.8)
project(nbv_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ufomap REQUIRED)
find_package(ufomap_msgs REQUIRED)
find_package(ufomap_rviz_plugins REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanPath.action"
  DEPENDENCIES geometry_msgs nav_msgs
)

# --- Global Definitions ---
# Required for PCL + Eigen to work without crashing
add_definitions(${PCL_DEFINITIONS})

# --- Library: ufomap_manager ---
add_library(ufomap_manager src/ufomap_manager.cpp)
target_include_directories(ufomap_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ufomap_manager
  ${PCL_LIBRARIES}
  UFO::Map
  Eigen3::Eigen
)
ament_target_dependencies(ufomap_manager
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_eigen
)

# --- Library: simple_planner ---
#add_library(simple_planner src/simple_planner.cpp)
#target_include_directories(simple_planner PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#  ${OMPL_INCLUDE_DIRS}
#  ${OCTOMAP_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
#)
# Link OMPL and Eigen
#target_link_libraries(simple_planner
#  octomap_manager
#  ${OMPL_LIBRARIES}
#  Eigen3::Eigen
#)
#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} rosidl_typesupport_cpp)
#target_link_libraries(simple_planner "${cpp_typesupport_target}")
#add_dependencies(simple_planner ${PROJECT_NAME})
#ament_target_dependencies(simple_planner
#  rclcpp
#  rclcpp_action
#  geometry_msgs
#  nav_msgs
#  sensor_msgs
#)

# --- Library: nbv_planner_lib ---
#add_library(nbv_planner_lib src/nbv_planner.cpp)
#arget_include_directories(nbv_planner_lib PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)
#target_link_libraries(nbv_planner_lib
#  UFO::Map
#  Eigen3::Eigen
#)
#ament_target_dependencies(nbv_planner_lib
#  rclcpp
#  geometry_msgs
#  tf2
#  tf2_geometry_msgs
#)

# --- Executable: nbv_planner_node ---
add_executable(nbv_planner_node src/nbv_planner_node.cpp)
target_include_directories(nbv_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(nbv_planner_node
  ufomap_manager
#  simple_planner
#  nbv_planner_lib
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(nbv_planner_node "${cpp_typesupport_target}")
add_dependencies(nbv_planner_node ${PROJECT_NAME})
ament_target_dependencies(nbv_planner_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  message_filters
  pcl_conversions
  ufomap_msgs
)

# --- Executable: plan_path_client ---
#add_executable(plan_path_client src/plan_path_client.cpp)
#target_include_directories(plan_path_client PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

# Link action interface to client
#target_link_libraries(plan_path_client "${cpp_typesupport_target}")
#add_dependencies(plan_path_client ${PROJECT_NAME})

#ament_target_dependencies(plan_path_client
#  rclcpp
#  rclcpp_action
#  geometry_msgs
#  tf2
#  tf2_geometry_msgs
#)

# --- Install Rules ---

# Install Libraries (to global lib/)
install(TARGETS
  ufomap_manager
#  simple_planner
#  nbv_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Executables (to lib/project_name/)
install(TARGETS
  nbv_planner_node
  #plan_path_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install Headers
install(DIRECTORY include/
  DESTINATION include
)

# Install Launch, Config, RViz
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

# --- Export for downstream packages ---
ament_export_include_directories(include)
ament_export_libraries(ufo_manager)#nbv_planner_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  ompl
  ufomap
)
ament_package()
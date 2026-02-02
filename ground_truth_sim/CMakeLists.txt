cmake_minimum_required(VERSION 3.8)
project(ground_truth_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(voxel_map_ground_truth src/ground_truth_map.cpp)

ament_target_dependencies(voxel_map_ground_truth
  rclcpp
  pcl_conversions
  pcl_ros
)

target_include_directories(voxel_map_ground_truth PUBLIC
  ${PCL_INCLUDE_DIRS}      # Tells compiler where to find PCL headers
  ${OCTOMAP_INCLUDE_DIRS}  # Tells compiler where to find OctoMap headers
)

target_link_libraries(voxel_map_ground_truth
  ${PCL_LIBRARIES}      # Links the actual PCL binary code
  ${OCTOMAP_LIBRARIES}  # Links the actual OctoMap binary code
)

install(TARGETS voxel_map_ground_truth
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

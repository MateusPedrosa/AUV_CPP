cmake_minimum_required(VERSION 3.8)
project(sonar_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sonar_point_cloud src/sonar_point_cloud.cpp)
target_include_directories(sonar_point_cloud PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sonar_point_cloud
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(sonar_point_cloud
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  #message_filters
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

install(TARGETS
  sonar_point_cloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

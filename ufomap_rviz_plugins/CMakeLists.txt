cmake_minimum_required(VERSION 3.9)
project(ufomap_rviz_plugins
  VERSION 1.0 LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(ufomap_msgs REQUIRED)

find_package(ufomap REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

add_library(${PROJECT_NAME} SHARED
  src/ufomap_display.cpp
  include/ufomap_rviz_plugins/ufomap_display.h
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz2
  rviz_common
  rviz_rendering
  ufomap_msgs
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  UFO::Map
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wno-register
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  ufomap_msgs
  pluginlib
)

ament_package()
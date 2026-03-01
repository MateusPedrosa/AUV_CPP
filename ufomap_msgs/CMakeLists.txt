cmake_minimum_required(VERSION 3.8.2)
project(ufomap_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ufomap REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AABB.msg"
  "msg/BoundingVolume.msg"
  "msg/Frustum.msg"
  "msg/LineSegment.msg"
  "msg/OBB.msg"
  "msg/Plane.msg"
  "msg/Point.msg"
  "msg/Ray.msg"
  "msg/Sphere.msg"
  "msg/UFOMap.msg"
  "msg/UFOMapMetaData.msg"
  "msg/UFOMapStamped.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

set(CONVERSIONS_LIB ${PROJECT_NAME}_conversions)

add_library(${CONVERSIONS_LIB} STATIC src/conversions.cpp)
ament_target_dependencies(${CONVERSIONS_LIB}
  geometry_msgs
  std_msgs
  builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${CONVERSIONS_LIB} 
    UFO::Map
    "${cpp_typesupport_target}"
)
target_include_directories(${CONVERSIONS_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${CONVERSIONS_LIB}
  EXPORT export_${CONVERSIONS_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_export_targets(export_${CONVERSIONS_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  std_msgs
  builtin_interfaces
  ufomap
)

ament_package()
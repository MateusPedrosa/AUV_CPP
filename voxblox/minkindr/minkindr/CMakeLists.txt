cmake_minimum_required(VERSION 3.5)
project(minkindr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(eigen_checks REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} INTERFACE glog::glog eigen_checks::eigen_checks)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 glog eigen_checks)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  macro(minkindr_add_test custom_target source)
      ament_add_gtest(${custom_target} ${source})
      target_link_libraries(${custom_target} ${PROJECT_NAME})
  endmacro()

  minkindr_add_test(${PROJECT_NAME}_tests_main test/test-main.cc)
  minkindr_add_test(${PROJECT_NAME}_tests_rotation test/test-rotation.cc)
  minkindr_add_test(${PROJECT_NAME}_tests_sim3 test/test-sim3.cc)
  minkindr_add_test(${PROJECT_NAME}_tests_tf test/test-transformation.cc)
  minkindr_add_test(${PROJECT_NAME}_tests_tf2d test/test-transformation-2d.cc)
endif()

ament_package()


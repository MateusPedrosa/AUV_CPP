cmake_minimum_required(VERSION 3.5)
project(eigen_checks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} src/eigen-checks.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} glog::glog)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 glog)

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
  
  macro(eigen_checks_add_test custom_target source)
      ament_add_gtest(${custom_target} ${source})
      target_link_libraries(${custom_target} ${PROJECT_NAME} Threads::Threads)
  endmacro()

  eigen_checks_add_test(test_gtest_near test/test_gtest-near.cc)
  eigen_checks_add_test(test_gtest_equal test/test_gtest-equal.cc)
  eigen_checks_add_test(test_gtest_zero test/test_gtest-zero.cc)
  eigen_checks_add_test(test_gtest_equal_double test/test_gtest-equal-double.cc)
  eigen_checks_add_test(test_glog_near test/test_glog-near.cc)
  eigen_checks_add_test(test_glog_equal test/test_glog-equal.cc)
  eigen_checks_add_test(test_glog_zero test/test_glog-zero.cc)
  eigen_checks_add_test(test_glog_equal_double test/test_glog-equal-double.cc)
  eigen_checks_add_test(test_glog-zero-sized-types test/test_glog-zero-sized-types.cc)
  eigen_checks_add_test(test_gtest-zero-sized-types test/test_gtest-zero-sized-types.cc)
endif()

ament_package()

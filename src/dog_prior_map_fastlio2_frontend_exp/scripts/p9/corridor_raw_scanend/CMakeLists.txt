cmake_minimum_required(VERSION 3.10)
project(p9_corridor_raw_scanend LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(VELODYNE_ROOT "" CACHE PATH "Pinned read-only Velodyne checkout")
execute_process(COMMAND git -C "${VELODYNE_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE PIN OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE RC)
execute_process(COMMAND git -C "${VELODYNE_ROOT}" diff HEAD --exit-code
  RESULT_VARIABLE DIRTY OUTPUT_QUIET)
if(NOT RC EQUAL 0 OR NOT DIRTY EQUAL 0 OR NOT PIN STREQUAL "29abd0e1361cb7f5eda451d2b51c35eeca45e0d5")
  message(FATAL_ERROR "Velodyne source pin/cleanliness failed")
endif()
find_package(catkin REQUIRED COMPONENTS roscpp rosbag roslib sensor_msgs velodyne_msgs tf2_ros)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)
add_executable(p9_corridor_extract extract.cpp
  "${VELODYNE_ROOT}/velodyne_pointcloud/src/lib/rawdata.cc"
  "${VELODYNE_ROOT}/velodyne_pointcloud/src/lib/calibration.cc")
target_include_directories(p9_corridor_extract PRIVATE
  "${VELODYNE_ROOT}/velodyne_pointcloud/include" ${catkin_INCLUDE_DIRS})
target_link_libraries(p9_corridor_extract PRIVATE ${catkin_LIBRARIES} Eigen3::Eigen yaml-cpp)
target_compile_options(p9_corridor_extract PRIVATE -Wall -Wextra)
target_compile_definitions(p9_corridor_extract PRIVATE HAVE_NEW_YAMLCPP)
enable_testing()
add_test(NAME raw_timing_self_test COMMAND p9_corridor_extract --self-test)
add_test(NAME pinned_packet_test COMMAND p9_corridor_extract --packet-test
  "${VELODYNE_ROOT}/velodyne_pointcloud/params/VLP16db.yaml")

# Optional input/initialization probe. Links the real P7 chain without any NDT.
set(FASTLIO2_ROOT "" CACHE PATH "Pinned FAST-LIO2 for input admission tests")
if(FASTLIO2_ROOT)
  execute_process(COMMAND git -C "${FASTLIO2_ROOT}" rev-parse HEAD
    OUTPUT_VARIABLE FAST_PIN OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE FAST_RC)
  if(NOT FAST_RC EQUAL 0 OR NOT FAST_PIN STREQUAL "7cc4175de6f8ba2edf34bab02a42195b141027e9")
    message(FATAL_ERROR "FAST-LIO2 pin mismatch")
  endif()
  find_package(OpenMP REQUIRED)
  set(PKG "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  add_library(p9_corridor_input_core STATIC
    "${PKG}/src/p7_replay_io.cpp" "${PKG}/src/fastlio2_frontend_ikfom.cpp"
    "${PKG}/src/registration_geometry.cpp" "${PKG}/src/scan_processor.cpp"
    "${PKG}/src/lidar_deskew_geometry.cpp")
  target_include_directories(p9_corridor_input_core PUBLIC "${PKG}/include"
    PRIVATE "${FASTLIO2_ROOT}/include")
  target_link_libraries(p9_corridor_input_core PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX)
  add_executable(p9_corridor_input_probe input_probe.cpp)
  target_link_libraries(p9_corridor_input_probe PRIVATE p9_corridor_input_core)
  foreach(TEST p7_replay_io_test scan_end_deskew_runtime_test fastlio2_propagation_contract_test registration_geometry_test)
    add_executable(${TEST} "${PKG}/tests/${TEST}.cpp")
    target_link_libraries(${TEST} PRIVATE p9_corridor_input_core)
    add_test(NAME ${TEST} COMMAND ${TEST})
  endforeach()
endif()

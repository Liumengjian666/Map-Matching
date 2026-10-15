cmake_minimum_required(VERSION 3.10)
project(corridor01_decode LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(VELODYNE_ROOT "/home/jian/livox_ws/superloc_adapter_ws/src/velodyne" CACHE PATH "Existing read-only driver source")
execute_process(COMMAND git -C "${VELODYNE_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE DRIVER_SHA OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE DRIVER_RC)
execute_process(COMMAND git -C "${VELODYNE_ROOT}" diff HEAD --exit-code
  RESULT_VARIABLE DRIVER_DIRTY OUTPUT_QUIET)
if(NOT DRIVER_RC EQUAL 0 OR NOT DRIVER_DIRTY EQUAL 0 OR
   NOT DRIVER_SHA STREQUAL "29abd0e1361cb7f5eda451d2b51c35eeca45e0d5")
  message(FATAL_ERROR "Existing Velodyne driver identity mismatch")
endif()
find_package(catkin REQUIRED COMPONENTS roscpp rosbag sensor_msgs velodyne_msgs tf2_ros)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(corridor01_decode_full decode_full.cpp
  "${VELODYNE_ROOT}/velodyne_pointcloud/src/lib/rawdata.cc"
  "${VELODYNE_ROOT}/velodyne_pointcloud/src/lib/calibration.cc")
target_include_directories(corridor01_decode_full PRIVATE
  "${VELODYNE_ROOT}/velodyne_pointcloud/include" ${catkin_INCLUDE_DIRS})
target_compile_definitions(corridor01_decode_full PRIVATE HAVE_NEW_YAMLCPP)
target_compile_options(corridor01_decode_full PRIVATE -Wall -Wextra)
target_link_libraries(corridor01_decode_full PRIVATE ${catkin_LIBRARIES} Eigen3::Eigen yaml-cpp)
enable_testing()
add_test(NAME decoder_packet_contract COMMAND corridor01_decode_full --self-test
  "${VELODYNE_ROOT}/velodyne_pointcloud/params/VLP16db.yaml")

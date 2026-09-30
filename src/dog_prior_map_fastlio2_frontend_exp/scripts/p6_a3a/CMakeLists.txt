cmake_minimum_required(VERSION 3.10)
project(p6_a3a_raw_decoder LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag roslib tf2_ros tf2)
set(ADAPTER_ROOT "/home/jian/livox_ws/superloc_adapter_ws" CACHE PATH "Pinned sensor adapter workspace")
set(VELODYNE_ROOT "${ADAPTER_ROOT}/src/velodyne/velodyne_pointcloud")
file(SHA256 "${VELODYNE_ROOT}/src/lib/rawdata.cc" DRIVER_SOURCE_SHA)
file(SHA256 "${VELODYNE_ROOT}/src/lib/calibration.cc" CALIBRATION_SOURCE_SHA)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/../p6_a3a_decode_velodyne.cpp" DECODER_SOURCE_SHA)
add_library(p6_a3a_pinned_velodyne_rawdata SHARED
  "${VELODYNE_ROOT}/src/lib/rawdata.cc" "${VELODYNE_ROOT}/src/lib/calibration.cc"
  ../p6_a3a_driver_identity.cpp)
target_include_directories(p6_a3a_pinned_velodyne_rawdata PUBLIC ${catkin_INCLUDE_DIRS}
  "${ADAPTER_ROOT}/devel/include" "${VELODYNE_ROOT}/include" /usr/include/eigen3)
target_link_libraries(p6_a3a_pinned_velodyne_rawdata PUBLIC ${catkin_LIBRARIES} yaml-cpp)
target_compile_definitions(p6_a3a_pinned_velodyne_rawdata PRIVATE HAVE_NEW_YAMLCPP
  A3A_DRIVER_SOURCE_SHA="${DRIVER_SOURCE_SHA}" A3A_CALIBRATION_SOURCE_SHA="${CALIBRATION_SOURCE_SHA}")
add_executable(p6_a3a_decode_velodyne ../p6_a3a_decode_velodyne.cpp)
target_compile_features(p6_a3a_decode_velodyne PRIVATE cxx_std_17)
target_include_directories(p6_a3a_decode_velodyne PRIVATE ${catkin_INCLUDE_DIRS}
  "${ADAPTER_ROOT}/devel/include" "${ADAPTER_ROOT}/src/velodyne/velodyne_pointcloud/include" /usr/include/eigen3)
target_compile_definitions(p6_a3a_decode_velodyne PRIVATE
  A3A_DECODER_SOURCE_SHA="${DECODER_SOURCE_SHA}")
target_link_libraries(p6_a3a_decode_velodyne PRIVATE p6_a3a_pinned_velodyne_rawdata ${CMAKE_DL_LIBS})

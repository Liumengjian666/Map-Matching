cmake_minimum_required(VERSION 3.0.2)
project(dog_prior_map_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  cv_bridge
  tf2_ros
  tf
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs nav_msgs geometry_msgs std_msgs diagnostic_msgs cv_bridge tf2_ros tf pcl_ros pcl_conversions livox_ros_driver2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dog_prior_map_ekf_node_cpp
  src/dog_prior_map_ekf_node.cpp
  src/dog_prior_map_ekf_node_core.cpp
  src/map_loader.cpp
  src/imu_processor.cpp
  src/lidar_matcher.cpp
  src/vision_observation.cpp
  src/ros_output.cpp
  src/math_utils.cpp
)

target_link_libraries(dog_prior_map_ekf_node_cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(dog_prior_map_ekf_node_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(dog_prior_map_ndt_node_cpp
  src/dog_prior_map_ndt_node.cpp
)

target_link_libraries(dog_prior_map_ndt_node_cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

add_dependencies(dog_prior_map_ndt_node_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Offline-only NDT objective probe. It is intentionally independent of the
# runtime localization nodes and is never linked into their execution path.
add_executable(ndt_uncertainty_probe
  src/ndt_uncertainty_probe.cpp
)

target_link_libraries(ndt_uncertainty_probe
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

# Offline-only Stage3A.6 local covariance/eigenvector alignment probe.
add_executable(ndt_direction_alignment_probe
  src/ndt_direction_alignment_probe.cpp
)

target_link_libraries(ndt_direction_alignment_probe
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

catkin_install_python(PROGRAMS
  scripts/dog_prior_map_ekf_node.py
  scripts/pcd_to_npz_map.py
  scripts/evaluate_runtime_and_accuracy.py
  scripts/build_livox_keyframe_scan_db.py
  scripts/run_dataset_evaluation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch docs rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS dog_prior_map_ekf_node_cpp dog_prior_map_ndt_node_cpp ndt_uncertainty_probe ndt_direction_alignment_probe
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

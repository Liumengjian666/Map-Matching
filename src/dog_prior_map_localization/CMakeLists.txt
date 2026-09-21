cmake_minimum_required(VERSION 3.0.2)
project(dog_prior_map_localization)

option(
  DOG_PRIOR_BUILD_RESEARCH_TOOLS
  "Build offline research and diagnostic tools"
  OFF
)

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
  src/core/oosm_replay_planner.cpp
  src/core/state_history.cpp
  src/imu_processor.cpp
  src/lidar_matcher.cpp
  src/vision_observation.cpp
  src/fusion/ndt_observation.cpp
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

# ROS-free contract test for rollback/replay planning.  It deliberately links
# only the core planner and StateHistory, so its API cannot acquire a ROS
# dependency accidentally.
add_executable(oosm_replay_planner_contract_test
  tests/oosm_replay_planner_contract_test.cpp
  src/core/oosm_replay_planner.cpp
  src/core/state_history.cpp
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

if(DOG_PRIOR_BUILD_RESEARCH_TOOLS)
  # Offline-only probes are intentionally independent of the runtime
  # localization nodes and are never linked into their execution path.
  add_executable(ndt_uncertainty_probe
    tools/offline/ndt_uncertainty_probe.cpp
  )

  target_link_libraries(ndt_uncertainty_probe
    ${PCL_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libusb-1.0.so
  )

  add_executable(ndt_direction_alignment_probe
    tools/offline/ndt_direction_alignment_probe.cpp
  )

  target_link_libraries(ndt_direction_alignment_probe
    ${PCL_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libusb-1.0.so
  )
endif()

catkin_install_python(PROGRAMS
  scripts/pcd_to_npz_map.py
  scripts/evaluate_runtime_and_accuracy.py
  scripts/build_livox_keyframe_scan_db.py
  scripts/run_dataset_evaluation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch docs rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS dog_prior_map_ekf_node_cpp dog_prior_map_ndt_node_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(DOG_PRIOR_BUILD_RESEARCH_TOOLS)
  install(TARGETS ndt_uncertainty_probe ndt_direction_alignment_probe
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

cmake_minimum_required(VERSION 3.0.2)
project(dog_prior_map_localization)

option(
  DOG_PRIOR_BUILD_RESEARCH_TOOLS
  "Build offline research and diagnostic tools"
  OFF
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  dog_prior_map_interfaces
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf pcl_ros pcl_conversions livox_ros_driver2 dog_prior_map_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(dog_prior_map_ekf_node_cpp
  src/dog_prior_map_ekf_node.cpp
  src/dog_prior_map_ekf_node_core.cpp
  src/core/oosm_replay_planner.cpp
  src/core/state_history.cpp
  src/core/imu_propagation.cpp
  src/core/imu_deskew.cpp
  src/imu_deskew_runtime.cpp
  src/imu_processor.cpp
  src/fusion/ndt_observation.cpp
  src/ros_output.cpp
)

target_link_libraries(dog_prior_map_ekf_node_cpp
  ${catkin_LIBRARIES}
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

add_executable(imu_deskew_contract_test
  tests/imu_deskew_contract_test.cpp
  src/core/state_history.cpp
  src/core/imu_propagation.cpp
  src/core/imu_deskew.cpp
)

add_executable(imu_interval_numeric_test
  tests/imu_interval_numeric_test.cpp
  src/core/imu_propagation.cpp
)

add_executable(frame_conversion_contract_test
  tests/frame_conversion_contract_test.cpp
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

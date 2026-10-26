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
  std_msgs
  tf
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  dog_prior_map_interfaces
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(OpenSSL REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs std_msgs tf pcl_ros pcl_conversions livox_ros_driver2 dog_prior_map_interfaces
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
  src/core/oosm_fixed_lag_scheduler.cpp
  src/core/oosm_replay_planner.cpp
  src/core/state_history.cpp
  src/core/imu_propagation.cpp
  src/core/imu_deskew.cpp
  src/core/selective_relative_visual_update.cpp
  src/core/square_root_covariance.cpp
  src/imu_deskew_runtime.cpp
  src/imu_processor.cpp
  src/fusion/ndt_observation.cpp
  src/fusion/skf_visual.cpp
  src/fusion/msckf_visual.cpp
  src/core/msckf_visual_measurement.cpp
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

add_executable(oosm_fixed_lag_scheduler_contract_test
  tests/oosm_fixed_lag_scheduler_contract_test.cpp
  src/core/oosm_fixed_lag_scheduler.cpp
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

add_executable(scan_local_imu_test
  tests/scan_local_imu_test.cpp
  src/core/state_history.cpp
  src/core/imu_propagation.cpp
  src/core/imu_deskew.cpp
)

add_executable(deskew_trajectory_mode_test
  tests/deskew_trajectory_mode_test.cpp
  src/core/state_history.cpp
  src/core/imu_propagation.cpp
  src/core/imu_deskew.cpp
)

add_executable(selective_relative_visual_update_test
  tests/selective_relative_visual_update_test.cpp
  src/core/selective_relative_visual_update.cpp
  src/core/square_root_covariance.cpp
)
add_executable(covariance_stability_regression_test
  tests/covariance_stability_regression_test.cpp
  src/core/selective_relative_visual_update.cpp
  src/core/square_root_covariance.cpp
)
add_executable(square_root_covariance_test
  tests/square_root_covariance_test.cpp
  src/core/square_root_covariance.cpp
  src/core/selective_relative_visual_update.cpp
  src/core/state_history.cpp
)
add_executable(msckf_visual_measurement_test
  tests/msckf_visual_measurement_test.cpp
  src/core/msckf_visual_measurement.cpp
)

add_executable(frame_conversion_contract_test
  tests/frame_conversion_contract_test.cpp
)
add_executable(visual_initial_guess_test
  tests/visual_initial_guess_test.cpp
  src/visual_aided_initial_guess.cpp
)

add_executable(dog_prior_map_ndt_node_cpp
  src/dog_prior_map_ndt_node.cpp
  src/local_map_tiles.cpp
  src/mature_ndt_algorithms.cpp
  src/skf_visual_measurement.cpp
  src/visual_aided_initial_guess.cpp
  src/external_ndt_transaction_server.cpp
  ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_shadow.cpp
  ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_temporal.cpp
  ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_anchor.cpp
  ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_weak_refinement.cpp
  ../dog_prior_map_fastlio2_frontend_exp/src/registration_geometry.cpp
)

target_include_directories(dog_prior_map_ndt_node_cpp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../dog_prior_map_fastlio2_frontend_exp/include
)

target_link_libraries(dog_prior_map_ndt_node_cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

add_dependencies(dog_prior_map_ndt_node_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Offline P45 map preparation. It reproduces the two global P44 target
# VoxelGrid passes before writing metadata-indexed 20 m XY tiles.
add_executable(build_ndt_map_tiles
  tools/offline/build_ndt_map_tiles.cpp
)
target_link_libraries(build_ndt_map_tiles
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

# Bounded component-only test for replacing the active local tile set. The
# remote pose is a deterministic test fixture; this executable never calls NDT.
add_executable(p45_local_map_switch_probe
  tools/offline/p45_local_map_switch_probe.cpp
  src/local_map_tiles.cpp
)
target_link_libraries(p45_local_map_switch_probe
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

# One-scan Newer College input adapter. It preserves the official PCD XYZ and
# uint32 scan-relative nanosecond field, exposing only the float32 seconds field
# required by the existing IMU deskew node.
add_executable(newer_college_first_scan_publisher
  src/newer_college_first_scan_publisher.cpp
)
target_link_libraries(newer_college_first_scan_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)
add_dependencies(newer_college_first_scan_publisher
  ${catkin_EXPORTED_TARGETS}
)

# Backpressured multi-scan PCD/IMU input mode used by the P44 continuous run.
# It shares the P41 single-scan adapter implementation and acknowledges each
# scan through NDT diagnostics rather than relying on a guessed delay.
add_executable(newer_college_sequence_publisher
  src/newer_college_first_scan_publisher.cpp
)
target_link_libraries(newer_college_sequence_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)
add_dependencies(newer_college_sequence_publisher
  ${catkin_EXPORTED_TARGETS}
)

# P11 SOLiD place-recognition shadow module. The upstream descriptor is kept
# isolated from the NDT/EKF runtime path and launched only by explicit opt-in.
add_library(p11_solid_place_recognition
  src/solid_place_recognition.cpp
  third_party/solid/src/solid.cpp
)
target_include_directories(p11_solid_place_recognition PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/solid/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(p11_solid_place_recognition
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so
)

add_executable(solid_reference_database_builder
  scripts/solid_reference_database_builder.cpp
)
target_link_libraries(solid_reference_database_builder
  p11_solid_place_recognition
  ${PCL_LIBRARIES}
)

add_executable(solid_shadow_node_cpp
  src/solid_shadow_node.cpp
)
target_link_libraries(solid_shadow_node_cpp
  p11_solid_place_recognition
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(solid_shadow_node_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS p11_solid_place_recognition
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(solid_adapter_contract_test
  tests/solid_adapter_contract_test.cpp
)
target_link_libraries(solid_adapter_contract_test
  p11_solid_place_recognition
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(observation_identity_contract_test tests/observation_identity_contract_test.cpp)
  add_test(NAME observation_identity_contract_test COMMAND $<TARGET_FILE:observation_identity_contract_test>)

  add_executable(ndt_target_resolution_contract_test
    tests/ndt_target_resolution_contract_test.cpp
  )
  target_link_libraries(ndt_target_resolution_contract_test ${PCL_LIBRARIES})
  add_test(NAME ndt_target_resolution_contract_test
    COMMAND $<TARGET_FILE:ndt_target_resolution_contract_test>)

  add_executable(p49_causal_trigger_contract_test
    tests/p49_causal_trigger_contract_test.cpp
  )
  target_include_directories(p49_causal_trigger_contract_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dog_prior_map_fastlio2_frontend_exp/include
  )
  add_test(NAME p49_causal_trigger_contract_test
    COMMAND $<TARGET_FILE:p49_causal_trigger_contract_test>)

  add_test(NAME msckf_visual_measurement_test COMMAND $<TARGET_FILE:msckf_visual_measurement_test>)
  add_test(NAME square_root_covariance_test
    COMMAND $<TARGET_FILE:square_root_covariance_test>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/p26_r1_skf_covariance_stability/post_fix_actual_IMU_PSD_loss.csv)
  add_test(NAME covariance_stability_regression_test
    COMMAND $<TARGET_FILE:covariance_stability_regression_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/p26_r1_skf_covariance_stability/eigensolver_false_negative.csv
      ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/p26_r1_skf_covariance_stability/double_LLT_false_negative.csv
      ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/p26_r1_skf_covariance_stability/post_fix_actual_IMU_PSD_loss.csv)
  add_test(NAME selective_relative_visual_update_test
    COMMAND $<TARGET_FILE:selective_relative_visual_update_test>)

  add_test(NAME oosm_fixed_lag_scheduler_contract_test
    COMMAND $<TARGET_FILE:oosm_fixed_lag_scheduler_contract_test>)

  add_executable(observable_pcl_ndt_contract_test
    tests/observable_pcl_ndt_contract_test.cpp
  )
  add_executable(p17_ndt_readonly_test tests/p17_ndt_readonly_test.cpp)
  target_include_directories(p17_ndt_readonly_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dog_prior_map_fastlio2_frontend_exp/include)
  target_link_libraries(p17_ndt_readonly_test ${PCL_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libusb-1.0.so)
  target_include_directories(observable_pcl_ndt_contract_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dog_prior_map_fastlio2_frontend_exp/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(observable_pcl_ndt_contract_test
    ${PCL_LIBRARIES}
    /lib/x86_64-linux-gnu/libusb-1.0.so.0
  )
  add_test(NAME observable_pcl_ndt_contract_test
    COMMAND $<TARGET_FILE:observable_pcl_ndt_contract_test>)
  set_tests_properties(observable_pcl_ndt_contract_test PROPERTIES
    ENVIRONMENT "LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0")

  add_executable(causal_pose_prediction_test
    tests/causal_pose_prediction_test.cpp
  )
  add_test(NAME causal_pose_prediction_test
    COMMAND $<TARGET_FILE:causal_pose_prediction_test>)

  add_executable(ekf_translation_prior_test tests/ekf_translation_prior_test.cpp)
  add_test(NAME ekf_translation_prior_test COMMAND $<TARGET_FILE:ekf_translation_prior_test>)

  add_executable(mature_ndt_algorithms_test
    tests/mature_ndt_algorithms_test.cpp
    src/mature_ndt_algorithms.cpp
  )
  target_include_directories(mature_ndt_algorithms_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dog_prior_map_fastlio2_frontend_exp/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  add_test(NAME mature_ndt_algorithms_test COMMAND $<TARGET_FILE:mature_ndt_algorithms_test>)

  add_executable(external_ndt_server_integration_test
    tests/external_ndt_server_integration_test.cpp
    src/external_ndt_transaction_server.cpp
  )
  target_link_libraries(external_ndt_server_integration_test
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OpenSSL::Crypto
    /usr/lib/x86_64-linux-gnu/libusb-1.0.so
  )
  add_dependencies(external_ndt_server_integration_test ${catkin_EXPORTED_TARGETS})

  add_test(NAME solid_adapter_contract_test
    COMMAND $<TARGET_FILE:solid_adapter_contract_test>)
endif()

if(DOG_PRIOR_BUILD_RESEARCH_TOOLS)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  add_executable(p47_reconstruct_frozen_sources
    ../../docs/p47_newer_college_coupled_shadow/tools/reconstruct_frozen_sources.cpp
    src/core/state_history.cpp
    src/core/imu_propagation.cpp
    src/core/imu_deskew.cpp
  )
  target_link_libraries(p47_reconstruct_frozen_sources ${PCL_LIBRARIES} ${LIBUSB_LIBRARIES})

  add_executable(p47_coupled_shadow_probe
    ../../docs/p47_newer_college_coupled_shadow/tools/coupled_shadow_probe.cpp
    ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_shadow.cpp
    ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_temporal.cpp
    ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_anchor.cpp
    ../dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_weak_refinement.cpp
    ../dog_prior_map_fastlio2_frontend_exp/src/registration_geometry.cpp
  )
  target_include_directories(p47_coupled_shadow_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dog_prior_map_fastlio2_frontend_exp/include
  )
  target_link_libraries(p47_coupled_shadow_probe
    ${PCL_LIBRARIES} ${OPENSSL_LIBRARIES} ${LIBUSB_LIBRARIES}
  )

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
  scripts/dataset_gt_path_publisher.py
  scripts/evaluate_runtime_and_accuracy.py
  scripts/build_livox_keyframe_scan_db.py
  scripts/monitor_p45_local_map_run.py
  scripts/ensure_output_dir_exec.py
  scripts/run_dataset_evaluation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch docs rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS dog_prior_map_ekf_node_cpp dog_prior_map_ndt_node_cpp
        newer_college_first_scan_publisher newer_college_sequence_publisher
        build_ndt_map_tiles solid_reference_database_builder solid_shadow_node_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(DOG_PRIOR_BUILD_RESEARCH_TOOLS)
  install(TARGETS p47_reconstruct_frozen_sources p47_coupled_shadow_probe
    ndt_uncertainty_probe ndt_direction_alignment_probe
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

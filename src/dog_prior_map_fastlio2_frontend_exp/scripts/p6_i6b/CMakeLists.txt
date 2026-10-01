cmake_minimum_required(VERSION 3.10)
project(p6_i6b_dual_reliability LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
option(BUILD_I6B_CLOSED_LOOP_TOOLS
  "Build the PCL/FAST-LIO2-backed frozen Floor01 replay runner" OFF)

find_package(Eigen3 REQUIRED)

add_library(p6_i6b_dual_reliability STATIC
  "${PACKAGE_ROOT}/src/reliability_metrics.cpp"
  "${PACKAGE_ROOT}/src/dual_reliability.cpp"
  "${PACKAGE_ROOT}/src/measurement_noise_model.cpp"
  "${PACKAGE_ROOT}/src/window_imu_factor.cpp"
  "${PACKAGE_ROOT}/src/lidar_deskew_geometry.cpp"
  "${PACKAGE_ROOT}/src/window_scan_processor.cpp"
  "${PACKAGE_ROOT}/src/window_lidar_factor.cpp"
  "${PACKAGE_ROOT}/src/window_visual_factor.cpp"
  "${PACKAGE_ROOT}/src/window_marginalization.cpp"
  "${PACKAGE_ROOT}/src/window_square_root.cpp"
  "${PACKAGE_ROOT}/src/window_square_root_marginalization.cpp"
  "${PACKAGE_ROOT}/src/fixed_lag_window.cpp"
  "${PACKAGE_ROOT}/src/window_linear_system.cpp"
  "${PACKAGE_ROOT}/src/fixed_lag_experiment.cpp"
  "${PACKAGE_ROOT}/src/fixed_lag_event_adapter.cpp")
target_compile_features(p6_i6b_dual_reliability PUBLIC cxx_std_17)
target_include_directories(p6_i6b_dual_reliability PUBLIC
  "${PACKAGE_ROOT}/include")
target_link_libraries(p6_i6b_dual_reliability PUBLIC Eigen3::Eigen)
target_compile_options(p6_i6b_dual_reliability PRIVATE -Wall -Wextra -Wpedantic)

add_executable(p6_i6b_dual_reliability_test
  "${PACKAGE_ROOT}/scripts/p6_i6b_reliability_test.cpp")
target_compile_features(p6_i6b_dual_reliability_test PRIVATE cxx_std_17)
target_link_libraries(p6_i6b_dual_reliability_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_i6b_dual_reliability_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(p6_i6e_conditional_compensation_test
  "${PACKAGE_ROOT}/scripts/p6_i6e_conditional_compensation_test.cpp")
target_compile_features(p6_i6e_conditional_compensation_test PRIVATE cxx_std_17)
target_link_libraries(p6_i6e_conditional_compensation_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_i6e_conditional_compensation_test PRIVATE
  -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(p6_a3e_r2_square_root_test "${PACKAGE_ROOT}/scripts/p6_a3e_r2_square_root_test.cpp")
target_link_libraries(p6_a3e_r2_square_root_test PRIVATE p6_i6b_dual_reliability Eigen3::Eigen)
add_test(NAME p6_a3e_r2_square_root COMMAND $<TARGET_FILE:p6_a3e_r2_square_root_test>)
add_executable(p6_a3d_r1_termination_test
  "${PACKAGE_ROOT}/scripts/p6_a3d_r1_termination_test.cpp")
target_link_libraries(p6_a3d_r1_termination_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
add_test(NAME p6_a3d_r1_termination
  COMMAND $<TARGET_FILE:p6_a3d_r1_termination_test>)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME p6_a3e_r2_replay_guard
  COMMAND ${Python3_EXECUTABLE} "${PACKAGE_ROOT}/scripts/p6_a3e_r2_replay_guard.py" --self-test)
add_test(NAME p6_a3e_r1_high_precision_oracle
  COMMAND ${Python3_EXECUTABLE}
    "${PACKAGE_ROOT}/scripts/p6_a3e_r1_tx155_schur_oracle.py" --self-test)
add_test(NAME p6_a3a_input_contract
  COMMAND ${Python3_EXECUTABLE} "${PACKAGE_ROOT}/scripts/p6_a3a_input_contract_test.py")
add_test(NAME p6_a2c_source_boundary_audit
  COMMAND ${Python3_EXECUTABLE} "${PACKAGE_ROOT}/scripts/p6_a2c_source_audit.py")
add_test(NAME p6_i6b_dual_reliability
  COMMAND $<TARGET_FILE:p6_i6b_dual_reliability_test>)
add_test(NAME p6_i6e_conditional_compensation
  COMMAND $<TARGET_FILE:p6_i6e_conditional_compensation_test>)

add_executable(p6_i6f_measurement_noise_test
  "${PACKAGE_ROOT}/scripts/p6_i6f_measurement_noise_test.cpp")
target_compile_features(p6_i6f_measurement_noise_test PRIVATE cxx_std_17)
target_link_libraries(p6_i6f_measurement_noise_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_i6f_measurement_noise_test PRIVATE
  -Wall -Wextra -Wpedantic)
add_test(NAME p6_i6f_measurement_noise
  COMMAND $<TARGET_FILE:p6_i6f_measurement_noise_test>)

add_executable(p6_i6f_joint_window_test
  "${PACKAGE_ROOT}/scripts/p6_i6f_joint_window_test.cpp")
target_compile_features(p6_i6f_joint_window_test PRIVATE cxx_std_17)
target_link_libraries(p6_i6f_joint_window_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_i6f_joint_window_test PRIVATE
  -Wall -Wextra -Wpedantic)
add_test(NAME p6_i6f_joint_window
  COMMAND $<TARGET_FILE:p6_i6f_joint_window_test>)

add_executable(p6_i6f_imu_preintegration_test
  "${PACKAGE_ROOT}/scripts/p6_i6f_imu_preintegration_test.cpp")
target_compile_features(p6_i6f_imu_preintegration_test PRIVATE cxx_std_17)
target_link_libraries(p6_i6f_imu_preintegration_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_i6f_imu_preintegration_test PRIVATE
  -Wall -Wextra -Wpedantic)
add_test(NAME p6_i6f_imu_preintegration
  COMMAND $<TARGET_FILE:p6_i6f_imu_preintegration_test>)

add_executable(p6_a1_r1_fixed_lag_regression_test
  "${PACKAGE_ROOT}/scripts/p6_a1_r1_fixed_lag_regression_test.cpp")
target_compile_features(p6_a1_r1_fixed_lag_regression_test PRIVATE cxx_std_17)
target_link_libraries(p6_a1_r1_fixed_lag_regression_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_a1_r1_fixed_lag_regression_test PRIVATE
  -Wall -Wextra -Wpedantic)
add_test(NAME p6_a1_r1_fixed_lag_regression
  COMMAND $<TARGET_FILE:p6_a1_r1_fixed_lag_regression_test>)

add_executable(p6_alg_integration_a2a_adapter_test
  "${PACKAGE_ROOT}/scripts/p6_alg_integration_a2a_adapter_test.cpp")
target_compile_features(p6_alg_integration_a2a_adapter_test PRIVATE cxx_std_17)
target_link_libraries(p6_alg_integration_a2a_adapter_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_alg_integration_a2a_adapter_test PRIVATE
  -Wall -Wextra -Wpedantic)
add_test(NAME p6_alg_integration_a2a_adapter
  COMMAND $<TARGET_FILE:p6_alg_integration_a2a_adapter_test>)

add_executable(p6_alg_integration_a2b_test
  "${PACKAGE_ROOT}/scripts/p6_alg_integration_a2b_test.cpp")
target_compile_features(p6_alg_integration_a2b_test PRIVATE cxx_std_17)
target_link_libraries(p6_alg_integration_a2b_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
target_compile_options(p6_alg_integration_a2b_test PRIVATE
  -Wall -Wextra -Wpedantic)
add_test(NAME p6_alg_integration_a2b
  COMMAND $<TARGET_FILE:p6_alg_integration_a2b_test>)

if(BUILD_I6B_CLOSED_LOOP_TOOLS)
  set(FASTLIO2_ROOT "" CACHE PATH "Pinned FAST-LIO2 source root")
  set(DCREG_ROOT "" CACHE PATH "Pinned DCReg source root")
  find_path(FASTLIO2_INCLUDE_DIR NAMES use-ikfom.hpp
    HINTS "${FASTLIO2_ROOT}" PATH_SUFFIXES include)
  find_path(DCREG_INCLUDE_DIR NAMES dcreg.hpp
    HINTS "${DCREG_ROOT}" PATH_SUFFIXES include)
  if(NOT FASTLIO2_INCLUDE_DIR OR NOT DCREG_INCLUDE_DIR)
    message(FATAL_ERROR "Set FASTLIO2_ROOT and DCREG_ROOT to the pinned checkouts")
  endif()
  find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
  find_package(OpenSSL REQUIRED)
  find_library(USB_1_LIBRARY NAMES usb-1.0 REQUIRED)

  add_executable(p6_a2c_initialization_test
    "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
    "${PACKAGE_ROOT}/scripts/p6_a2c_initialization_test.cpp")
  target_include_directories(p6_a2c_initialization_test PRIVATE
    "${PACKAGE_ROOT}/include" "${FASTLIO2_INCLUDE_DIR}")
  target_compile_features(p6_a2c_initialization_test PRIVATE cxx_std_17)
  target_compile_options(p6_a2c_initialization_test PRIVATE -fopenmp)
  target_link_libraries(p6_a2c_initialization_test PRIVATE Eigen3::Eigen gomp)
  add_test(NAME p6_a2c_initialization
    COMMAND $<TARGET_FILE:p6_a2c_initialization_test>)

  foreach(legacy_scan_test scan_end_deskew_runtime_test frontend_runtime_end_to_end_test)
    add_executable(${legacy_scan_test}
      "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
      "${PACKAGE_ROOT}/src/scan_processor.cpp"
      "${PACKAGE_ROOT}/src/lidar_deskew_geometry.cpp"
      "${PACKAGE_ROOT}/src/frontend_runtime.cpp"
      "${PACKAGE_ROOT}/tests/${legacy_scan_test}.cpp")
    target_include_directories(${legacy_scan_test} PRIVATE "${FASTLIO2_INCLUDE_DIR}")
    target_compile_features(${legacy_scan_test} PRIVATE cxx_std_17)
    target_compile_options(${legacy_scan_test} PRIVATE -fopenmp)
    target_link_libraries(${legacy_scan_test} PRIVATE p6_i6b_dual_reliability Eigen3::Eigen gomp)
    add_test(NAME ${legacy_scan_test} COMMAND $<TARGET_FILE:${legacy_scan_test}>)
  endforeach()

  add_executable(p6_i6b_closed_loop
    "${PACKAGE_ROOT}/scripts/p6_i1_branched_recovery.cpp")
  target_compile_features(p6_i6b_closed_loop PRIVATE cxx_std_17)
  target_include_directories(p6_i6b_closed_loop PRIVATE
    "${PACKAGE_ROOT}/include"
    "${PACKAGE_ROOT}/scripts"
    "${FASTLIO2_INCLUDE_DIR}"
    "${DCREG_INCLUDE_DIR}"
    ${PCL_INCLUDE_DIRS})
  target_link_directories(p6_i6b_closed_loop PRIVATE ${PCL_LIBRARY_DIRS})
  target_compile_options(p6_i6b_closed_loop PRIVATE
    ${PCL_DEFINITIONS} -fopenmp -Wall -Wextra -Wpedantic)
  target_link_libraries(p6_i6b_closed_loop PRIVATE
    p6_i6b_dual_reliability Eigen3::Eigen ${PCL_LIBRARIES}
    OpenSSL::Crypto ${USB_1_LIBRARY} gomp)
  add_test(NAME p6_a2c_real_pcl_producer
    COMMAND $<TARGET_FILE:p6_i6b_closed_loop> A2C_FIXTURE)
  add_test(NAME p6_a2d_window_owned_pcl_producer
    COMMAND $<TARGET_FILE:p6_i6b_closed_loop> A2D_FIXTURE)
  add_test(NAME p6_a3b_no_vision
    COMMAND $<TARGET_FILE:p6_i6b_closed_loop> A3B_NO_VISION_FIXTURE)

  add_executable(p6_a3b_r1_optimizer_diagnostics_test
    "${PACKAGE_ROOT}/scripts/p6_a3b_r1_optimizer_diagnostics_test.cpp")
  target_compile_features(p6_a3b_r1_optimizer_diagnostics_test PRIVATE cxx_std_17)
  target_link_libraries(p6_a3b_r1_optimizer_diagnostics_test PRIVATE
    p6_i6b_dual_reliability Eigen3::Eigen)
  target_compile_options(p6_a3b_r1_optimizer_diagnostics_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME p6_a3b_r1_optimizer_diagnostics
    COMMAND $<TARGET_FILE:p6_a3b_r1_optimizer_diagnostics_test>)

  add_executable(p6_a3b_r2_frozen_projection_test
    "${PACKAGE_ROOT}/scripts/p6_a3b_r2_frozen_projection_test.cpp")
  target_compile_features(p6_a3b_r2_frozen_projection_test PRIVATE cxx_std_17)
  target_link_libraries(p6_a3b_r2_frozen_projection_test PRIVATE
    p6_i6b_dual_reliability Eigen3::Eigen)
  target_compile_options(p6_a3b_r2_frozen_projection_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME p6_a3b_r2_frozen_projection
    COMMAND $<TARGET_FILE:p6_a3b_r2_frozen_projection_test>)

  add_executable(p6_a3c_r1_marginalization_diagnostics_test
    "${PACKAGE_ROOT}/scripts/p6_a3c_r1_marginalization_diagnostics_test.cpp")
  target_compile_features(p6_a3c_r1_marginalization_diagnostics_test PRIVATE
    cxx_std_17)
  target_link_libraries(p6_a3c_r1_marginalization_diagnostics_test PRIVATE
    p6_i6b_dual_reliability Eigen3::Eigen)
  target_compile_options(p6_a3c_r1_marginalization_diagnostics_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME p6_a3c_r1_marginalization_diagnostics
    COMMAND $<TARGET_FILE:p6_a3c_r1_marginalization_diagnostics_test>)
  add_test(NAME p6_a3c_r1_marginalization_oracle_math
    COMMAND ${Python3_EXECUTABLE}
      "${PACKAGE_ROOT}/scripts/p6_a3c_r1_marginalization_oracle.py"
      --self-test)

  add_executable(p6_a3c_r2_alias_regression_test
    "${PACKAGE_ROOT}/scripts/p6_a3c_r2_alias_regression_test.cpp")
  target_compile_features(p6_a3c_r2_alias_regression_test PRIVATE cxx_std_17)
  target_link_libraries(p6_a3c_r2_alias_regression_test PRIVATE
    p6_i6b_dual_reliability Eigen3::Eigen)
  target_compile_options(p6_a3c_r2_alias_regression_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME p6_a3c_r2_alias_regression
    COMMAND $<TARGET_FILE:p6_a3c_r2_alias_regression_test>)
  add_test(NAME p6_a3c_r2_source_alias_inventory
    COMMAND ${Python3_EXECUTABLE}
      "${PACKAGE_ROOT}/scripts/p6_a3c_r2_alias_audit.py" --inventory)
  add_test(NAME p6_a3c_r2_tx90_capsule_repair
    COMMAND ${Python3_EXECUTABLE}
      "${PACKAGE_ROOT}/scripts/p6_a3c_r2_alias_audit.py"
      --capsule-inspector $<TARGET_FILE:p6_a3c_r1_marginalization_diagnostics_test>)

  add_executable(p6_i6e_projected_pose_consistency_test
    "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
    "${PACKAGE_ROOT}/tests/ndt_pose_update_runtime_test.cpp")
  target_compile_features(p6_i6e_projected_pose_consistency_test PRIVATE cxx_std_17)
  target_include_directories(p6_i6e_projected_pose_consistency_test PRIVATE
    "${PACKAGE_ROOT}/include"
    "${FASTLIO2_INCLUDE_DIR}")
  target_compile_options(p6_i6e_projected_pose_consistency_test PRIVATE -fopenmp)
  target_compile_definitions(p6_i6e_projected_pose_consistency_test PRIVATE
    DOG_PRIOR_MAP_ENABLE_TEST_HOOKS)
  target_link_libraries(p6_i6e_projected_pose_consistency_test PRIVATE
    Eigen3::Eigen gomp)
  add_test(NAME p6_i6e_projected_pose_consistency
    COMMAND $<TARGET_FILE:p6_i6e_projected_pose_consistency_test>)

  add_executable(p6_i6e_r3_residual_jacobian_test
    "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
    "${PACKAGE_ROOT}/scripts/p6_i6e_r3_residual_jacobian_test.cpp")
  target_compile_features(p6_i6e_r3_residual_jacobian_test PRIVATE cxx_std_17)
  target_include_directories(p6_i6e_r3_residual_jacobian_test PRIVATE
    "${PACKAGE_ROOT}/include"
    "${PACKAGE_ROOT}/scripts"
    "${FASTLIO2_INCLUDE_DIR}")
  target_compile_options(p6_i6e_r3_residual_jacobian_test PRIVATE -fopenmp)
  target_link_libraries(p6_i6e_r3_residual_jacobian_test PRIVATE
    p6_i6b_dual_reliability Eigen3::Eigen gomp)
  add_test(NAME p6_i6e_r3_residual_jacobian
    COMMAND $<TARGET_FILE:p6_i6e_r3_residual_jacobian_test>)
endif()

add_executable(p6_alg_integration_a2c_test
  "${PACKAGE_ROOT}/scripts/p6_alg_integration_a2c_test.cpp")
target_link_libraries(p6_alg_integration_a2c_test PRIVATE
  p6_i6b_dual_reliability Eigen3::Eigen)
add_test(NAME p6_alg_integration_a2c
  COMMAND $<TARGET_FILE:p6_alg_integration_a2c_test>)

add_executable(window_deskew_math_test "${PACKAGE_ROOT}/scripts/window_deskew_math_test.cpp")
target_link_libraries(window_deskew_math_test PRIVATE p6_i6b_dual_reliability)
add_test(NAME window_deskew_math_test COMMAND $<TARGET_FILE:window_deskew_math_test>)
foreach(a2d_test dense_sparse_equivalence_test solver_scaling_sanity_test measurement_provenance_test sparse_marginal_covariance_test)
  add_executable(${a2d_test} "${PACKAGE_ROOT}/scripts/${a2d_test}.cpp")
  target_link_libraries(${a2d_test} PRIVATE p6_i6b_dual_reliability)
  add_test(NAME ${a2d_test} COMMAND $<TARGET_FILE:${a2d_test}>)
endforeach()

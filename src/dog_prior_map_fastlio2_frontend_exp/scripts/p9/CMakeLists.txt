cmake_minimum_required(VERSION 3.10)
project(p9_ndt_energy_contract LANGUAGES CXX)

find_package(PCL 1.10 REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(p9_ndt_energy_contract p9_ndt_energy_contract.cpp)
add_executable(p9_true_profile_closure p9_true_profile_closure.cpp)
add_executable(p9_strong_attractor_closure p9_strong_attractor_closure.cpp)
add_executable(p9_support_aware_continuation p9_support_aware_continuation.cpp)
add_executable(p9_numerical_stationarity p9_numerical_stationarity.cpp)
add_executable(p9_h2_matched_discovery p9_h2_matched_discovery.cpp)
add_executable(p9_secondgen_branch_gate p9_secondgen_branch_gate.cpp)
add_executable(p9_r1c3a_contract_closure p9_r1c3a_contract_closure.cpp)
add_executable(p9_discrete_support_evidence p9_discrete_support_evidence.cpp)
add_executable(p9_r2_terminal_stability p9_r2_terminal_stability.cpp)
add_executable(p9_r2a_predictor_search p9_r2a_predictor_search.cpp)
add_executable(p9_r4_ndt p9_r4_ndt.cpp)
add_executable(p9_r4_source_audit p9_r4_source_audit.cpp)
foreach(p9_target p9_ndt_energy_contract p9_true_profile_closure p9_strong_attractor_closure p9_support_aware_continuation p9_numerical_stationarity p9_h2_matched_discovery p9_secondgen_branch_gate p9_r1c3a_contract_closure p9_discrete_support_evidence p9_r2_terminal_stability p9_r2a_predictor_search p9_r4_ndt p9_r4_source_audit)
  target_compile_features(${p9_target} PRIVATE cxx_std_14)
  target_include_directories(${p9_target} PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${LIBUSB_INCLUDE_DIRS})
  target_compile_definitions(${p9_target} PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(${p9_target} PRIVATE ${PCL_LIBRARIES} ${LIBUSB_LIBRARIES})
endforeach()
target_include_directories(p9_r4_ndt PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/r4_frozen
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

add_library(p9_r2b_chart SHARED p9_r2b_chart.cpp)
add_executable(p9_r2b_chart_self_test p9_r2b_chart.cpp)
foreach(math_target p9_r2b_chart p9_r2b_chart_self_test)
  target_compile_features(${math_target} PRIVATE cxx_std_14)
  target_include_directories(${math_target} PRIVATE ${EIGEN3_INCLUDE_DIRS})
endforeach()
target_compile_definitions(p9_r2b_chart_self_test PRIVATE P9_R2B_CHART_TEST_MAIN)

execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/p9_r4_codegen.py
  ${CMAKE_CURRENT_BINARY_DIR}/r4_frozen RESULT_VARIABLE r4_codegen_status)
if(NOT r4_codegen_status EQUAL 0)
  message(FATAL_ERROR "R4 frozen source extraction failed")
endif()
add_library(p9_r4_oracle_geometry SHARED p9_r4_oracle_geometry.cpp)
add_executable(p9_r4_oracle_geometry_self_test p9_r4_oracle_geometry.cpp)
foreach(math_target p9_r4_oracle_geometry p9_r4_oracle_geometry_self_test)
  target_compile_features(${math_target} PRIVATE cxx_std_14)
  target_include_directories(${math_target} PRIVATE ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/r4_frozen)
endforeach()
target_compile_definitions(p9_r4_oracle_geometry_self_test PRIVATE P9_R4_ORACLE_GEOMETRY_TEST_MAIN)

enable_testing()
add_test(NAME p9_r4_selection_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r4_contract.py self-test)
add_test(NAME p9_r4_oracle_geometry_self_test COMMAND p9_r4_oracle_geometry_self_test)
add_test(NAME p9_r4_ndt_proposal_self_test COMMAND p9_r4_ndt --self-test)
set_tests_properties(p9_r4_ndt_proposal_self_test PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu")
add_test(NAME p9_r4_oracle_label_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r4_oracle_labels.py self-test --library $<TARGET_FILE:p9_r4_oracle_geometry>)
add_test(NAME p9_r4_blind_freeze_chain_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_r4_evidence.py self-test)
add_test(NAME p9_r3b_admission_boundary_cheirality_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r3b_pnp_frontend.py)
add_test(NAME p9_r3b_information_order_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/evaluate_r3b_pnp_admission.py self-test)
add_test(NAME p9_r3a_depth_geometry_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r3a_depth_completion.py)
add_test(NAME p9_r3a_direct_pnp_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r3a_visual_frontend.py)
add_test(NAME p9_r3a_stage_gate_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/evaluate_r3a_metric_depth_coverage.py self-test)
add_test(NAME p9_r3_visual_contract_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r3_visual_contract.py)
add_test(NAME p9_r3_blind_evidence_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/build_r3_visual_evidence.py self-test)
add_test(NAME p9_r3_frame_statistics_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/evaluate_r3_visual_evidence.py self-test)
add_test(NAME p9_r2b_chart_self_test COMMAND p9_r2b_chart_self_test)
add_test(NAME p9_r2b_evidence_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/p9_r2b_nonoracle_evidence.py self-test --library $<TARGET_FILE:p9_r2b_chart>)
add_test(NAME p9_r2b_statistics_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/evaluate_r2b_nonoracle_evidence.py self-test)
add_test(NAME p9_r1c3_self_test COMMAND p9_secondgen_branch_gate --self-test)
add_test(NAME p9_r1c3a_contract_self_test COMMAND p9_r1c3a_contract_closure --self-test)
add_test(NAME p9_r1c3b_discrete_support_self_test COMMAND p9_discrete_support_evidence --self-test)
add_test(NAME p9_r1c3b_statistics_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_r1c3b_discrete_support_evidence.py --self-test)
add_test(NAME p9_r2_terminal_stability_self_test COMMAND p9_r2_terminal_stability --self-test)
add_test(NAME p9_r2_terminal_stability_statistics_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_r2_terminal_stability.py self-test)
add_test(NAME p9_r2a_proposal_self_test COMMAND p9_r2a_predictor_search --self-test)
add_test(NAME p9_r2a_orchestration_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_r2a_predictor_search.py self-test)
add_test(NAME p9_r2a_statistics_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/summarize_r2a_predictor_search.py self-test)
add_test(NAME p9_r1c3a_audit_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_r1c3a_contract_closure.py self-test)
add_test(NAME p9_r1c3_audit_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_r1c3_branch_gate.py self-test)
set_tests_properties(p9_r1c3_self_test PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu")
add_test(NAME p9_h2_proposal_self_test COMMAND p9_h2_matched_discovery --self-test)
add_test(NAME p9_h2_statistics_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_h2_discovery.py self-test)
add_test(NAME p9_math_self_test COMMAND p9_ndt_energy_contract --self-test)
add_test(NAME p9_r1a_self_test COMMAND p9_true_profile_closure --self-test)
add_test(NAME p9_r1b_self_test COMMAND p9_strong_attractor_closure --self-test)
add_test(NAME p9_r1c_self_test COMMAND p9_support_aware_continuation --self-test)
add_test(NAME p9_r1c2_numerical_self_test COMMAND p9_numerical_stationarity --self-test)
add_test(NAME p9_r1c2_multih_audit_self_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_numerical_stationarity.py --self-test)
add_test(NAME p9_r1c_certificate_audit COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_support_aware_continuation.py --self-test)
add_test(NAME p9_r1b_orchestration_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_strong_attractor_experiment.py --self-test)
add_test(NAME p9_r1b_inference_regression COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/summarize_strong_attractor_closure.py --self-test)
set_tests_properties(p9_math_self_test p9_r1a_self_test p9_r1b_self_test p9_r1c_self_test p9_r1c2_numerical_self_test p9_r1c3b_discrete_support_self_test p9_r2_terminal_stability_self_test p9_r2a_proposal_self_test PROPERTIES
  ENVIRONMENT "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu")

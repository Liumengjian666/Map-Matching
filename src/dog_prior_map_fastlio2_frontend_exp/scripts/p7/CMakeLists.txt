cmake_minimum_required(VERSION 3.10)
project(p7_single_state_baseline LANGUAGES C CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(FASTLIO2_ROOT "" CACHE PATH "Existing pinned FAST-LIO2 checkout")
if(NOT FASTLIO2_ROOT)
  message(FATAL_ERROR "Set FASTLIO2_ROOT to the existing pinned checkout")
endif()
execute_process(COMMAND git -C "${FASTLIO2_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE FASTLIO2_SHA OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE PIN_RESULT)
if(NOT PIN_RESULT EQUAL 0 OR NOT FASTLIO2_SHA STREQUAL "7cc4175de6f8ba2edf34bab02a42195b141027e9")
  message(FATAL_ERROR "FAST-LIO2 pin mismatch: ${FASTLIO2_SHA}")
endif()
find_path(FASTLIO2_INCLUDE_DIR use-ikfom.hpp HINTS "${FASTLIO2_ROOT}/include" NO_DEFAULT_PATH)
if(NOT FASTLIO2_INCLUDE_DIR)
  message(FATAL_ERROR "Pinned FAST-LIO2 header missing")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(p7_reliability_metrics STATIC "${PACKAGE_ROOT}/src/reliability_metrics.cpp")
target_include_directories(p7_reliability_metrics PUBLIC "${PACKAGE_ROOT}/include")
target_link_libraries(p7_reliability_metrics PUBLIC Eigen3::Eigen)
target_compile_options(p7_reliability_metrics PRIVATE -Wall -Wextra -Wpedantic)

add_library(p7_current_frame_ndt STATIC "${PACKAGE_ROOT}/src/current_frame_ndt.cpp")
target_include_directories(p7_current_frame_ndt PUBLIC "${PACKAGE_ROOT}/include" PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(p7_current_frame_ndt PUBLIC Eigen3::Eigen p7_reliability_metrics PRIVATE ${PCL_LIBRARIES})
target_compile_options(p7_current_frame_ndt PRIVATE ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

enable_testing()
find_program(P7_PYTHON3_EXECUTABLE NAMES python3)
if(P7_PYTHON3_EXECUTABLE)
  add_test(NAME p7_shadow_reporting_test COMMAND "${P7_PYTHON3_EXECUTABLE}"
    "${PACKAGE_ROOT}/tests/p7_shadow_reporting_test.py")
  add_test(NAME p7_solution_remapping_reporting_test COMMAND "${P7_PYTHON3_EXECUTABLE}"
    "${PACKAGE_ROOT}/tests/p7_solution_remapping_reporting_test.py")
  add_test(NAME p7_admission_reporting_test COMMAND "${P7_PYTHON3_EXECUTABLE}"
    "${PACKAGE_ROOT}/tests/p7_admission_reporting_test.py")
endif()
add_executable(p7_fixed_physical_uobs_test "${PACKAGE_ROOT}/tests/p7_fixed_physical_uobs_test.cpp")
target_link_libraries(p7_fixed_physical_uobs_test PRIVATE p7_reliability_metrics)
add_test(NAME p7_fixed_physical_uobs_test COMMAND p7_fixed_physical_uobs_test)
add_executable(current_frame_ndt_test "${PACKAGE_ROOT}/tests/current_frame_ndt_test.cpp")
target_include_directories(current_frame_ndt_test PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(current_frame_ndt_test PRIVATE p7_current_frame_ndt ${PCL_LIBRARIES})
add_test(NAME current_frame_ndt_test COMMAND current_frame_ndt_test)

add_library(p7_replay_io STATIC "${PACKAGE_ROOT}/src/p7_replay_io.cpp")
target_include_directories(p7_replay_io PUBLIC "${PACKAGE_ROOT}/include")
target_link_libraries(p7_replay_io PUBLIC Eigen3::Eigen)
target_compile_options(p7_replay_io PRIVATE -Wall -Wextra -Wpedantic)
add_executable(p7_replay_io_test "${PACKAGE_ROOT}/tests/p7_replay_io_test.cpp")
target_link_libraries(p7_replay_io_test PRIVATE p7_replay_io)
add_test(NAME p7_replay_io_test COMMAND p7_replay_io_test)

add_library(p7_solution_remapping_baseline STATIC
  "${PACKAGE_ROOT}/src/solution_remapping_baseline.cpp"
  "${PACKAGE_ROOT}/src/registration_geometry.cpp")
target_include_directories(p7_solution_remapping_baseline PUBLIC "${PACKAGE_ROOT}/include")
target_link_libraries(p7_solution_remapping_baseline PUBLIC Eigen3::Eigen)
target_compile_options(p7_solution_remapping_baseline PRIVATE -Wall -Wextra -Wpedantic)
add_executable(solution_remapping_baseline_test "${PACKAGE_ROOT}/tests/solution_remapping_baseline_test.cpp")
target_link_libraries(solution_remapping_baseline_test PRIVATE p7_solution_remapping_baseline p7_reliability_metrics)
add_test(NAME solution_remapping_baseline_test COMMAND solution_remapping_baseline_test)

add_executable(p7_single_state_runner p7_single_state_runner.cpp
  "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
  "${PACKAGE_ROOT}/src/mature_measurement_admission.cpp")
target_include_directories(p7_single_state_runner PRIVATE "${FASTLIO2_INCLUDE_DIR}")
target_link_libraries(p7_single_state_runner PRIVATE
  p7_current_frame_ndt p7_replay_io p7_solution_remapping_baseline Eigen3::Eigen OpenMP::OpenMP_CXX)

add_executable(mature_measurement_admission_test
  "${PACKAGE_ROOT}/tests/mature_measurement_admission_test.cpp"
  "${PACKAGE_ROOT}/src/mature_measurement_admission.cpp"
  "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp")
target_include_directories(mature_measurement_admission_test PRIVATE
  "${PACKAGE_ROOT}/include" "${FASTLIO2_INCLUDE_DIR}")
target_link_libraries(mature_measurement_admission_test PRIVATE
  p7_solution_remapping_baseline Eigen3::Eigen OpenMP::OpenMP_CXX)
add_test(NAME mature_measurement_admission_test COMMAND mature_measurement_admission_test)

# Posthoc frozen-measurement audit: no PCL registration object or GT input.
add_executable(p7_frozen_full_pose_innovation_audit p7_frozen_full_pose_innovation_audit.cpp
  "${PACKAGE_ROOT}/src/mature_measurement_admission.cpp"
  "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp")
target_include_directories(p7_frozen_full_pose_innovation_audit PRIVATE "${FASTLIO2_INCLUDE_DIR}")
target_link_libraries(p7_frozen_full_pose_innovation_audit PRIVATE
  p7_replay_io p7_solution_remapping_baseline Eigen3::Eigen OpenMP::OpenMP_CXX)

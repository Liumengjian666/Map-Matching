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
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(p7_current_frame_ndt STATIC "${PACKAGE_ROOT}/src/current_frame_ndt.cpp"
  "${PACKAGE_ROOT}/src/coupled_ndt_shadow.cpp" "${PACKAGE_ROOT}/src/coupled_ndt_temporal.cpp"
  "${PACKAGE_ROOT}/src/registration_geometry.cpp" "${PACKAGE_ROOT}/src/coupled_ndt_anchor.cpp"
  "${PACKAGE_ROOT}/src/coupled_ndt_weak_refinement.cpp")
target_include_directories(p7_current_frame_ndt PUBLIC "${PACKAGE_ROOT}/include" PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(p7_current_frame_ndt PUBLIC Eigen3::Eigen PRIVATE ${PCL_LIBRARIES})
target_compile_options(p7_current_frame_ndt PRIVATE ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(current_frame_ndt_test "${PACKAGE_ROOT}/tests/current_frame_ndt_test.cpp")
target_include_directories(current_frame_ndt_test PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(current_frame_ndt_test PRIVATE
  p7_current_frame_ndt ${PCL_LIBRARIES} ${USB_10_LIBRARY})
add_test(NAME current_frame_ndt_test COMMAND current_frame_ndt_test)

add_library(p7_replay_io STATIC "${PACKAGE_ROOT}/src/p7_replay_io.cpp")
target_include_directories(p7_replay_io PUBLIC "${PACKAGE_ROOT}/include")
target_link_libraries(p7_replay_io PUBLIC Eigen3::Eigen)
target_compile_options(p7_replay_io PRIVATE -Wall -Wextra -Wpedantic)
add_executable(p7_replay_io_test "${PACKAGE_ROOT}/tests/p7_replay_io_test.cpp")
target_link_libraries(p7_replay_io_test PRIVATE p7_replay_io)
add_test(NAME p7_replay_io_test COMMAND p7_replay_io_test)

add_executable(p7_single_state_runner p7_single_state_runner.cpp
  "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
  "${PACKAGE_ROOT}/src/registration_geometry.cpp"
  "${PACKAGE_ROOT}/src/scan_processor.cpp"
  "${PACKAGE_ROOT}/src/lidar_deskew_geometry.cpp")
target_include_directories(p7_single_state_runner PRIVATE "${FASTLIO2_INCLUDE_DIR}")
target_link_libraries(p7_single_state_runner PRIVATE
  p7_current_frame_ndt p7_replay_io Eigen3::Eigen OpenMP::OpenMP_CXX
  ${PCL_LIBRARIES} ${USB_10_LIBRARY})

set(P10_BUDGETED_ROOT "${PACKAGE_ROOT}/scripts/p10/budgeted")
add_executable(p10_r2_single "${P10_BUDGETED_ROOT}/p10_r2_single.cpp")
target_link_libraries(p10_r2_single PRIVATE p7_current_frame_ndt p7_replay_io
  Eigen3::Eigen ${PCL_LIBRARIES} ${USB_10_LIBRARY})

add_executable(p10_r2_replay "${P10_BUDGETED_ROOT}/p10_r2_replay.cpp"
  "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
  "${PACKAGE_ROOT}/src/registration_geometry.cpp"
  "${PACKAGE_ROOT}/src/scan_processor.cpp"
  "${PACKAGE_ROOT}/src/lidar_deskew_geometry.cpp")
target_include_directories(p10_r2_replay PRIVATE "${FASTLIO2_INCLUDE_DIR}")
target_link_libraries(p10_r2_replay PRIVATE p7_current_frame_ndt p7_replay_io
  Eigen3::Eigen OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${USB_10_LIBRARY})

add_executable(p10_r2_shadow_test "${P10_BUDGETED_ROOT}/test_shadow.cpp")
target_link_libraries(p10_r2_shadow_test PRIVATE p7_current_frame_ndt Eigen3::Eigen)
add_test(NAME p10_r2_shadow_test COMMAND p10_r2_shadow_test)
add_executable(p10_r3_temporal_test "${P10_BUDGETED_ROOT}/test_temporal.cpp")
target_link_libraries(p10_r3_temporal_test PRIVATE p7_current_frame_ndt Eigen3::Eigen)
add_test(NAME p10_r3_temporal_test COMMAND p10_r3_temporal_test)
add_executable(p10_r4_admission_test "${P10_BUDGETED_ROOT}/test_admission.cpp")
target_link_libraries(p10_r4_admission_test PRIVATE p7_current_frame_ndt Eigen3::Eigen)
add_test(NAME p10_r4_admission_test COMMAND p10_r4_admission_test)
add_executable(p10_r5_anchor_test "${P10_BUDGETED_ROOT}/test_anchor.cpp")
target_link_libraries(p10_r5_anchor_test PRIVATE p7_current_frame_ndt Eigen3::Eigen)
add_test(NAME p10_r5_anchor_test COMMAND p10_r5_anchor_test)
add_executable(p10_r6_weak_refinement_test "${P10_BUDGETED_ROOT}/test_weak_refinement.cpp")
target_link_libraries(p10_r6_weak_refinement_test PRIVATE p7_current_frame_ndt Eigen3::Eigen)
add_test(NAME p10_r6_weak_refinement_test COMMAND p10_r6_weak_refinement_test)
set_tests_properties(current_frame_ndt_test p7_replay_io_test p10_r2_shadow_test
  p10_r3_temporal_test p10_r4_admission_test
  PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu")

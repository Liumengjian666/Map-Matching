cmake_minimum_required(VERSION 3.10)
project(p8_official_pose_candidate_eval LANGUAGES CXX)

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
find_package(PCL 1.10.0 EXACT REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenMP REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_library(USB_10_LIBRARY usb-1.0 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(p8_baseline_registration STATIC
  "${PACKAGE_ROOT}/src/current_frame_ndt.cpp"
  "${PACKAGE_ROOT}/src/p7_replay_io.cpp")
target_include_directories(p8_baseline_registration PUBLIC
  "${PACKAGE_ROOT}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(p8_baseline_registration PUBLIC
  Eigen3::Eigen ${PCL_LIBRARIES} ${USB_10_LIBRARY})
target_compile_options(p8_baseline_registration PRIVATE ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

add_executable(p8_official_pose_candidate_eval official_pose_candidate_eval.cpp)
target_include_directories(p8_official_pose_candidate_eval PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(p8_official_pose_candidate_eval PRIVATE
  p8_baseline_registration ${PCL_LIBRARIES})
target_compile_options(p8_official_pose_candidate_eval PRIVATE ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

add_executable(p8_tx666_deskew_export tx666_deskew_export.cpp
  "${PACKAGE_ROOT}/src/fastlio2_frontend_ikfom.cpp"
  "${PACKAGE_ROOT}/src/registration_geometry.cpp"
  "${PACKAGE_ROOT}/src/scan_processor.cpp"
  "${PACKAGE_ROOT}/src/lidar_deskew_geometry.cpp")
target_include_directories(p8_tx666_deskew_export PRIVATE
  "${PACKAGE_ROOT}/include" "${FASTLIO2_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS})
target_link_libraries(p8_tx666_deskew_export PRIVATE
  p8_baseline_registration Eigen3::Eigen OpenMP::OpenMP_CXX
  ${PCL_LIBRARIES} ${USB_10_LIBRARY})
target_compile_options(p8_tx666_deskew_export PRIVATE
  ${PCL_DEFINITIONS} ${OpenMP_CXX_FLAGS} -Wall -Wextra -Wpedantic)

add_executable(p8_public_raw_tx666_ndt public_raw_tx666_ndt.cpp)
target_include_directories(p8_public_raw_tx666_ndt PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(p8_public_raw_tx666_ndt PRIVATE
  p8_baseline_registration ${PCL_LIBRARIES})
target_compile_options(p8_public_raw_tx666_ndt PRIVATE
  ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME p8_tx666_rviz_pcd_parser_test
  COMMAND ${PYTHON_EXECUTABLE}
    "${PACKAGE_ROOT}/tests/p8_tx666_rviz_pcd_parser_test.py")

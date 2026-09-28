cmake_minimum_required(VERSION 3.10)
project(p6_i6a_convergence_control LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(FASTLIO2_ROOT "/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
set(DCREG_ROOT "/home/jian/livox_ws/DCReg/DCReg")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(OpenSSL REQUIRED)
find_library(USB_1_LIBRARY NAMES usb-1.0 REQUIRED)

add_library(p6_i6a_reliability_metrics STATIC
  "${PACKAGE_ROOT}/src/reliability_metrics.cpp")
target_compile_features(p6_i6a_reliability_metrics PUBLIC cxx_std_17)
target_include_directories(p6_i6a_reliability_metrics PUBLIC
  "${PACKAGE_ROOT}/include"
  ${EIGEN3_INCLUDE_DIRS})
target_compile_options(p6_i6a_reliability_metrics PRIVATE -Wall -Wextra -Wpedantic)

add_executable(p6_i6a_reliability_metrics_test
  "${PACKAGE_ROOT}/scripts/p6_i6a_reliability_metrics_test.cpp")
target_link_libraries(p6_i6a_reliability_metrics_test PRIVATE p6_i6a_reliability_metrics)
target_compile_options(p6_i6a_reliability_metrics_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME p6_i6a_reliability_metrics
  COMMAND $<TARGET_FILE:p6_i6a_reliability_metrics_test>)

add_executable(p6_i6a_strict_frame_comparison
  "${PACKAGE_ROOT}/scripts/p6_i6a_strict_frame_comparison.cpp")
target_compile_features(p6_i6a_strict_frame_comparison PRIVATE cxx_std_17)
target_include_directories(p6_i6a_strict_frame_comparison PRIVATE
  "${PACKAGE_ROOT}/include"
  "${FASTLIO2_ROOT}/include"
  "${DCREG_ROOT}/include"
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS})
target_link_directories(p6_i6a_strict_frame_comparison PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(p6_i6a_strict_frame_comparison PRIVATE
  ${PCL_DEFINITIONS} -fopenmp -Wall -Wextra -Wpedantic)
target_link_libraries(p6_i6a_strict_frame_comparison PRIVATE
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  ${USB_1_LIBRARY}
  gomp)

add_executable(p6_i6a_score_gradient_check
  "${PACKAGE_ROOT}/scripts/p6_i6a_score_gradient_check.cpp")
target_compile_features(p6_i6a_score_gradient_check PRIVATE cxx_std_17)
target_include_directories(p6_i6a_score_gradient_check PRIVATE
  "${PACKAGE_ROOT}/include"
  "${FASTLIO2_ROOT}/include"
  "${DCREG_ROOT}/include"
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS})
target_link_directories(p6_i6a_score_gradient_check PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(p6_i6a_score_gradient_check PRIVATE
  ${PCL_DEFINITIONS} -fopenmp -Wall -Wextra -Wpedantic)
target_link_libraries(p6_i6a_score_gradient_check PRIVATE
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  ${USB_1_LIBRARY}
  gomp)

cmake_minimum_required(VERSION 3.10)
project(p6_i6b_dual_reliability LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
option(BUILD_I6B_CLOSED_LOOP_TOOLS
  "Build the PCL/FAST-LIO2-backed frozen Floor01 replay runner" OFF)

find_package(Eigen3 REQUIRED)

add_library(p6_i6b_dual_reliability STATIC
  "${PACKAGE_ROOT}/src/reliability_metrics.cpp"
  "${PACKAGE_ROOT}/src/dual_reliability.cpp")
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

enable_testing()
add_test(NAME p6_i6b_dual_reliability
  COMMAND $<TARGET_FILE:p6_i6b_dual_reliability_test>)

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
endif()

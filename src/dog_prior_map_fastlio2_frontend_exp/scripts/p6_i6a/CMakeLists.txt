cmake_minimum_required(VERSION 3.10)
project(p6_i6a_convergence_control LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
option(BUILD_I6A_EXPERIMENT_TOOLS
  "Build the PCL-backed I6A data/derivative tools" OFF)

find_package(Eigen3 REQUIRED)

add_library(p6_i6a_reliability_metrics STATIC
  "${PACKAGE_ROOT}/src/reliability_metrics.cpp")
target_compile_features(p6_i6a_reliability_metrics PUBLIC cxx_std_17)
target_include_directories(p6_i6a_reliability_metrics PUBLIC
  "${PACKAGE_ROOT}/include")
target_link_libraries(p6_i6a_reliability_metrics PUBLIC Eigen3::Eigen)
target_compile_options(p6_i6a_reliability_metrics PRIVATE -Wall -Wextra -Wpedantic)

add_executable(p6_i6a_reliability_metrics_test
  "${PACKAGE_ROOT}/scripts/p6_i6a_reliability_metrics_test.cpp")
target_link_libraries(p6_i6a_reliability_metrics_test PRIVATE
  p6_i6a_reliability_metrics Eigen3::Eigen)
target_compile_options(p6_i6a_reliability_metrics_test PRIVATE
  -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME p6_i6a_reliability_metrics
  COMMAND $<TARGET_FILE:p6_i6a_reliability_metrics_test>)

if(BUILD_I6A_EXPERIMENT_TOOLS)
  set(FASTLIO2_ROOT "" CACHE PATH "FAST-LIO2 source root")
  set(DCREG_ROOT "" CACHE PATH "DCReg source root")

  find_path(FASTLIO2_INCLUDE_DIR
    NAMES use-ikfom.hpp
    HINTS "${FASTLIO2_ROOT}"
    PATH_SUFFIXES include)
  find_path(DCREG_INCLUDE_DIR
    NAMES dcreg.hpp
    HINTS "${DCREG_ROOT}"
    PATH_SUFFIXES include)
  if(NOT FASTLIO2_INCLUDE_DIR)
    message(FATAL_ERROR
      "FAST-LIO2 headers not found; set FASTLIO2_ROOT to the source root")
  endif()
  if(NOT DCREG_INCLUDE_DIR)
    message(FATAL_ERROR
      "DCReg headers not found; set DCREG_ROOT to the source root")
  endif()

  find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
  find_package(OpenSSL REQUIRED)
  find_library(USB_1_LIBRARY NAMES usb-1.0)
  if(NOT USB_1_LIBRARY)
    message(FATAL_ERROR "libusb-1.0 is required by the PCL-backed tools")
  endif()

  function(add_i6a_pcl_tool target source)
    add_executable(${target} "${PACKAGE_ROOT}/${source}")
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_include_directories(${target} PRIVATE
      "${PACKAGE_ROOT}/include"
      "${FASTLIO2_INCLUDE_DIR}"
      "${DCREG_INCLUDE_DIR}"
      ${PCL_INCLUDE_DIRS})
    target_link_directories(${target} PRIVATE ${PCL_LIBRARY_DIRS})
    target_compile_options(${target} PRIVATE
      ${PCL_DEFINITIONS} -fopenmp -Wall -Wextra -Wpedantic)
    target_link_libraries(${target} PRIVATE
      Eigen3::Eigen ${PCL_LIBRARIES} OpenSSL::Crypto ${USB_1_LIBRARY} gomp)
  endfunction()

  add_i6a_pcl_tool(p6_i6a_strict_frame_comparison
    "scripts/p6_i6a_strict_frame_comparison.cpp")
  add_i6a_pcl_tool(p6_i6a_score_gradient_check
    "scripts/p6_i6a_score_gradient_check.cpp")
endif()

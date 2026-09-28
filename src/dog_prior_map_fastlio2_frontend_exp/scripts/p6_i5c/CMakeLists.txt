cmake_minimum_required(VERSION 3.10)
project(p6_i5c_targeted_terminal_audit LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(FASTLIO2_ROOT "/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
set(DCREG_ROOT "/home/jian/livox_ws/DCReg/DCReg")

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(OpenSSL REQUIRED)
find_package(PythonInterp REQUIRED)
find_library(USB_1_LIBRARY NAMES usb-1.0 REQUIRED)

add_executable(p6_i5c_stationary_audit ../p6_i5c_stationary_audit.cpp)
target_compile_features(p6_i5c_stationary_audit PRIVATE cxx_std_17)
target_include_directories(p6_i5c_stationary_audit PRIVATE
  "${PACKAGE_ROOT}/include"
  "${FASTLIO2_ROOT}/include"
  "${DCREG_ROOT}/include"
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS})
target_link_directories(p6_i5c_stationary_audit PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(p6_i5c_stationary_audit PRIVATE
  ${PCL_DEFINITIONS} -fopenmp -Wall -Wextra -Wpedantic)
target_link_libraries(p6_i5c_stationary_audit PRIVATE
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  ${USB_1_LIBRARY}
  gomp)

add_executable(p6_i5c_stationary_math_test ../p6_i5c_stationary_math_test.cpp)
target_compile_features(p6_i5c_stationary_math_test PRIVATE cxx_std_17)
target_include_directories(p6_i5c_stationary_math_test PRIVATE /usr/include/eigen3)
target_compile_options(p6_i5c_stationary_math_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME p6_i5c_stationary_math
  COMMAND p6_i5c_stationary_math_test)
add_test(NAME p6_i5c_selection_protocol
  COMMAND ${PYTHON_EXECUTABLE}
    "${PACKAGE_ROOT}/scripts/test_p6_i5c_select_cases.py")
add_test(NAME p6_i5c_frozen_selection_hash_guard
  COMMAND ${PYTHON_EXECUTABLE}
    "${PACKAGE_ROOT}/scripts/test_p6_i5c_frozen_selection_guard.py"
    $<TARGET_FILE:p6_i5c_stationary_audit>
    "${PACKAGE_ROOT}")

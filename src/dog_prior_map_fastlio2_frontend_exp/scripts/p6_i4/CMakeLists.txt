cmake_minimum_required(VERSION 3.10)
project(p6_i4_prior_conditioned_basin_margin LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(FASTLIO2_ROOT "/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
set(DCREG_ROOT "/home/jian/livox_ws/DCReg/DCReg")

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(OpenSSL REQUIRED)
find_library(USB_1_LIBRARY NAMES usb-1.0 REQUIRED)

add_executable(p6_i4_basin_margin ../p6_i4_basin_margin.cpp)
target_compile_features(p6_i4_basin_margin PRIVATE cxx_std_17)
target_include_directories(p6_i4_basin_margin PRIVATE
  "${PACKAGE_ROOT}/include"
  "${FASTLIO2_ROOT}/include"
  "${DCREG_ROOT}/include"
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS})
target_link_directories(p6_i4_basin_margin PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(p6_i4_basin_margin PRIVATE ${PCL_DEFINITIONS} -fopenmp -Wall -Wextra -Wpedantic)
target_link_libraries(p6_i4_basin_margin PRIVATE
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
  ${USB_1_LIBRARY}
  gomp)

add_executable(p6_i4_basin_margin_math_test ../p6_i4_basin_margin_math_test.cpp)
target_compile_features(p6_i4_basin_margin_math_test PRIVATE cxx_std_17)
target_include_directories(p6_i4_basin_margin_math_test PRIVATE /usr/include/eigen3)
target_compile_options(p6_i4_basin_margin_math_test PRIVATE -Wall -Wextra -Wpedantic)

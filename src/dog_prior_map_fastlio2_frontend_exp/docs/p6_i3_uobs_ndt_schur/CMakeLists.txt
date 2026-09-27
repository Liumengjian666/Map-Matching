cmake_minimum_required(VERSION 3.10)
project(p6_i3_uobs_ndt LANGUAGES CXX)

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)

add_executable(p6_i3_uobs_ndt
  ../../scripts/p6_i3_uobs_ndt.cpp)
target_compile_features(p6_i3_uobs_ndt PRIVATE cxx_std_14)
target_include_directories(p6_i3_uobs_ndt PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(p6_i3_uobs_ndt PRIVATE ${PCL_LIBRARIES})
target_compile_options(p6_i3_uobs_ndt PRIVATE -O3 -Wall -Wextra -Wpedantic)
add_definitions(${PCL_DEFINITIONS})

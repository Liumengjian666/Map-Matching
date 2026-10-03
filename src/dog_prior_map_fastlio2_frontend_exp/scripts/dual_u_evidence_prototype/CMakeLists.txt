cmake_minimum_required(VERSION 3.16)
project(dual_u_evidence_prototype LANGUAGES CXX)

find_package(PCL 1.10 REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)
find_library(USB_10_LIBRARY NAMES usb-1.0 REQUIRED)

add_executable(dual_u_evidence_scan dual_u_evidence_scan.cpp)
target_compile_features(dual_u_evidence_scan PRIVATE cxx_std_17)
target_include_directories(dual_u_evidence_scan PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(dual_u_evidence_scan PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(dual_u_evidence_scan PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen ${USB_10_LIBRARY})
target_compile_definitions(dual_u_evidence_scan PRIVATE ${PCL_DEFINITIONS})

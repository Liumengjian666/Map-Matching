cmake_minimum_required(VERSION 3.10)
project(p8_official_pose_candidate_eval LANGUAGES CXX)

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10.0 EXACT REQUIRED COMPONENTS common io filters kdtree registration)
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

cmake_minimum_required(VERSION 3.10)
project(p61_r3_native_basin LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration kdtree)
get_filename_component(REPO "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(FRONTEND "${REPO}/src/dog_prior_map_fastlio2_frontend_exp")
include_directories(${PCL_INCLUDE_DIRS} "${FRONTEND}/include"
  "${REPO}/docs/p60_r4_ground_challenge_ndt_degeneracy/tools"
  "${REPO}/docs/p60_r5_ndt_resolution_correction/tools")
add_definitions(${PCL_DEFINITIONS})
add_executable(basin_probe basin_probe.cpp ${FRONTEND}/src/coupled_ndt_shadow.cpp)
target_compile_options(basin_probe PRIVATE -ffunction-sections -fdata-sections)
set_target_properties(basin_probe PROPERTIES LINK_FLAGS "-Wl,--gc-sections")
target_link_libraries(basin_probe ${PCL_LIBRARIES} usb-1.0)

cmake_minimum_required(VERSION 3.10)
project(p29_fixed_source_pair LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
add_executable(p29_offline_pair offline_pair.cpp)
target_include_directories(p29_offline_pair PRIVATE ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../dog_prior_map_fastlio2_frontend_exp/include)
target_compile_definitions(p29_offline_pair PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(p29_offline_pair ${PCL_LIBRARIES} usb-1.0)

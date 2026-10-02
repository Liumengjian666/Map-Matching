cmake_minimum_required(VERSION 3.10)
project(i2_offline_falsification LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(DCREG_ROOT "" CACHE PATH "Read-only pinned DCReg checkout")
execute_process(COMMAND git -C "${DCREG_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE SHA OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE ERR)
if(NOT ERR EQUAL 0 OR NOT SHA STREQUAL "ce7db8220f549a4a4391729e3bf4de4d4ab74635")
  message(FATAL_ERROR "DCReg reference pin mismatch")
endif()
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
add_executable(i2_geometry_audit i2_geometry_audit.cpp
  "${ROOT}/src/reliability_metrics.cpp" "${ROOT}/src/p7_replay_io.cpp")
target_include_directories(i2_geometry_audit PRIVATE "${ROOT}/include"
  "${DCREG_ROOT}/DCReg/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(i2_geometry_audit PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_options(i2_geometry_audit PRIVATE -Wall -Wextra)

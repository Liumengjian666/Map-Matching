cmake_minimum_required(VERSION 3.16)
project(p12_dcreg_solid_probe LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters kdtree)
set(DCREG_CHECKOUT "" CACHE PATH "External, unmodified author checkout")
set(PAPER_WORKSPACE "" CACHE PATH "Built paper workspace")
execute_process(COMMAND git -C ${DCREG_CHECKOUT} rev-parse HEAD
  OUTPUT_VARIABLE DCREG_SHA OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT DCREG_SHA STREQUAL "8ce8451b15491a4bbe17cf85ab02a8bed6696861")
  message(FATAL_ERROR "DCReg checkout must match the frozen SHA")
endif()
execute_process(COMMAND git -C ${DCREG_CHECKOUT} diff HEAD --exit-code
  RESULT_VARIABLE DCREG_DIRTY)
if(NOT DCREG_DIRTY EQUAL 0)
  message(FATAL_ERROR "Author source must be unmodified")
endif()
add_executable(interface_test interface_test.cpp)
target_include_directories(interface_test PRIVATE ${DCREG_CHECKOUT}/DCReg/include ${PCL_INCLUDE_DIRS})
target_link_libraries(interface_test PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})
enable_testing()
add_test(NAME p12_coordinate_interface COMMAND interface_test)
add_executable(dcreg_solid_integration_probe dcreg_solid_integration_probe.cpp)
target_include_directories(dcreg_solid_integration_probe PRIVATE
  ${DCREG_CHECKOUT}/DCReg/include ${PCL_INCLUDE_DIRS}
  ${PAPER_WORKSPACE}/src/dog_prior_map_localization/include
  ${PAPER_WORKSPACE}/src/dog_prior_map_localization/third_party/solid/include)
target_link_libraries(dcreg_solid_integration_probe PRIVATE Eigen3::Eigen
  ${PCL_LIBRARIES} ${PAPER_WORKSPACE}/devel/lib/libp11_solid_place_recognition.so
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so)
target_link_libraries(interface_test PRIVATE /usr/lib/x86_64-linux-gnu/libusb-1.0.so)
add_executable(reference_pose_test reference_pose_test.cpp)
target_link_libraries(reference_pose_test PRIVATE Eigen3::Eigen)
add_test(NAME p12b_reference_pose_interface COMMAND reference_pose_test)
add_executable(reference_pose_retry reference_pose_retry.cpp)
target_include_directories(reference_pose_retry PRIVATE
  ${DCREG_CHECKOUT}/DCReg/include ${PCL_INCLUDE_DIRS}
  ${PAPER_WORKSPACE}/src/dog_prior_map_localization/include
  ${PAPER_WORKSPACE}/src/dog_prior_map_localization/third_party/solid/include)
target_link_libraries(reference_pose_retry PRIVATE Eigen3::Eigen ${PCL_LIBRARIES}
  ${PAPER_WORKSPACE}/devel/lib/libp11_solid_place_recognition.so
  /usr/lib/x86_64-linux-gnu/libusb-1.0.so)

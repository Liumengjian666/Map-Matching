cmake_minimum_required(VERSION 3.10)
project(p9_ndt_energy_contract LANGUAGES CXX)

find_package(PCL 1.10 REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(p9_ndt_energy_contract p9_ndt_energy_contract.cpp)
target_compile_features(p9_ndt_energy_contract PRIVATE cxx_std_14)
target_include_directories(p9_ndt_energy_contract PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_compile_definitions(p9_ndt_energy_contract PRIVATE ${PCL_DEFINITIONS})
target_include_directories(p9_ndt_energy_contract PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(p9_ndt_energy_contract PRIVATE ${PCL_LIBRARIES} ${LIBUSB_LIBRARIES})

enable_testing()
add_test(NAME p9_math_self_test COMMAND p9_ndt_energy_contract --self-test)
set_tests_properties(p9_math_self_test PROPERTIES
  ENVIRONMENT "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu")

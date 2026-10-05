cmake_minimum_required(VERSION 3.10)
project(p9_ndt_energy_contract LANGUAGES CXX)

find_package(PCL 1.10 REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(p9_ndt_energy_contract p9_ndt_energy_contract.cpp)
add_executable(p9_true_profile_closure p9_true_profile_closure.cpp)
add_executable(p9_strong_attractor_closure p9_strong_attractor_closure.cpp)
add_executable(p9_support_aware_continuation p9_support_aware_continuation.cpp)
foreach(p9_target p9_ndt_energy_contract p9_true_profile_closure p9_strong_attractor_closure p9_support_aware_continuation)
  target_compile_features(${p9_target} PRIVATE cxx_std_14)
  target_include_directories(${p9_target} PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${LIBUSB_INCLUDE_DIRS})
  target_compile_definitions(${p9_target} PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(${p9_target} PRIVATE ${PCL_LIBRARIES} ${LIBUSB_LIBRARIES})
endforeach()

enable_testing()
add_test(NAME p9_math_self_test COMMAND p9_ndt_energy_contract --self-test)
add_test(NAME p9_r1a_self_test COMMAND p9_true_profile_closure --self-test)
add_test(NAME p9_r1b_self_test COMMAND p9_strong_attractor_closure --self-test)
add_test(NAME p9_r1c_self_test COMMAND p9_support_aware_continuation --self-test)
add_test(NAME p9_r1b_orchestration_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/run_strong_attractor_experiment.py --self-test)
add_test(NAME p9_r1b_inference_regression COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/summarize_strong_attractor_closure.py --self-test)
set_tests_properties(p9_math_self_test p9_r1a_self_test p9_r1b_self_test p9_r1c_self_test PROPERTIES
  ENVIRONMENT "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu")

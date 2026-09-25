cmake_minimum_required(VERSION 3.0.2)
project(dog_prior_map_fastlio2_frontend_exp)

option(
  DOG_PRIOR_ENABLE_FASTLIO2_EXP
  "Enable the opt-in FAST-LIO2/IKFoM research spike"
  OFF
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dog_prior_map_interfaces
)

catkin_package(
  CATKIN_DEPENDS roscpp dog_prior_map_interfaces
)

if(DOG_PRIOR_ENABLE_FASTLIO2_EXP)
  set(FASTLIO2_REF_ROOT "" CACHE PATH "Pinned external FAST-LIO2 checkout")
  set(_fastlio2_expected_sha "7cc4175de6f8ba2edf34bab02a42195b141027e9")

  if(NOT FASTLIO2_REF_ROOT)
    message(FATAL_ERROR
      "DOG_PRIOR_ENABLE_FASTLIO2_EXP=ON requires FASTLIO2_REF_ROOT")
  endif()
  if(NOT IS_DIRECTORY "${FASTLIO2_REF_ROOT}/.git")
    message(FATAL_ERROR "FASTLIO2_REF_ROOT is not a Git checkout: ${FASTLIO2_REF_ROOT}")
  endif()

  execute_process(
    COMMAND git -C "${FASTLIO2_REF_ROOT}" rev-parse HEAD
    RESULT_VARIABLE _fastlio2_git_result
    OUTPUT_VARIABLE _fastlio2_actual_sha
    ERROR_VARIABLE _fastlio2_git_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _fastlio2_git_result EQUAL 0)
    message(FATAL_ERROR "Cannot inspect FAST-LIO2 checkout: ${_fastlio2_git_error}")
  endif()
  if(NOT _fastlio2_actual_sha STREQUAL _fastlio2_expected_sha)
    message(FATAL_ERROR
      "FAST-LIO2 SHA mismatch: expected ${_fastlio2_expected_sha}, got ${_fastlio2_actual_sha}")
  endif()

  foreach(_required_file
      include/use-ikfom.hpp
      include/IKFoM_toolkit/esekfom/esekfom.hpp
      include/IKFoM_toolkit/mtk/build_manifold.hpp)
    if(NOT EXISTS "${FASTLIO2_REF_ROOT}/${_required_file}")
      message(FATAL_ERROR "Pinned FAST-LIO2 file missing: ${_required_file}")
    endif()
  endforeach()

  find_package(Eigen3 REQUIRED)
  find_package(OpenMP REQUIRED)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_executable(ikfom_pose_api_spike
    src/ikfom_pose_api_spike.cpp
    tests/ikfom_pose_api_spike_main.cpp
  )
  target_include_directories(ikfom_pose_api_spike PRIVATE
    include
    "${FASTLIO2_REF_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_compile_options(ikfom_pose_api_spike PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(ikfom_pose_api_spike
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
  )
  add_dependencies(ikfom_pose_api_spike ${catkin_EXPORTED_TARGETS})
endif()

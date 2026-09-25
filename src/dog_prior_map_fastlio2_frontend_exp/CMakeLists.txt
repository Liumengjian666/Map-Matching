cmake_minimum_required(VERSION 3.0.2)
project(dog_prior_map_fastlio2_frontend_exp)

option(
  DOG_PRIOR_ENABLE_FASTLIO2_EXP
  "Enable the opt-in FAST-LIO2/IKFoM research spike"
  OFF
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  dog_prior_map_interfaces
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs dog_prior_map_interfaces
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
  add_test(NAME ikfom_pose_api_spike
    COMMAND $<TARGET_FILE:ikfom_pose_api_spike>)

  add_executable(fastlio2_propagation_contract_test
    src/fastlio2_frontend_ikfom.cpp
    tests/fastlio2_propagation_contract_test.cpp
  )
  target_include_directories(fastlio2_propagation_contract_test PRIVATE
    include
    "${FASTLIO2_REF_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_compile_options(fastlio2_propagation_contract_test PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(fastlio2_propagation_contract_test
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
  )
  add_dependencies(fastlio2_propagation_contract_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME fastlio2_propagation_contract_test
    COMMAND $<TARGET_FILE:fastlio2_propagation_contract_test>)

  add_executable(scan_end_deskew_runtime_test
    src/fastlio2_frontend_ikfom.cpp
    src/scan_processor.cpp
    tests/scan_end_deskew_runtime_test.cpp
  )
  target_include_directories(scan_end_deskew_runtime_test PRIVATE
    include
    "${FASTLIO2_REF_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_compile_options(scan_end_deskew_runtime_test PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(scan_end_deskew_runtime_test
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
  )
  add_dependencies(scan_end_deskew_runtime_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME scan_end_deskew_runtime_test
    COMMAND $<TARGET_FILE:scan_end_deskew_runtime_test>)

  add_executable(frontend_runtime_end_to_end_test
    src/fastlio2_frontend_ikfom.cpp
    src/scan_processor.cpp
    src/frontend_runtime.cpp
    tests/frontend_runtime_end_to_end_test.cpp
  )
  target_include_directories(frontend_runtime_end_to_end_test PRIVATE
    include
    "${FASTLIO2_REF_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_compile_options(frontend_runtime_end_to_end_test PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(frontend_runtime_end_to_end_test
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
  )
  add_dependencies(frontend_runtime_end_to_end_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME frontend_runtime_end_to_end_test
    COMMAND $<TARGET_FILE:frontend_runtime_end_to_end_test>)

  add_executable(dog_prior_map_fastlio2_frontend_node
    src/fastlio2_frontend_ikfom.cpp
    src/scan_processor.cpp
    src/frontend_runtime.cpp
    src/fastlio2_frontend_node.cpp
  )
  target_include_directories(dog_prior_map_fastlio2_frontend_node PRIVATE
    include
    "${FASTLIO2_REF_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_compile_options(dog_prior_map_fastlio2_frontend_node PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(dog_prior_map_fastlio2_frontend_node
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
  )
  add_dependencies(dog_prior_map_fastlio2_frontend_node ${catkin_EXPORTED_TARGETS})
  install(TARGETS dog_prior_map_fastlio2_frontend_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY config launch docs
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  add_executable(ndt_pose_update_runtime_test
    src/fastlio2_frontend_ikfom.cpp
    tests/ndt_pose_update_runtime_test.cpp
  )
  target_include_directories(ndt_pose_update_runtime_test PRIVATE
    include
    "${FASTLIO2_REF_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_compile_options(ndt_pose_update_runtime_test PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(ndt_pose_update_runtime_test
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
  )
  add_dependencies(ndt_pose_update_runtime_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME ndt_pose_update_runtime_test
    COMMAND $<TARGET_FILE:ndt_pose_update_runtime_test>)
endif()

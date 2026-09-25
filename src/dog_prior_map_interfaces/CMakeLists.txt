cmake_minimum_required(VERSION 3.0.2)
project(dog_prior_map_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  NdtScanRequest.msg
  NdtScanResult.msg
  NdtServerStatus.msg
  NdtSessionControl.msg
  NdtSessionControlAck.msg
  NdtScanAck.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
)

if(CATKIN_ENABLE_TESTING)
  add_executable(ndt_protocol_contract_test
    test/ndt_protocol_contract_test.cpp
  )
  add_dependencies(ndt_protocol_contract_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(ndt_protocol_contract_test PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ndt_protocol_contract_test ${catkin_LIBRARIES})
  add_test(NAME ndt_protocol_contract_test COMMAND $<TARGET_FILE:ndt_protocol_contract_test>)
endif()

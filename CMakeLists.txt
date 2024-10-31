cmake_minimum_required(VERSION 3.8)
project(integration_test)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

find_package(ament_cmake REQUIRED) #turns on BUILD_TESTING
find_package(catch_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  # disable these tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()


###########################
## INTEGRATION TEST NODE ##
###########################
# This is the node in which integration tests occur
add_executable(integration_test_node
  src/integration_test_node.cpp
  )
# The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
# to get the default integration test node main function
target_link_libraries(integration_test_node
  catch_ros2::catch_ros2_with_node_main
  )
ament_target_dependencies(integration_test_node
  rclcpp std_srvs std_msgs
  )
install(TARGETS
  integration_test_node
  DESTINATION lib/${PROJECT_NAME}
  )

##########################
## INTEGRATION AUX NODE ##
##########################
# This could be any node in your ROS 2 package
# Likely you are looking to test some functionality of this node
add_executable(integration_aux_node
  src/integration_aux_node.cpp
)
ament_target_dependencies(integration_aux_node
  rclcpp std_srvs
)

install(TARGETS
  integration_aux_node
  DESTINATION lib/${PROJECT_NAME}
)

######################
## INTEGRATION TEST ##
######################
# This CMake function allows the integration test to be run
# when running "colcon test".
catch_ros2_add_integration_test(ExampleIntegration_TestYAML
  LAUNCH_FILE integration_test.launch.yaml  # example in YAML
  )


#################
## LAUNCH FILE ##
#################
# This is the launch file that will be used to run the integration test
# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()

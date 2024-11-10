cmake_minimum_required(VERSION 3.8)
project(integration_test)

find_package(ament_cmake REQUIRED) # turns on the BUILD_TESTING option
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


#####################################################################################
# 0.) Create the compilation database for clangd and move it out of the build dir. ##
#####################################################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

###########################################################################
# 1.) BUILD THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
###########################################################################
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

#######################################################################
## 2.) BUILD AN EXAMPLE INTEGRATION AUX NODE (aka Nodes under teset) ##
#######################################################################
# Here, we build a node to be tested and package it within this ROS2
# package.  However, it could be a any node(s) in any ROS 2 package(s)
# that you want to test.  This means the nodes and packages already
# exist and this step is not needed here.  See the launcher file to
# see all the node(s) under test.
add_executable (service_server
  src/service_server_node.cpp
  )
ament_target_dependencies (service_server
  rclcpp std_srvs
  )

######################################
## 3.) ADD INTEGRATION TEST SUPPORT ##
######################################
# This CMake function allows the integration test to be run
# when running "colcon test".
catch_ros2_add_integration_test (ExampleIntegration_TestYAML
  LAUNCH_FILE integration_test.launch.yaml  # example in YAML
  )

#####################################################
## 4.) INSTALL NODES (EXECUTABLE) AND LAUNCH FILES ##
#####################################################
install(TARGETS
  integration_test_node
  service_server
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY
  launch/                        # install the entire lauch/ subdirectory
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()

// Copyright 2023 Nick Morales.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// # CHANGES:
//
// 2024-10-31, Tommy Chang
//     - Renamed the first test to "service_test".
//     - Added a second test, "talker test".

/// @file This is an example ROS 2 node that checks assertions using Catch2.
/// It simply checks if the "test_service" service is available at least once
/// during the duration of the test.

#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

TEST_CASE("service_test", "[integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a service available
  auto node = rclcpp::Node::make_shared("integration_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Create a client for the service we're looking for
  auto client = node->create_client<std_srvs::srv::Empty>("test_service");


  rclcpp::Time start_time = rclcpp::Clock().now();

  bool service_found = false;

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // Repeatedly check for the dummy service until its found
    if (client->wait_for_service(0s)) {
      service_found = true;
      break;
    }

    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK(service_found);
}



TEST_CASE("talker_test", "[integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a topic available
  auto node = rclcpp::Node::make_shared("talker_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // subscribe to the topic 
  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  bool gotData = false;
  SUBSCRIBER subscription = node->create_subscription<String>
    ("chatter", 10,
     // Lambda expression begins
     [&](const std_msgs::msg::String& msg) {
       RCLCPP_INFO(node->get_logger(), "I heard: '%s'", msg.data.c_str());
       gotData = true;
     } // end of lambda expression
     );

  // should get data winhin TEST_DURATION
  rclcpp::Rate rate(2.0);       // 2hz checks
  rclcpp::Time start_time = rclcpp::Clock().now();
  while ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
    {
      rclcpp::spin_some(node);
      rate.sleep();

      if (gotData)
        break;
    }
  
  // Test assertions - check that the dummy node was found
  CHECK (gotData);
}

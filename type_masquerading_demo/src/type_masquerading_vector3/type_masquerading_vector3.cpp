// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/Vector3.hpp>

struct MyVector3
{
  double x;
  double y;
  double z;
};

template<>
struct TypeMasquerade<geometry_msgs::msg::Vector3, MyVector3>
{
  void
  to_message(const MyVector3 & non_message, geometry_msgs::msg::Vector3 & message)
  {
    message.x = non_message.x;
    message.y = non_message.y;
    message.z = non_message.z;
  }

  void
  to_non_message(const geometry_msgs::msg::Vector3 & message, MyVector3 & non_message)
  {
    non_message.x = message.x;
    non_message.y = message.y;
    non_message.z = message.z;
  }
};

struct MyNode : public rclcpp::Node
{
  MyNode(const std::string & name)
  : Node(name, true)
  {
    // pub = this->create_publisher<geometry_msgs::msg::Vector3>("out", rmw_qos_profile_default);
    pub = this->create_publisher<geometry_msgs::msg::Vector3, MyVector3>("out", rmw_qos_profile_default);
    MyVector3 my_vector3 {1, 2, 3};
    pub->publish(my_vector3);
    // pub->publish<MyVector3>()
    // Create a subscription on the input topic.
    // sub = this->create_subscription_with_unique_ptr_callback<std_msgs::msg::Int32>(
    //   in, rmw_qos_profile_default,
    //   [this](std_msgs::msg::Int32::UniquePtr & msg) {
    //     printf("Received message with value:         %d, and address: %p\n", msg->data, msg.get());
    //     printf("  sleeping for 1 second...\n");
    //     if (!rclcpp::sleep_for(1_s)) {
    //       return;  // Return if the sleep failed (e.g. on ctrl-c).
    //     }
    //     printf("  done.\n");
    //     msg->data++;  // Increment the message's data.
    //     printf("Incrementing and sending with value: %d, and address: %p\n", msg->data, msg.get());
    //     this->pub->publish(msg);  // Send the message along to the output topic.
    //   });
  }

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<MyNode>("type_masquerade_vector3");

  executor.add_node(node);
  executor.spin();
  return 0;
}

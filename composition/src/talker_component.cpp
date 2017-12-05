// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "composition/talker_component.hpp"

#include <chrono>
#include <cinttypes>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

namespace composition
{

// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
Talker::Talker()
: Node("talker", "", true), count_(0)
{
  // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
  pub_ = create_publisher<std_msgs::msg::UInt32>("chatter", rmw_qos_profile_sensor_data);

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(5ms, std::bind(&Talker::on_timer, this));

  // std::unique_ptr<std_msgs::msg::UInt32> msg(new std_msgs::msg::UInt32());
}

void Talker::on_timer()
{
  std::unique_ptr<std_msgs::msg::UInt32> msg(new std_msgs::msg::UInt32());
  msg->data = ++count_ % UINT32_MAX;
  if (0 == count_ % 1000) {
    // printf("Publishing: '%s'\n", msg->data.c_str());
    printf(
      "Publishing message '%u', and address: 0x%" PRIXPTR "\n",
      msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
    std::flush(std::cout);
  }
  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  pub_->publish(msg);
}

}  // namespace composition

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(composition::Talker, rclcpp::Node)

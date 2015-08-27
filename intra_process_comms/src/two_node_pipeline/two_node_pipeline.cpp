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
#include <std_msgs/msg/int32.hpp>

struct Producer : public rclcpp::Node
{
  Producer(std::string name, bool use_intra_process) : Node(name, use_intra_process)
  {
    pub_ = this->create_publisher<std_msgs::msg::Int32>("number", rmw_qos_profile_default);
    timer_ = this->create_wall_timer(1_s, [this]() {
      static size_t count = 0;
      std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
      msg->data = count++;
      printf("Published message with value: %d, and address: %p\n", msg->data, msg.get());
      pub_->publish(msg);
    });
  }

  rclcpp::Publisher::SharedPtr pub_;
  rclcpp::WallTimer::SharedPtr timer_;
};

struct Consumer : public rclcpp::Node
{
  Consumer(std::string name, bool use_intra_process) : Node(name, use_intra_process)
  {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "number", rmw_qos_profile_default, [](std_msgs::msg::Int32::UniquePtr & msg) {
        printf(" Received message with value: %d, and address: %p\n", msg->data, msg.get());
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("producer", true);
  auto consumer = std::make_shared<Consumer>("consumer", true);

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();
  return 0;
}

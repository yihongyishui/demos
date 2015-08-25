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

#include <rclcpp/rclcpp.hpp>

#include "intra_process_comms/device_node.hpp"
#include "intra_process_comms/extractor_node.hpp"
#include "intra_process_comms/modifier_node.hpp"
#include "intra_process_comms/visualizer_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::Node::SharedPtr deviceNode = std::make_shared<intra_process_comms::DeviceNode>();
  rclcpp::Node::SharedPtr extractorNode = std::make_shared<intra_process_comms::ExtractorNode>();
  rclcpp::Node::SharedPtr modifierNode = std::make_shared<intra_process_comms::ModifierNode>();
  rclcpp::Node::SharedPtr visualizerNode = std::make_shared<intra_process_comms::VisualizerNode>();

  executor.add_node(deviceNode);
  executor.add_node(extractorNode);
  executor.add_node(modifierNode);
  executor.add_node(visualizerNode);

  executor.spin();
  return 0;
}

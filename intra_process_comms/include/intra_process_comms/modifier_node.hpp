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

#ifndef INTRA_PROCESS_COMMS_MODIFIER_NODE_HPP_
#define INTRA_PROCESS_COMMS_MODIFIER_NODE_HPP_

#include <chrono>

#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "intra_process_comms/utils.hpp"

namespace intra_process_comms
{

class ModifierNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ModifierNode);

  ModifierNode(const std::string & node_name = "img_modifier",
    bool use_intra_process_comms = true)
  : Node(node_name, use_intra_process_comms)
  {
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 16;
    // after getting the first message, reassign based on encoding type
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("modified_image", qos);

    // TODO: Copy occurs here. Change to UniquePtr once implemented
    auto img_sub_callback =
      [this](sensor_msgs::msg::Image::SharedPtr msg) -> void {
        convert_message_to_frame(msg, frame_);

        cv::bitwise_not(*frame_, *frame_);

        img_pub_->publish(msg);
        ++msg_number_;
      };

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("extracted_image", qos,
        img_sub_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher::SharedPtr img_pub_;

  std::shared_ptr<cv::Mat> frame_;
  size_t msg_number_ = 0;

  std::chrono::milliseconds wait_key_period_;
};

}

#endif  /* INTRA_PROCESS_COMMS_MODIFIER_NODE_HPP_ */

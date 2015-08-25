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

#ifndef INTRA_PROCESS_COMMS_VISUALIZER_NODE_HPP_
#define INTRA_PROCESS_COMMS_VISUALIZER_NODE_HPP_

#include <chrono>

#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "intra_process_comms/utils.hpp"

namespace intra_process_comms
{

class VisualizerNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(VisualizerNode);

  VisualizerNode(const std::string & node_name = "img_visualizer",
    bool use_intra_process_comms = true,
    std::chrono::nanoseconds update_period = std::chrono::nanoseconds(1000000))
  : Node(node_name, use_intra_process_comms)
  {
    wait_key_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(update_period);
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 16;
    // after getting the first message, reassign based on encoding type

    auto img_sub_callback =
      [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void {
        std::cout << "Got image from modifier" << std::endl;
        convert_message_to_frame(msg, frame_);

        cv::imshow("Modified image", *frame_);
        cv::waitKey(wait_key_period_.count());
      };

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("modified_image", qos,
        img_sub_callback);
  }

private:
  rclcpp::WallTimer::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  std::shared_ptr<cv::Mat> frame_;
  size_t msg_number_ = 0;

  std::chrono::milliseconds wait_key_period_;
};

}

#endif  /* INTRA_PROCESS_COMMS_VISUALIZER_NODE_HPP_ */

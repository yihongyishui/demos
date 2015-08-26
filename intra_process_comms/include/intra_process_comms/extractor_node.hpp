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

#ifndef INTRA_PROCESS_COMMS_EXTRACTOR_NODE_HPP_
#define INTRA_PROCESS_COMMS_EXTRACTOR_NODE_HPP_

#include <chrono>

#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "intra_process_comms/utils.hpp"

namespace intra_process_comms
{

class ExtractorNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ExtractorNode);

  ExtractorNode(const std::string & node_name = "feature_extractor",
    bool use_intra_process_comms = true,
    int histogram_size = 30)
  : Node(node_name, use_intra_process_comms)
  {
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 16;

    image_msg_ = std::make_shared<sensor_msgs::msg::Image>();


    // after getting the first message, reassign based on encoding type
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("extracted_image", qos);

    //feature_pub_ = this->create_publisher<sensor_msgs::msg::Image>("extracted_features", qos);

    // Subscribe to image
    auto sub_callback =
      [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void {
        image_msg_ = msg;
        img_pub_->publish(image_msg_);
        convert_message_to_frame(image_msg_, frame_);

        if (planes_.empty()) {
          cv::split(*frame_, planes_);
        }
        int histSize = 32;
        float range[] = {0, 256};
        const float * histRange = {range};
        // TODO: encoding type
        if (planes_.size() < 3) {
          throw std::runtime_error("Image wasn't split into 3 channels!");
        }
        cv::calcHist(&planes_[0], 1, 0, cv::Mat(), b_hist_, 1, &histSize, &histRange);
        cv::calcHist(&planes_[1], 1, 0, cv::Mat(), g_hist_, 1, &histSize, &histRange);
        cv::calcHist(&planes_[2], 1, 0, cv::Mat(), r_hist_, 1, &histSize, &histRange);

      };

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image", qos, sub_callback);
  }

private:
  rclcpp::Publisher::SharedPtr img_pub_;
  //rclcpp::Publisher::SharedPtr feature_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::WallTimer::SharedPtr timer_;

  sensor_msgs::msg::Image::SharedPtr image_msg_;

  std::shared_ptr<cv::Mat> frame_;

  std::vector<cv::Mat> planes_;
  cv::Mat r_hist_;
  cv::Mat g_hist_;
  cv::Mat b_hist_;

  int hist_dims_;
};

}

#endif  /* INTRA_PROCESS_COMMS_EXTRACTOR_NODE_HPP_ */

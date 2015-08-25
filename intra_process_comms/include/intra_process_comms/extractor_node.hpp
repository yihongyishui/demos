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
    std::chrono::nanoseconds update_period = std::chrono::nanoseconds(1000000),
    int histogram_r = 30, int histogram_g = 30, int histogram_b = 30)
  : Node(node_name, use_intra_process_comms)
  {

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 2;

    image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
    // after getting the first message, reassign based on encoding type
    hist_dims_[0] = histogram_r;
    hist_dims_[1] = histogram_g;
    hist_dims_[2] = histogram_b;

    float range[] = {0, 255};
    const float * ranges[] = {range, range, range};
    // Subscribe to image
    auto sub_callback =
      [this, &ranges](const sensor_msgs::msg::Image::SharedPtr msg) -> void {
        image_msg_ = msg;
        convert_message_to_frame(image_msg_, frame_);
        int channels = frame_->channels();
        cv::calcHist(frame_.get(), 1, &channels, cv::Mat(), hist_, 3, &hist_dims_[0],
          const_cast<const float **>(ranges));
        // TODO Do something awesome with the histogram.
        convert_frame_to_message(hist_, msg_number_, hist_msg_);
        ++msg_number_;
      };

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image", qos, sub_callback);

    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("extracted_image", qos);

    feature_pub_ = this->create_publisher<sensor_msgs::msg::Image>("extracted_features", qos);

    auto pub_callback =
      [this]() -> void {
        // If frame hasn't been initialized yet, don't do anything.
        if (!frame_) {
          return;
        }
        img_pub_->publish(image_msg_);
        feature_pub_->publish(hist_msg_);
      };

    timer_ = this->create_wall_timer(update_period, pub_callback);
  }

private:
  rclcpp::Publisher::SharedPtr img_pub_;
  rclcpp::Publisher::SharedPtr feature_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::WallTimer::SharedPtr timer_;

  sensor_msgs::msg::Image::SharedPtr image_msg_;
  sensor_msgs::msg::Image::SharedPtr hist_msg_;

  std::shared_ptr<cv::Mat> frame_;
  size_t msg_number_ = 0;

  cv::MatND hist_;
  int hist_dims_[3];
};

}

#endif  /* INTRA_PROCESS_COMMS_EXTRACTOR_NODE_HPP_ */

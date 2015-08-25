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

#ifndef INTRA_PROCESS_COMMS_DEVICE_NODE_HPP_
#define INTRA_PROCESS_COMMS_DEVICE_NODE_HPP_

#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "intra_process_comms/utils.hpp"

namespace intra_process_comms
{

class DeviceNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DeviceNode);

  DeviceNode(const std::string & node_name = "camera_device", bool use_intra_process_comms = false,
    std::chrono::nanoseconds update_period = std::chrono::nanoseconds(50000000),
    int device = 0, int width = 320, int height = 240)
  : Node(node_name, use_intra_process_comms)
  {
    // Initialize OpenCV
    cap_.open(device);
    cap_.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap_.isOpened()) {
      throw std::runtime_error("Could not open video stream!");
    }
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 16;

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", qos);

    auto pub_callback =
      [this]() -> void {
        cap_ >> frame_;
        // skip dropped frame
        if (frame_.empty()) {
          return;
        }
        convert_frame_to_message(frame_, msg_number_, image_msg_);
        if (!image_msg_) {
          throw std::runtime_error("Image message was null");
          return;
        }
        pub_->publish(image_msg_);
        ++msg_number_;
      };

    timer_ = this->create_wall_timer(update_period, pub_callback);
  }

private:
  rclcpp::Publisher::SharedPtr pub_;
  rclcpp::WallTimer::SharedPtr timer_;

  sensor_msgs::msg::Image::SharedPtr image_msg_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
  size_t msg_number_ = 0;

};

}

#endif  /* INTRA_PROCESS_COMMS_DEVICE_NODE_HPP_ */

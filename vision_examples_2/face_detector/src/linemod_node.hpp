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

#ifndef FACE_DETECTOR__LINEMOD_NODE_HPP_
#define FACE_DETECTOR__LINEMOD_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "linemod.cpp"

#include "intra_process_demo/image_pipeline/common.hpp"

// Node that receives an image, locates faces and draws boxes around them, and publishes it again.
class LinemodNode : public rclcpp::Node
{
public:
  LinemodNode(
    const std::string & input, const std::string & depth_input, const std::string & output,
    const std::string & node_name = "linemod_node")
  : Node(node_name, "", true)
  {
    cv::namedWindow("orig_color");
    std::cout << "starting linemod node..." << std::endl;
    load();
    auto qos = rmw_qos_profile_sensor_data;
    // Create a publisher on the input topic.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, qos);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a subscription on the output topic.
    std::cout << "creating subscribers..." << std::endl;
    sub_color_ = this->create_subscription<sensor_msgs::msg::Image>(
        input, [&](sensor_msgs::msg::Image::UniquePtr msg)
    {
      // // Create a cv::Mat from the image message (without copying).
      // cv::Mat cv_mat(
      //   msg->height, msg->width,
      //   encoding2mat_type(msg->encoding),
      //   msg->data.data());
      // cv::cvtColor(cv_mat, cv_mat, CV_BGR2RGB);

      // frame_color_ = cv_mat;
      // std::cout << "stored color frame" << std::endl;
      // cv::imshow("orig_color", frame_color_);
      // cv::waitKey(1);

    }, qos);

    sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_input, [&](sensor_msgs::msg::Image::UniquePtr msg)
    {
      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(
        msg->height, msg->width,
        encoding2mat_type(msg->encoding),
        msg->data.data());

      frame_depth_ = cv_mat;
      std::cout << "stored depth frame" << std::endl;

      if (frame_depth_.depth() == CV_32F)
      {
        frame_depth_.convertTo(frame_depth_, CV_16UC1, 1000.0);
        std::cout << "converted depth frame" << std::endl;
      }

      if(!(frame_depth_.data == NULL) /*&& !(frame_color_.data == NULL)*/)
      {
        std::cout << "calling linemod..." << std::endl;
        // cv::imshow("orig_color", frame_color_);
        // cv::waitKey(1000);
        doit(frame_color_, frame_depth_);
      }

    }, qos);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;

  cv::VideoCapture cap_;
  cv::Mat frame_color_;
  cv::Mat frame_depth_;
};

#endif

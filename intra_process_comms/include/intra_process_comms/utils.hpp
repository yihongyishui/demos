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

#ifndef INTRA_PROCESS_COMMS_UTILS_HPP_
#define INTRA_PROCESS_COMMS_UTILS_HPP_

std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}

void convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
  // assign cv information into ros message
  // if msg is null, it's never been assigned, so we should allocate it
  size_t size = frame.step * frame.rows;
  if (!msg) {
    msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->height = frame.rows;
    msg->width = frame.cols;
    msg->encoding = mat_type2encoding(frame.type());
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    msg->data.resize(size);
  } else {
    // We are overwriting an existing msg. Check that the dimensions are the same
    auto frame_step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    if (static_cast<int>(msg->height) != frame.rows ||
      static_cast<int>(msg->width) != frame.cols ||
      msg->encoding != mat_type2encoding(frame.type()) || msg->step != frame_step)
    {
      throw std::runtime_error("Invalid dimension change requested");
    }
  }
  // TODO: assign, don't copy
  memcpy(&msg->data[0], frame.data, size);

  msg->header.frame_id = std::to_string(frame_id);
}

void convert_message_to_frame(
  const sensor_msgs::msg::Image::SharedPtr msg,
  std::shared_ptr<cv::Mat> frame)
{
  if (!msg) {
    throw std::runtime_error("Null message cannot be converted to OpenCV frame!");
  }
  if (!frame) {
    // Allocate a new frame
    frame = std::make_shared<cv::Mat>(msg->height, msg->width, encoding2mat_type(msg->encoding), 0);
  } else {
    auto frame_step = static_cast<sensor_msgs::msg::Image::_step_type>(frame->step);
    if (static_cast<int>(msg->height) != frame->rows ||
      static_cast<int>(msg->width) != frame->cols ||
      msg->encoding != mat_type2encoding(frame->type()) ||
      msg->step != frame_step)
    {
      throw std::runtime_error("Invalid dimension change requested");
    }
  }
  size_t size = msg->step * msg->height;

  // TODO: assign, don't copy?
  memcpy(frame->data, &msg->data[0], size);
}

#endif

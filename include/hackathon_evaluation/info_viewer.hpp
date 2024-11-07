// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef HACKATHON_EVALUATION__INFO_VIEWER_HPP_
#define HACKATHON_EVALUATION__INFO_VIEWER_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <unordered_map>

namespace hackathon
{

class InfoViewer
{
public:
  using OverlayText = rviz_2d_overlay_msgs::msg::OverlayText;

  struct FieldData
  {
    double crushed_percentage = 0;
    double covered_percentage = 0;
  };

  using Fields = std::unordered_map<std::string, FieldData>;

public:
  InfoViewer(rclcpp::Node & node);

  void add_field(const std::string & field_name);
  void set_covered_percentage(const std::string & field_name, double value);
  void set_crushed_percentage(const std::string & field_name, double value);

private:
  void timer_callback_();

private:
  Fields fields_;
  OverlayText msg_;

  std::shared_ptr<rclcpp::Publisher<OverlayText>> text_pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

}  // namespace hackathon

#endif

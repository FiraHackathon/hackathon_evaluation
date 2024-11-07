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

#include "hackathon_evaluation/info_viewer.hpp"

#include <functional>
#include <iomanip>
#include <memory>
#include <rclcpp/qos.hpp>
#include <sstream>

namespace hackathon
{

InfoViewer::InfoViewer(rclcpp::Node & node)
{
  text_pub_ = node.create_publisher<OverlayText>("~/info_overlay", rclcpp::QoS{1});

  using namespace std::chrono_literals;
  timer_ = node.create_wall_timer(200ms, std::bind(&InfoViewer::timer_callback_, this));

  msg_.width = 400;
  msg_.height = 120;
  msg_.vertical_alignment = OverlayText::TOP;
  msg_.horizontal_alignment = OverlayText::LEFT;
  msg_.vertical_distance = 10;
  msg_.horizontal_distance = 10;
  msg_.line_width = 200;
  // msg_.bg_color.r = 1.0;
  // msg_.bg_color.g = 1.0;
  // msg_.bg_color.b = 1.0;
  msg_.bg_color.a = 0.05;
  msg_.fg_color.r = 0.0;
  msg_.fg_color.g = 0.639;
  msg_.fg_color.b = 0.651;
  msg_.fg_color.a = 1.0;
  msg_.font = "DejaVu Sans Mono";
  msg_.text_size = 16;
}

void InfoViewer::add_field(const std::string & field_name)
{
  fields_.insert({field_name, FieldData{}});
}

void InfoViewer::set_covered_percentage(const std::string & field_name, double value)
{
  fields_[field_name].covered_percentage = value;
}

void InfoViewer::set_crushed_percentage(const std::string & field_name, double value)
{
  fields_[field_name].crushed_percentage = value;
}

void InfoViewer::timer_callback_()
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  for (auto const & [name, field] : fields_) {
    oss << name << ", covered: " << field.covered_percentage << " %\n";
    oss << name << ", crushed: " << field.crushed_percentage << " %\n";
  }
  msg_.text = oss.str();

  text_pub_->publish(msg_);
}

}  // namespace hackathon

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

#ifndef HACKATHON_EVALUATION__CROPS_VIEWER_HPP_
#define HACKATHON_EVALUATION__CROPS_VIEWER_HPP_

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hackathon_evaluation/crop_field.hpp"

namespace hackathon
{

class CropsViewer
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using Markers = visualization_msgs::msg::MarkerArray;

  struct Field
  {
    const CropField * data;
    Markers markers;
    bool changed;
  };

public:
  CropsViewer(rclcpp::Node & node);

  /// Add field and create an rviz marker
  void add_field(const std::string & name, const CropField & field);

  /// The specified field will be updated in the next run of the timer
  void notify_change(const std::string & field_name);

private:
  void timer_callback_();

  void create_markers_(Field & field);

  void update_field_(Field & field);

private:
  std::unordered_map<std::string, Field> fields_;
  std::time_t last_id_ = 0;

  rclcpp::Node & node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<rclcpp::Publisher<Markers>> markers_pub_;
};

}  // namespace hackathon

#endif

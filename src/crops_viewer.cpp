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

#include "hackathon_evaluation/crops_viewer.hpp"

#include <rclcpp/time.hpp>
#include <stdexcept>

namespace hackathon
{

namespace
{

geometry_msgs::msg::Point ground_projection(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point res;
  res.x = p.x();
  res.y = p.y();
  return res;
}

}  // namespace

CropsViewer::CropsViewer(rclcpp::Node & node) : node_(node)
{
  using namespace std::chrono_literals;
  timer_ = node.create_wall_timer(200ms, std::bind(&CropsViewer::timer_callback_, this));

  markers_pub_ = node.create_publisher<Markers>("~/crops_marker", rclcpp::QoS(2));
}

void CropsViewer::add_field(const std::string & name, const CropField & field)
{
  if (fields_.count(name)) {
    throw std::runtime_error(std::string{"CropViewer: field '"} + name + "' already exist.");
  }

  fields_.insert({name, Field{&field, Markers(), false}});
  Field & new_field = fields_[name];
  create_markers_(new_field);
  update_field_(new_field);
}

/// The specified field will be updated in the next run of the timer
void CropsViewer::notify_change(const std::string & field_name)
{
  fields_[field_name].changed = true;
}

void CropsViewer::timer_callback_()
{
  for (auto & [name, field] : fields_) {
    if (field.changed) {
      update_field_(field);
      markers_pub_->publish(field.markers);
    }
  }
}

void CropsViewer::create_markers_(Field & field)
{
  field.markers.markers.emplace_back();
  auto & crushed = field.markers.markers.emplace_back();
  auto & crops = field.markers.markers.front();  // the first emplace_back's return is invalidated

  crops.header.frame_id = "map";
  crops.ns = "crops";
  crops.id = last_id_++;
  crops.type = Marker::SPHERE_LIST;
  crops.action = Marker::ADD;
  crops.color.g = 1.0;
  crops.color.a = 0.5;
  crops.scale.x = 0.1;
  crops.scale.y = 0.1;
  crops.scale.z = 0.2;

  crushed.header.frame_id = "map";
  crushed.ns = "crushed_crops";
  crushed.id = last_id_++;
  crushed.type = Marker::SPHERE_LIST;
  crushed.action = Marker::ADD;
  crushed.color.r = 1.0;
  crushed.color.a = 0.5;
  crushed.scale.x = 0.15;
  crushed.scale.y = 0.15;
  crushed.scale.z = 0.3;
}

void CropsViewer::update_field_(Field & field)
{
  auto & crops = field.markers.markers.front();
  auto & crushed = field.markers.markers[1];

  // Disabled to ignore time in rviz
  // crops.header.stamp = node_.now();
  // crushed.header.stamp = node_.now();

  crops.points.clear();
  crushed.points.clear();

  for (const auto & crop : field.data->get_crops()) {
    if (crop.crushed) {
      crushed.points.push_back(ground_projection(crop.pos));
    } else {
      crops.points.push_back(ground_projection(crop.pos));
    }
  }
}

}  // namespace hackathon

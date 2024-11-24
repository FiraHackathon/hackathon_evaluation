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

#include "hackathon_evaluation/evaluation.hpp"

#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>

#include "hackathon_evaluation/collision_counter.hpp"
#include "hackathon_evaluation/crop_field.hpp"
#include "hackathon_evaluation/crops_viewer.hpp"
#include "hackathon_evaluation/info_viewer.hpp"
#include "hackathon_evaluation/xml_world_parser.hpp"

namespace hackathon
{

Evaluation::Evaluation(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("evaluation", options)),
  crops_viewer_(*node_),
  info_viewer_(*node_),
  collision_counter_(node_),
  geofence_checker_(node_)
{
  // Use world parser to get transform of each model
  node_->declare_parameter<std::string>("world_file");
  auto world_file = node_->get_parameter("world_file");
  XmlWorldParser world_parser{world_file.as_string()};

  // load ROS params and topics for each model
  for (const auto & name : {"mixed_field", "sloping_field"}) {
    if (!world_parser.model_exists(name)) {
      throw std::runtime_error(std::string{"Missing model '"} + name + "' in world file");
    }
    init_field_(name, world_parser.get_transform(name));
  }
}

void Evaluation::init_field_(const std::string & field_name, const Eigen::Affine3d & transform)
{
  if (fields_.count(field_name))
    throw std::runtime_error("Failed to load field " + field_name + ": it already exists");

  using std::placeholders::_1;
  using CollisionCb = std::function<void(const ContactsState &)>;
  using CoverageCb = std::function<void(const Float32Msg &)>;

  std::string data_file_param_name = field_name + "_data_file";
  std::string collision_topic_name = field_name + "/crop_collisions";
  std::string coverage_topic_name = field_name + "/coverage";
  std::string crushed_topic_name = field_name + "/crushed";

  fields_.emplace(field_name, FieldInterface{});
  FieldInterface & field = fields_[field_name];
  field.name = field_name;

  // Open CSV file to load all the crop positions and convert them to world coordinates
  node_->declare_parameter<std::string>(data_file_param_name);
  auto param = node_->get_parameter(data_file_param_name);
  field.data.load_csv(param.as_string(), transform);

  // Create a subscriber to listen contact events
  CollisionCb cb = [this, &field](const ContactsState & msg) { collision_callback_(field, msg); };
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  field.collision_sub = node_->create_subscription<ContactsState>(collision_topic_name, qos, cb);

  // Create a subscriber to listen to coverage
  CoverageCb coverage_cb = [this, &field](const Float32Msg & msg) { coverage_callback_(field, msg); };
  field.coverage_sub = node_->create_subscription<Float32Msg>(coverage_topic_name, qos, coverage_cb);

  field.crushed_pub = node_->create_publisher<Float32Msg>(crushed_topic_name, qos);

  crops_viewer_.add_field(field_name, field.data);
  info_viewer_.set_covered_percentage(field_name, 0);  // To create the field in the viewer
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Evaluation::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void Evaluation::coverage_callback_(FieldInterface & field, const Float32Msg & msg)
{
  info_viewer_.set_covered_percentage(field.name, msg.data * 100);
}

void Evaluation::collision_callback_(FieldInterface & field, const ContactsState & msg)
{
  for (const auto & state : msg.states) {
    if (field.data.crush_around(state)) {
      crops_viewer_.notify_change(field.name);

      auto crushed_percentage = field.data.get_crushed_ratio() * 100;
      info_viewer_.set_crushed_percentage(field.name, crushed_percentage);

      Float32Msg msg;
      msg.data = static_cast<float>(crushed_percentage);
      field.crushed_pub->publish(msg);
    }
  }
}

}  // namespace hackathon

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hackathon::Evaluation)

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

#include "hackathon_evaluation/crop_field.hpp"

namespace hackathon
{

Evaluation::Evaluation(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("evaluation", options))
{
  for (const auto & name : {"mixed_field", "sloping_field"}) {
    init_field_(name);
  }
}

void Evaluation::init_field_(const std::string & field_name)
{
  if (fields_.count(field_name))
    throw std::runtime_error("Failed to load field " + field_name + ": it already exists");

  using std::placeholders::_1;
  using CollisionCb = std::function<void(const ContactsState &)>;

  std::string data_file_param_name = field_name + "_data_file";
  std::string topic_name = "~/" + field_name + "/crop_collisions";

  fields_.insert({field_name, FieldInterface{}});
  FieldInterface & field = fields_[field_name];
  field.name = field_name;

  node_->declare_parameter<std::string>(data_file_param_name);
  auto param = node_->get_parameter(data_file_param_name);
  field.data.load_csv(param.as_string());

  CollisionCb cb = [this, &field](const ContactsState & msg) {
    collision_callback_(field, msg);
  };
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  field.sub = node_->create_subscription<ContactsState>(topic_name, qos, cb);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Evaluation::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void Evaluation::collision_callback_(FieldInterface & field, const ContactsState & msg)
{
  for(auto const & state : msg.states) {
    field.data.crush_around(state);
  }

  RCLCPP_INFO(node_->get_logger(), "Crushed crops: %.2lf%%", field.data.get_crushed_ratio() * 100);
}

}  // namespace hackathon

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hackathon::Evaluation)

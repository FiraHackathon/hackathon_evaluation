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
#include <rclcpp/node.hpp>

namespace hackathon
{

Evaluation::Evaluation(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("evaluation", options))
{
  using std::placeholders::_1;
  using CollisionCb = std::function<void(const ContactsState &)>;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  CollisionCb mixed_cb = [this](const ContactsState & msg) {
    collision_callback_(Field::mixed, msg);
  };
  contact_subscriptions_[Field::mixed] =
    node_->create_subscription<ContactsState>("mixed_field/crop_collisions", qos, mixed_cb);

  CollisionCb sloping_cb = [this](const ContactsState & msg) {
    collision_callback_(Field::sloping, msg);
  };
  contact_subscriptions_[Field::sloping] =
    node_->create_subscription<ContactsState>("sloping_field/crop_collisions", qos, sloping_cb);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Evaluation::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void Evaluation::collision_callback_(Field field_id, const Evaluation::ContactsState & msg)
{
}

}  // namespace hackathon

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hackathon::Evaluation)

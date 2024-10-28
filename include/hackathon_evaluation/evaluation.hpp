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

#ifndef HACKATHON_EVALUATION__EVALUATION_HPP_
#define HACKATHON_EVALUATION__EVALUATION_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <rclcpp/node.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <rclcpp/subscription.hpp>

namespace hackathon
{

class Evaluation
{
  enum Field: std::size_t {
    mixed = 0,
    sloping,

    count,  // number of fields
  };

  using ContactsState = gazebo_msgs::msg::ContactsState;
  using ContactSubscription = std::shared_ptr<rclcpp::Subscription<ContactsState>>;
  using ContactSubscriptions = std::array<ContactSubscription, Field::count>;

public:
  explicit Evaluation(const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  void collision_callback_(Field field_id, const Evaluation::ContactsState & msg);

private:
  rclcpp::Node::SharedPtr node_;
  ContactSubscriptions contact_subscriptions_;
};

}  // namespace hackathon

#endif

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

#ifndef HACKATHON_EVALUATION__COLLISION_COUNTER_HPP_
#define HACKATHON_EVALUATION__COLLISION_COUNTER_HPP_

#include <gazebo_msgs/msg/contacts_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/float32.hpp>

namespace hackathon
{

class CollisionCounter
{
public:
  using FloatMsg = std_msgs::msg::Float32;
  using FloatPublisher = rclcpp::Publisher<FloatMsg>::SharedPtr;
  using ContactsState = gazebo_msgs::msg::ContactsState;

public:
  CollisionCounter(rclcpp::Node::SharedPtr & node);

  std::size_t get_nb_collisions() const { return nb_collisions_; }

private:
  void collision_callback_(ContactsState const & msg);
  void timer_callback_();

private:
  std::size_t nb_collisions_ = 0;
  rclcpp::Time last_collision_time{};

  rclcpp::Node::SharedPtr node_;
  FloatPublisher pub_;
  rclcpp::SubscriptionBase::SharedPtr sub_;
};

}  // namespace hackathon

#endif

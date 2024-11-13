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

#include "hackathon_evaluation/collision_counter.hpp"

#include <rclcpp/create_timer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace hackathon
{

CollisionCounter::CollisionCounter(rclcpp::Node::SharedPtr & node) : node_{node}
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  pub_ = node_->create_publisher<FloatMsg>("collisions_count", 1);
  sub_ = node_->create_subscription<ContactsState>(
    "obstacle_collisions", 1, std::bind(&CollisionCounter::collision_callback_, this, _1));

  last_collision_time = node_->now();
}

void CollisionCounter::collision_callback_(const ContactsState &)
{
  using namespace std::chrono_literals;
  rclcpp::Time current_time = node_->now();
  if((current_time - last_collision_time) < 3s) {
    return;
  }

  ++nb_collisions_;
  last_collision_time = current_time;

  FloatMsg out;
  out.data = nb_collisions_;
  pub_->publish(out);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("collision_counter"), "nb collisions: " << nb_collisions_);
}

}  // namespace hackathon

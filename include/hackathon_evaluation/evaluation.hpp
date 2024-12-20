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

#include <Eigen/Geometry>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float32.hpp>
#include <unordered_map>

#include "hackathon_evaluation/collision_counter.hpp"
#include "hackathon_evaluation/geofence_checker.hpp"
#include "hackathon_evaluation/crop_field.hpp"
#include "hackathon_evaluation/crops_viewer.hpp"
#include "hackathon_evaluation/info_viewer.hpp"

namespace hackathon
{

class Evaluation
{
  using ContactsState = gazebo_msgs::msg::ContactsState;
  using ContactSubscription = std::shared_ptr<rclcpp::Subscription<ContactsState>>;
  using Float32Msg = std_msgs::msg::Float32;
  using Float32Subscription = std::shared_ptr<rclcpp::Subscription<Float32Msg>>;
  using Float32Publisher = std::shared_ptr<rclcpp::Publisher<Float32Msg>>;

  struct FieldInterface
  {
    std::string name;
    CropField data;
    ContactSubscription collision_sub;
    Float32Subscription coverage_sub;
    Float32Publisher crushed_pub;
  };

  using FieldInterfaces = std::unordered_map<std::string, FieldInterface>;

public:
  explicit Evaluation(const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  void init_field_(const std::string & field_name, const Eigen::Affine3d & transform);

private:
  void collision_callback_(FieldInterface & field, const ContactsState & msg);
  void coverage_callback_(FieldInterface & field, const Float32Msg & msg);

private:
  rclcpp::Node::SharedPtr node_;
  FieldInterfaces fields_;
  CropsViewer crops_viewer_;
  InfoViewer info_viewer_;
  CollisionCounter collision_counter_;
  GeofenceChecker geofence_checker_;
};

}  // namespace hackathon

#endif

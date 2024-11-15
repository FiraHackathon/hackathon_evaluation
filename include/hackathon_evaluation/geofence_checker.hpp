
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

#ifndef HACKATHON_EVALUATION__GEOFENCE_CHECKER_HPP_
#define HACKATHON_EVALUATION__GEOFENCE_CHECKER_HPP_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription_base.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::polygon<Point>                      Polygon;

namespace hackathon 
{

class GeofenceChecker 
{
  using FloatMsg = std_msgs::msg::Float32;
  using FloatPublisher = rclcpp::Publisher<FloatMsg>::SharedPtr;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

public:
  GeofenceChecker(rclcpp::Node::SharedPtr & node);
  std::size_t get_nb_exits() const { return nb_exits_; }

private:
  void pose_callback_(PoseStamped const & msg);
  void load_csv(const std::string & filename);

private:
  Polygon geofence_;
  std::size_t nb_exits_ = 0;
  rclcpp::Time last_exit_time{};

  rclcpp::Node::SharedPtr node_;
  FloatPublisher pub_;
  rclcpp::SubscriptionBase::SharedPtr sub_;

};
}

#endif // HACKATHON_EVALUATION__GEOFENCE_CHECKER_HPP_
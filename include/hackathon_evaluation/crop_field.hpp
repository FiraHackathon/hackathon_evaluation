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

#ifndef HACKATHON_EVALUATION__CROP_FIELD_HPP_
#define HACKATHON_EVALUATION__CROP_FIELD_HPP_

#include <Eigen/Dense>
#include <functional>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <string>
#include <vector>

namespace hackathon
{

struct Crop
{
  bool crushed = false;
  Eigen::Vector3d pos;

  Crop(Eigen::Vector3d pos) : pos(pos) {}
};

class CropField
{
public:
  using ContactState = gazebo_msgs::msg::ContactState;
  using Neighbors = std::vector<Crop*>;

public:
  /// Create Crops from a CSV file containing position of crop stems
  /// @return the number of loaded crops
  std::size_t load_csv(const std::string & filename);

  /// Crush the crops close to the given contact positions
  void crush_around(const ContactState & state);

  /// @return the the number of crushed crops / the total number of crops
  double get_crushed_ratio() const;

private:
  Neighbors get_neighborhood_(Eigen::Vector3d pos, double radius);

private:
  std::vector<Crop> crops_;
  std::size_t nb_crushed_ = 0;
};

}  // namespace hackathon

#endif

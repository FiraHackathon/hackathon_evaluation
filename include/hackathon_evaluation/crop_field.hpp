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
#include <Eigen/Geometry>
#include <gazebo_msgs/msg/contact_state.hpp>
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
  using Crops = std::vector<Crop>;
  using Neighbors = std::vector<Crop *>;

public:
  /// Create Crops from a CSV file containing position of crop stems
  /// @return the number of loaded crops
  std::size_t load_csv(const std::string & filename, const Eigen::Affine3d & transform);

  /// Crush the crops close to the given contact positions
  bool crush_around(const ContactState & state);

  /// @return the the number of crushed crops / the total number of crops
  double get_crushed_ratio() const;

  /// @return the the number of crushed crops
  std::size_t get_nb_crushed() const { return nb_crushed_; }

  const Crops & get_crops() const { return crops_; }

private:
  Neighbors get_neighborhood_(Eigen::Vector3d pos, double radius);

private:
  Crops crops_;
  std::size_t nb_crushed_ = 0;
};

}  // namespace hackathon

#endif

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
#include <algorithm>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <nanoflann.hpp>
#include <string>
#include <utility>
#include <vector>

namespace hackathon
{

struct Crop
{
  bool crushed = false;
  Eigen::Vector3d pos;

  Crop(Eigen::Vector3d pos) : pos(std::move(pos)) {}
};

class CropDataset
{
public:
  [[nodiscard]] size_t kdtree_get_point_count() const { return crops.size(); }

  [[nodiscard]] auto kdtree_get_pt(const size_t idx, int dim) const { return crops[idx].pos(dim); }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template<class BBOX>
  bool kdtree_get_bbox(BBOX & /*bb*/) const
  {
    return false;
  }

public:
  std::vector<Crop> crops;
};

class CropField
{
public:
  using ContactState = gazebo_msgs::msg::ContactState;
  using L2Distance = nanoflann::L2_Simple_Adaptor<double, CropDataset>;
  using KdTree = nanoflann::KDTreeSingleIndexAdaptor<L2Distance, CropDataset, 2>;

public:
  /// Create Crops from a CSV file containing position of crop stems
  /// @return the number of loaded crops
  std::size_t load_csv(const std::string & filename, const Eigen::Affine3d & transform);

  /// Crush the crops close to the given contact positions
  bool crush_around(const ContactState & state);

  /// @return the the number of crushed crops / the total number of crops
  [[nodiscard]] double get_crushed_ratio() const;

  /// @return the the number of crushed crops
  [[nodiscard]] std::size_t get_nb_crushed() const { return nb_crushed_; }

  [[nodiscard]] const std::vector<Crop> & get_crops() const { return dataset_.crops; }

private:
  [[nodiscard]] Crop * get_nearest_crop(const Eigen::Vector3d & point);

private:
  CropDataset dataset_;
  std::unique_ptr<KdTree> kdtree_;  // use unique_ptr because KdTree is not copyable
  std::size_t nb_crushed_ = 0;
};

}  // namespace hackathon

#endif

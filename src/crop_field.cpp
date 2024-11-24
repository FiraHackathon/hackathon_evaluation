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

#include "hackathon_evaluation/crop_field.hpp"

#include <fstream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "nanoflann.hpp"

namespace hackathon
{

namespace
{

using Points = std::vector<Eigen::Vector3d>;
using ContactPoints = CropField::ContactState::_contact_positions_type;

Points to_eigen_vectors(const ContactPoints & points)
{
  Points res;
  res.reserve(points.size());
  for (const auto & point : points) {
    res.emplace_back(point.x, point.y, point.z);
  }
  return res;
}

}  // namespace

std::size_t CropField::load_csv(const std::string & filename, const Eigen::Affine3d & transform)
{
  constexpr auto separator = ',';
  auto & crops = dataset_.crops;
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("failed to open CSV data file");
  }

  crops.clear();

  std::string line;

  // skip first line (header)
  if (!std::getline(file, line)) {
    throw std::runtime_error("failed to read CSV file " + filename + ": file is empty");
  }

  std::size_t i = 2;
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string object, id, x, y, z;

    std::getline(ss, object, separator);
    std::getline(ss, id, separator);
    std::getline(ss, x, separator);
    std::getline(ss, y, separator);
    std::getline(ss, z, separator);

    try {
      Eigen::Vector3d point{std::stod(x), std::stod(y), std::stod(z)};
      crops.emplace_back(transform * point);
    } catch (const std::exception &) {
      throw std::runtime_error("malformed number in " + filename + ", line " + std::to_string(i));
    }

    if (!ss) {
      throw std::runtime_error("malformed line in " + filename + ", line" + std::to_string(i));
    }

    ++i;
  }

  file.close();

  kdtree_ = std::make_unique<KdTree>(2, dataset_);
  kdtree_->buildIndex();

  return crops.size();
}

bool CropField::crush_around(const ContactState & state)
{
  if (state.contact_positions.empty()) {
    return false;
  }

  // Get nearest crops to avoid iterate all crops for each contact points
  Points points = to_eigen_vectors(state.contact_positions);
  bool crushed = false;

  for (const auto & point : points) {
    Crop * closest_crop = get_nearest_crop(point);
    if (closest_crop != nullptr && !closest_crop->crushed) {
      closest_crop->crushed = true;
      ++nb_crushed_;
      crushed = true;
    }
  }

  return crushed;
}

double CropField::get_crushed_ratio() const
{
  return static_cast<double>(nb_crushed_) / static_cast<double>(dataset_.crops.size());
}

Crop * CropField::get_nearest_crop(const Eigen::Vector3d & point)
{
  std::size_t index{};
  double distance{};
  nanoflann::KNNResultSet<double> result{1};
  result.init(&index, &distance);

  kdtree_->findNeighbors(result, point.data());
  return result.size() ? &dataset_.crops[index] : nullptr;
}

}  // namespace hackathon

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
#include <limits>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

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

Crop * get_nearest_crop(const CropField::Neighbors & crops, Eigen::Vector3d point)
{
  double current_dist = std::numeric_limits<double>::infinity();
  Crop * current_crop = nullptr;

  for (const auto & crop : crops) {
    double dist = (point - crop->pos).squaredNorm();
    if (dist < current_dist) {
      current_dist = dist;
      current_crop = crop;
    }
  }

  return current_crop;
}

}  // namespace

std::size_t CropField::load_csv(const std::string & filename, Eigen::Affine3d const & transform)
{
  constexpr auto separator = ',';
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("failed to open CSV data file");
  }

  crops_.clear();

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
      crops_.emplace_back(transform * point);
    } catch (const std::exception &) {
      throw std::runtime_error("malformed number in " + filename + ", line " + std::to_string(i));
    }

    if (!ss) {
      throw std::runtime_error("malformed line in " + filename + ", line" + std::to_string(i));
    }

    ++i;
  }

  file.close();  // Close the file

  return crops_.size();
}

bool CropField::crush_around(const ContactState & state)
{
  constexpr double neighbor_radius = 0.5;

  if (state.contact_positions.empty()) {
    return false;
  }

  // Get nearest crops to avoid iterate all crops for each contact points
  Points points = to_eigen_vectors(state.contact_positions);
  auto neighbors = get_neighborhood_(points.front(), neighbor_radius);
  bool crushed = false;

  for (const auto & point : points) {
    Crop * closest_crop = get_nearest_crop(neighbors, point);
    if(closest_crop != nullptr && !closest_crop->crushed) {
      closest_crop->crushed = true;
      ++nb_crushed_;
      crushed = true;
    }
  }

  return crushed;
}

CropField::Neighbors CropField::get_neighborhood_(Eigen::Vector3d pos, double radius)
{
  double sq_radius = radius * radius;
  CropField::Neighbors neighbors;

  for (auto & crop : crops_) {
    if ((pos - crop.pos).squaredNorm() < sq_radius) {
      neighbors.push_back(&crop);
    }
  }

  return neighbors;
}

double CropField::get_crushed_ratio() const
{
  return static_cast<double>(nb_crushed_) / crops_.size();
}

}  // namespace hackathon

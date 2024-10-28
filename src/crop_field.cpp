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
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>

namespace hackathon
{

std::size_t CropField::load_csv(const std::string & filename)
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
      crops_.emplace_back(Eigen::Vector3d{std::stod(x), std::stod(y), std::stod(z)});
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

}  // namespace hackathon

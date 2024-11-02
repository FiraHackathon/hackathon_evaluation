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

#include "hackathon_evaluation/xml_world_parser.hpp"

#include <tinyxml2.h>

#include <Eigen/Geometry>
#include <sstream>
#include <stdexcept>
#include <iostream>

namespace hackathon
{

namespace
{

auto rpy(double roll, double pitch, double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

Eigen::Affine3d read_pose(const std::string & pose_text)
{
  std::istringstream iss(pose_text);
  double x, y, z, roll, pitch, yaw;
  iss >> x >> y >> z >> roll >> pitch >> yaw;
  return Eigen::Translation3d{x, y, z} * rpy(roll, pitch, yaw);
}

}  // namespace

XmlWorldParser::XmlWorldParser(const std::string & filename)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to open world file '" + filename + "'");
  }

  auto root = doc.RootElement();
  auto element = root->FirstChildElement("world")->FirstChildElement("include");
  while (element != nullptr) {
    load_include_element_(element);
    element = element->NextSiblingElement("include");
  }
}

const Eigen::Affine3d & XmlWorldParser::get_transform(const std::string & model_name)
{
  return transforms_[model_name];
}

bool XmlWorldParser::model_exists(const std::string & name)
{
  return transforms_.count(name);
}

void XmlWorldParser::load_include_element_(tinyxml2::XMLElement * include_element)
{
  std::string name = include_element->FirstChildElement("name")->GetText();
  std::string pose_text = include_element->FirstChildElement("pose")->GetText();
  auto tf = read_pose(pose_text);

  std::cerr << "name: " << name << ", tf:\n" << tf.matrix() << std::endl;

  transforms_.insert({std::move(name), tf});
}

}  // namespace hackathon

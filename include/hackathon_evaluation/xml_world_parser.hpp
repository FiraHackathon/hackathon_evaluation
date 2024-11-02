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

#ifndef HACKATHON_EVALUATION__XML_WORLD_PARSER_HPP_
#define HACKATHON_EVALUATION__XML_WORLD_PARSER_HPP_

#include <Eigen/Geometry>
#include <map>
#include <string>

namespace tinyxml2
{
struct XMLElement;
}

namespace hackathon
{

class XmlWorldParser
{
public:
  using ModelTransforms = std::map<std::string, Eigen::Affine3d>;

public:
  XmlWorldParser(const std::string & filename);

  const Eigen::Affine3d & get_transform(const std::string & model_name);

  bool model_exists(const std::string & name);

private:
   /// Create transform from the pose in the given 'include' element
  void load_include_element_(tinyxml2::XMLElement * include_element);

private:
  ModelTransforms transforms_;
};

}  // namespace hackathon

#endif

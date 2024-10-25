#ifndef GAZEBO_CLASSIC_TOOLS__FIELD_COVERAGE_HPP_
#define GAZEBO_CLASSIC_TOOLS__FIELD_COVERAGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "field_coverage/field_grid.hpp"

namespace romea
{
  class FieldCoverage : public rclcpp::Node
  {
  public:
    FieldCoverage();
  private:
      std::vector<FieldGrid> fields_;
  };
}

#endif // GAZEBO_CLASSIC_TOOLS__FIELD_COVERAGE_HPP_
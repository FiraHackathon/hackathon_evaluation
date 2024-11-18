#include "field_coverage/field_grid.hpp"
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/detail/distance/interface.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/algorithms/detail/within/interface.hpp>
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace hackathon
{
  FieldGrid::FieldGrid(std::string name, double x, double y, double z,
                       double roll, double pitch, double yaw,
                       double width, double height, double resolution, double headland) :
    name_(name),
    intersected_cells_(0),
    full_area_({0, 0}, {width, height}),
    field_headland_(headland)
  {
    world_pos_ = Eigen::Vector3d(x, y, z);

    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yaw_angle * pitch_angle * roll_angle;
    Eigen::Matrix3d           rotation = q.toRotationMatrix();
    rotation.transposeInPlace();

    world_to_field_.linear() = rotation;

    Eigen::Vector3d translation{x, y, z};
    world_to_field_.translation() = -rotation * translation;

    n_rows_ = ceil(height / resolution);
    n_cols_ = ceil(width / resolution);

    for (int i = 0; i < n_cols_; ++i)
    {
      for (int j = 0; j < n_rows_; ++j)
      {
        double minX = i * resolution;
        double minY = j * resolution;
        double maxX = minX + resolution;
        double maxY = minY + resolution;
        cells_.push_back({Box(Point(minX, minY), Point(maxX, maxY))});
      }
    }
  }

  bool FieldGrid::collisionCallback(Eigen::Vector3d corner1,
                                    Eigen::Vector3d corner2,
                                    Eigen::Vector3d corner3,
                                    Eigen::Vector3d corner4)
  {
    corner1 = world_to_field_ * corner1;
    corner2 = world_to_field_ * corner2;
    corner3 = world_to_field_ * corner3;
    corner4 = world_to_field_ * corner4;

    Polygon tool_foot_print{{{corner1.x(), corner1.y()},
                             {corner2.x(), corner2.y()},
                             {corner3.x(), corner3.y()},
                             {corner4.x(), corner4.y()},
                             {corner1.x(), corner1.y()}}};

    Point tool_centroid;
    bg::centroid(tool_foot_print, tool_centroid);
    double distance = bg::distance(full_area_, tool_centroid);

    // TODO: remote it. Temporary fix for a strange bug
    if(distance > 1e8) {
      distance = 0;
    }

    if (distance < field_headland_) {
      checkIntersections(tool_foot_print);
      return true;
    }
    return false;
  }

  void FieldGrid::checkIntersections(const Polygon &collision_polygon)
  {
    for (auto &cell : cells_)
    {
      if (!cell.intersected && bg::intersects(cell.box, collision_polygon))
      {
        cell.intersected = true;
        ++intersected_cells_;
      }
    }
  }

  double FieldGrid::getIntersectedPercentage() const
  {
    return (100.0 * intersected_cells_ / (n_rows_ * n_cols_));
  }

  std::string FieldGrid::getName() const
  {
    return name_;
  }

  Eigen::Vector3d FieldGrid::getWordlPos() const
  {
    return world_pos_;
  }
} // namespace romea

#include "field_coverage/field_grid.hpp"
#include <cmath>

namespace hackathon
{
  FieldGrid::FieldGrid(std::string name, double x, double y, double z,
                       double roll, double pitch, double yaw,
                       double width, double height, double resolution) : name_(name),
                                                                      intersected_cells_(0)
  {
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = yaw_angle * pitch_angle * roll_angle;
    Eigen::Matrix3d rotation = q.toRotationMatrix();

    world_to_field_.linear() = rotation;

    Eigen::Vector3d translation{x, y, z};
    world_to_field_.translation() = translation;

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

  void FieldGrid::collisionCallback(Eigen::Vector3d corner1,
                                     Eigen::Vector3d corner2,
                                     Eigen::Vector3d corner3,
                                     Eigen::Vector3d corner4)
  {
    corner1 = world_to_field_ * corner1;
    corner2 = world_to_field_ * corner2;
    corner3 = world_to_field_ * corner3;
    corner4 = world_to_field_ * corner4;

    Box tool_foot_print(Point(corner1.x(), corner1.y()),
    Point(corner2.x(), corner2.y()),
    Point(corner3.x(), corner3.y()),
    Point(corner4.x(), corner4.y()));

    checkIntersections(tool_foot_print);
  }

  void FieldGrid::checkIntersections(const Box &collision_box)
  {
    for (auto &cell : cells_)
    {
      if (!cell.intersected && bg::intersects(cell.box, collision_box))
      {
        cell.intersected = true;
        ++intersected_cells_;
      }
    }
  }

  double FieldGrid::getIntersectedPercentage() const
  {
    return ((float)intersected_cells_ / (n_rows_ * n_cols_));
  }

  std::string FieldGrid::getName() const
  {
    return name_;
  }
} // namespace romea

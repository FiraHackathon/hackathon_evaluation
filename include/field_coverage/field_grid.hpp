#ifndef GAZEBO_CLASSIC_TOOLS__FIELD_GRID_HPP_
#define GAZEBO_CLASSIC_TOOLS__FIELD_GRID_HPP_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::box<Point>                          Box;

namespace romea
{
  struct GridCell
  {
    Box  box;
    bool intersected = false;
  };

  class FieldGrid
  {
public:
    void        checkIntersections();
    double      getIntersectedPercentage() const;
    std::string getName() const;

private:
    std::string           name_;
    int                   intersected_cells_;
    float                 resolution;
    float                 height;
    float                 width;
    std::vector<GridCell> cells_;
  };
} // namespace romea

#endif // GAZEBO_CLASSIC_TOOLS__FIELD_GRID_HPP_
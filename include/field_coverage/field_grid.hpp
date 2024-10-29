#ifndef HACKATHON_EVALUATION__FIELD_GRID_HPP_
#define HACKATHON_EVALUATION__FIELD_GRID_HPP_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::box<Point>                          Box;

namespace hackathon
{
  struct GridCell
  {
    Box  box;
    bool intersected = false;
  };

  class FieldGrid
  {
public:
    FieldGrid(std::string name, double x, double y, double z,
              double roll, double pitch, double yaw,
              double width, double height, double resolution);

    void collisionCallback(Eigen::Vector3d corner1,
                            Eigen::Vector3d corner2,
                            Eigen::Vector3d corner3,
                            Eigen::Vector3d corner4);

    void        checkIntersections(const Box &collision_box);
    double      getIntersectedPercentage() const;
    std::string getName() const;

private:
    std::string           name_;
    Eigen::Vector3d       world_pos_;
    int                   n_rows_, n_cols_;
    std::vector<GridCell> cells_;
    Eigen::Affine3d       world_to_field_;
    int                   intersected_cells_;
  };
} // namespace romea

#endif // HACKATHON_EVALUATION__FIELD_GRID_HPP_
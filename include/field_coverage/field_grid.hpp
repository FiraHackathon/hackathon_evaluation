#ifndef HACKATHON_EVALUATION__FIELD_GRID_HPP_
#define HACKATHON_EVALUATION__FIELD_GRID_HPP_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::box<Point>                          Box;
typedef bg::model::polygon<Point>                      Polygon;

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
              double width, double height, double resolution, double headland);

    /// Update intersected cells and return true if centroid is inside the extended field
    bool collisionCallback(Eigen::Vector3d corner1,
                           Eigen::Vector3d corner2,
                           Eigen::Vector3d corner3,
                           Eigen::Vector3d corner4);

    void            checkIntersections(const Polygon &collision_polygon);
    double          getIntersectedPercentage() const;
    std::string     getName() const;
    Eigen::Vector3d getWordlPos() const;

  private:
    std::string           name_;
    Eigen::Vector3d       world_pos_;
    int                   intersected_cells_;
    int                   n_rows_, n_cols_;
    std::vector<GridCell> cells_;
    Eigen::Affine3d       world_to_field_;
    Box                   full_area_;
    double                field_headland_;
  };
} // namespace hackathon

#endif // HACKATHON_EVALUATION__FIELD_GRID_HPP_

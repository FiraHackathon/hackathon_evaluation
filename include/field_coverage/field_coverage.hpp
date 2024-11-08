#ifndef HACKATHON_EVALUATION__FIELD_COVERAGE_HPP_
#define HACKATHON_EVALUATION__FIELD_COVERAGE_HPP_

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

#include "field_coverage/field_grid.hpp"

namespace hackathon
{
  class FieldCoverage : public rclcpp::Node
  {
  public:
    FieldCoverage(const rclcpp::NodeOptions &options);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void getToolCorners(const geometry_msgs::msg::TransformStamped &transform,
                        Eigen::Vector3d                            &up_right,
                        Eigen::Vector3d                            &up_left,
                        Eigen::Vector3d                            &bottom_left,
                        Eigen::Vector3d                            &bottom_right) const;

  private:
    std::unique_ptr<tf2_ros::Buffer>                                  tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                       tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_state_sub_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> coverage_pubs_;

    std::vector<FieldGrid> fields_;
    std::string            world_frame_;
    std::string            tool_joint_;
    std::string            tool_frame_;
    double                 tool_length_;
    double                 tool_width_;
    double                 tool_center_offset_x_;
    double                 tool_center_offset_y_;
  };
} // namespace hackathon

#endif // HACKATHON_EVALUATION__FIELD_COVERAGE_HPP_
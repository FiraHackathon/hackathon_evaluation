#ifndef HACKATHON_EVALUATION__FIELD_COVERAGE_HPP_
#define HACKATHON_EVALUATION__FIELD_COVERAGE_HPP_

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

#include "field_coverage/field_grid.hpp"

namespace hackathon
{
  class FieldCoverage : public rclcpp::Node
  {
  public:
    using FloatMsg = std_msgs::msg::Float32;
    using PoseMsg = geometry_msgs::msg::PoseStamped;
    using PoseSubscription = rclcpp::Subscription<PoseMsg>::SharedPtr;
    using FloatPublisher = rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr;
    using JointSubscription = rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr;
    using Seconds = std::chrono::duration<double>;

  public:
    FieldCoverage(const rclcpp::NodeOptions &options);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void getToolCorners(Eigen::Vector3d &up_right,
                        Eigen::Vector3d &up_left,
                        Eigen::Vector3d &bottom_left,
                        Eigen::Vector3d &bottom_right) const;

  private:
    void implementCallback_(const PoseMsg & msg);
    void updateOutsideCounter_();

  private:
    JointSubscription           joint_state_sub_;
    PoseSubscription            pose_sub_;
    std::vector<FloatPublisher> coverage_pubs_;
    FloatPublisher              outside_pub_;

    std::vector<FieldGrid> fields_;
    std::string            world_frame_;
    std::string            tool_joint_;
    std::string            tool_frame_;
    double                 tool_length_;
    double                 tool_width_;
    double                 tool_center_offset_x_;
    double                 tool_center_offset_y_;
    Eigen::Affine3d        tool_to_world_;
    std::size_t            tool_outside_counter_;
    Seconds                tool_outside_period_;
    rclcpp::Time           last_update_time_;
  };
} // namespace hackathon

#endif // HACKATHON_EVALUATION__FIELD_COVERAGE_HPP_

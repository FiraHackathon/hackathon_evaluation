#include "field_coverage/field_coverage.hpp"
#include <Eigen/Core>
#include <rclcpp/logging.hpp>

namespace hackathon
{
  FieldCoverage::FieldCoverage(const rclcpp::NodeOptions &options) : Node("field_coverage_node", options)
  {
    tool_length_          = this->get_parameter("tool_length").as_double();
    tool_width_           = this->get_parameter("tool_width").as_double();
    tool_joint_           = this->get_parameter("tool_joint").as_string();
    tool_frame_           = this->get_parameter("tool_frame").as_string();
    tool_center_offset_x_ = this->get_parameter("tool_center_offset.x").as_double();
    tool_center_offset_y_ = this->get_parameter("tool_center_offset.y").as_double();
    tool_frame_           = this->get_parameter("tool_frame").as_string();
    world_frame_          = this->get_parameter("world_frame").as_string();
    std::string              joint_state_topic = this->get_parameter("joint_state_topic").as_string();
    std::vector<std::string> fields_names = this->get_parameter("field_names").as_string_array();
    std::string              pose_topic = get_parameter("implement_pose_topic").as_string();

    for (const auto &field_name : fields_names)
    {
      double x = this->get_parameter("fields." + field_name + ".position.x").as_double();
      double y = this->get_parameter("fields." + field_name + ".position.y").as_double();
      double z = this->get_parameter("fields." + field_name + ".position.z").as_double();

      double roll  = this->get_parameter("fields." + field_name + ".rotation.r").as_double();
      double pitch = this->get_parameter("fields." + field_name + ".rotation.p").as_double();
      double yaw   = this->get_parameter("fields." + field_name + ".rotation.y").as_double();

      double width      = this->get_parameter("fields." + field_name + ".width").as_double();
      double height     = this->get_parameter("fields." + field_name + ".height").as_double();
      double resolution = this->get_parameter("fields." + field_name + ".resolution").as_double();

      fields_.push_back(FieldGrid(field_name, x, y, z, roll, pitch, yaw, width, height, resolution));

      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr field_pub =
          this->create_publisher<std_msgs::msg::Float32>(field_name + "/coverage", 10);
      coverage_pubs_.push_back(field_pub);
    }

    using std::placeholders::_1;
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                        joint_state_topic,
                        10,
                        std::bind(&FieldCoverage::jointStateCallback, this, _1));

    pose_sub_ = create_subscription<PoseMsg>(
      pose_topic, 1, std::bind(&FieldCoverage::implementCallback_, this, _1));

    RCLCPP_INFO(this->get_logger(), "Initialization complete");
  }

  void FieldCoverage::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto it = std::find(msg->name.cbegin(), msg->name.cend(), tool_joint_);
    if (it == msg->name.cend())
    {
      throw std::runtime_error("Joint not found in joint_states : " + tool_joint_);
    }
    auto  index = std::distance(msg->name.cbegin(), it);
    float tool_pos = msg->position[index];

    if (tool_pos > 0.7)
    {
      // Get closest field
      FieldGrid      *field;
      float           min_dist = FLT_MAX;
      Eigen::Vector3d tool_pos = tool_to_world_.translation();
      for (auto &f : fields_)
      {
        double dist = (tool_pos - f.getWordlPos()).norm();
        if (dist < min_dist)
        {
          min_dist = dist;
          field = &f;
        }
      }

      Eigen::Vector3d up_right, up_left, bottom_left, bottom_right;
      getToolCorners(up_right, up_left, bottom_left, bottom_right);
      field->collisionCallback(up_left, up_right, bottom_left, bottom_right);
    }

    for (size_t i = 0; i < fields_.size(); i++)
    {
      std_msgs::msg::Float32 msg;
      msg.data = fields_[i].getIntersectedPercentage();
      coverage_pubs_[i]->publish(msg);
    }
  }

  void FieldCoverage::getToolCorners( Eigen::Vector3d &up_right,
                                     Eigen::Vector3d &up_left,
                                     Eigen::Vector3d &bottom_left,
                                     Eigen::Vector3d &bottom_right) const
  {
    up_right = tool_to_world_ * Eigen::Vector3d{tool_length_ / 2 + tool_center_offset_x_,
                                              tool_width_ / 2 + tool_center_offset_y_,
                                              0};

    up_left = tool_to_world_ * Eigen::Vector3d{tool_length_ / 2 + tool_center_offset_x_,
                                              -tool_width_ / 2 + tool_center_offset_y_,
                                              0};

    bottom_left = tool_to_world_ * Eigen::Vector3d{-tool_length_ / 2 + tool_center_offset_x_,
                                                  -tool_width_ / 2 + tool_center_offset_y_,
                                                  0};

    bottom_right = tool_to_world_ * Eigen::Vector3d{-tool_length_ / 2 + tool_center_offset_x_,
                                                    tool_width_ / 2 + tool_center_offset_y_,
                                                    0};
  }

  void FieldCoverage::implementCallback_(const PoseMsg & msg)
  {
    const auto & pos = msg.pose.position;
    const auto & q = msg.pose.orientation;
    tool_to_world_.translation() = Eigen::Vector3d{pos.x, pos.y, pos.z};
    tool_to_world_.linear() = Eigen::Quaterniond{q.w, q.x, q.y, q.z}.toRotationMatrix();
  }
}  // namespace hackathon

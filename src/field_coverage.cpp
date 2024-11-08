#include "field_coverage/field_coverage.hpp"

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
          this->create_publisher<std_msgs::msg::Float32>("/evaluation/" + field_name + "/coverage", 10);
      coverage_pubs_.push_back(field_pub);
    }

    tf_buffer_       = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_     = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                        joint_state_topic,
                        10,
                        std::bind(&FieldCoverage::jointStateCallback, this, std::placeholders::_1));

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
      geometry_msgs::msg::TransformStamped tool_map_tf = tf_buffer_->lookupTransform(
          world_frame_,
          tool_frame_,
          tf2::TimePointZero);

      // Get closest field
      FieldGrid      *field;
      float           min_dist = FLT_MAX;
      Eigen::Vector3d tool_pos{tool_map_tf.transform.translation.x,
                               tool_map_tf.transform.translation.y,
                               tool_map_tf.transform.translation.z};

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
      getToolCorners(tool_map_tf, up_right, up_left, bottom_left, bottom_right);

      field->collisionCallback(up_left, up_right, bottom_left, bottom_right);
    }

    for (size_t i = 0; i < fields_.size(); i++)
    {
      std_msgs::msg::Float32 msg;
      msg.data = fields_[i].getIntersectedPercentage();
      coverage_pubs_[i]->publish(msg);
    }
  }

  void FieldCoverage::getToolCorners(const geometry_msgs::msg::TransformStamped &transform,
                                     Eigen::Vector3d &up_right,
                                     Eigen::Vector3d &up_left,
                                     Eigen::Vector3d &bottom_left,
                                     Eigen::Vector3d &bottom_right) const
  {
    geometry_msgs::msg::Vector3    position = transform.transform.translation;
    geometry_msgs::msg::Quaternion orientation = transform.transform.rotation;

    Eigen::Affine3d tool_to_world;
    tool_to_world.translation() = Eigen::Vector3d({position.x, position.y, position.z});
    tool_to_world.linear() = Eigen::Quaternion<double>(orientation.w, orientation.x, orientation.y, orientation.z).toRotationMatrix();

    up_right = tool_to_world * Eigen::Vector3d{tool_length_ / 2 + tool_center_offset_x_,
                                              tool_width_ / 2 + tool_center_offset_y_,
                                              0};

    up_left = tool_to_world * Eigen::Vector3d{tool_length_ / 2 + tool_center_offset_x_,
                                              -tool_width_ / 2 + tool_center_offset_y_,
                                              0};

    bottom_left = tool_to_world * Eigen::Vector3d{-tool_length_ / 2 + tool_center_offset_x_,
                                                  -tool_width_ / 2 + tool_center_offset_y_,
                                                  0};

    bottom_right = tool_to_world * Eigen::Vector3d{-tool_length_ / 2 + tool_center_offset_x_,
                                                    tool_width_ / 2 + tool_center_offset_y_,
                                                    0};
  }
}
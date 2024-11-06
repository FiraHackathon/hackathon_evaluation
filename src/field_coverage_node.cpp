#include "field_coverage/field_coverage.hpp"

int main(int argc, char *argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(args);
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<hackathon::FieldCoverage>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
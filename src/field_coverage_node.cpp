#include "field_coverage/field_coverage.hpp"

int main(int argc, char *argv[])
{
  auto node = std::make_shared<hackathon::FieldCoverage>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

#include "drone_line_vision/line_vision_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_line_vision::LineVisionNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("line_vision_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

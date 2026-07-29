#include "drone_line_vision/line_vision_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_line_vision::LineVisionNode>());
  rclcpp::shutdown();
  return 0;
}

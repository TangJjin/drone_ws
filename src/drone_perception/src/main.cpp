#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_perception/industrial_animal_vision_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(drone_perception::makeIndustrialAnimalVisionNode());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("industrial_animal_vision"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

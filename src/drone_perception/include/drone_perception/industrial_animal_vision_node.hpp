#pragma once

#include <memory>

#include <rclcpp/node.hpp>

namespace drone_perception
{
std::shared_ptr<rclcpp::Node> makeIndustrialAnimalVisionNode();
}  // namespace drone_perception

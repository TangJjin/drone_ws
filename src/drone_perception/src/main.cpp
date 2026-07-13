#include <cerrno>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <sched.h>

#include <rclcpp/rclcpp.hpp>

#include "drone_perception/qr_vision_node.hpp"

namespace
{
// 对齐 package_qr_shelf_rknn_probe：进程绑 RK3588 大核 4–7
void bindToPerformanceCpus()
{
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  for (int cpu = 4; cpu <= 7; ++cpu) {
    CPU_SET(cpu, &cpu_set);
  }
  if (sched_setaffinity(0, sizeof(cpu_set), &cpu_set) != 0) {
    std::cerr << "WARN: failed to bind CPU affinity to cores 4-7: "
              << std::strerror(errno) << '\n';
    return;
  }
  std::cout << "CPU affinity : cores 4-7\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  bindToPerformanceCpus();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QrVisionNode>());
  rclcpp::shutdown();
  return 0;
}

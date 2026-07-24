#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>

class RealsenseColorPublisher : public rclcpp::Node
{
public:
  RealsenseColorPublisher()
  : Node("realsense_color_transport_publisher")
  {
    const int width = declare_parameter("width", 640);
    const int height = declare_parameter("height", 480);
    const int fps = declare_parameter("fps", 30);
    const std::string topic = declare_parameter("topic", "/realsense/color/image_raw");

    if (width <= 0 || height <= 0 || fps <= 0) {
      throw std::invalid_argument("width, height and fps must be positive");
    }

    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    pipeline_.start(config);

    publisher_ = image_transport::create_publisher(this, topic, rmw_qos_profile_sensor_data);
    RCLCPP_INFO(get_logger(), "Publishing BGR8 %dx%d @ %d FPS on %s",
      width, height, fps, topic.c_str());
  }

  void run()
  {
    while (rclcpp::ok()) {
      const rs2::frameset frames = pipeline_.wait_for_frames();
      const rs2::video_frame color = frames.get_color_frame();
      if (!color) {
        continue;
      }

      const int width = color.get_width();
      const int height = color.get_height();
      cv::Mat image(height, width, CV_8UC3, const_cast<void *>(color.get_data()),
        color.get_stride_in_bytes());
      std_msgs::msg::Header header;
      header.stamp = now();
      publisher_.publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
      rclcpp::spin_some(get_node_base_interface());
    }
    pipeline_.stop();
  }

private:
  rs2::pipeline pipeline_;
  image_transport::Publisher publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RealsenseColorPublisher>();
    node->run();
  } catch (const std::exception & error) {
    fprintf(stderr, "realsense color publisher: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

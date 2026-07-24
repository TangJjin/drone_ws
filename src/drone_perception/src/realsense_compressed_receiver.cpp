#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

class RealsenseCompressedReceiver : public rclcpp::Node
{
public:
  RealsenseCompressedReceiver()
  : Node("realsense_compressed_receiver"), last_frame_(now())
  {
    const std::string topic = declare_parameter("topic", "/realsense/color/image_raw");
    const std::string window = declare_parameter("window", "RealSense compressed");
    const bool has_display = std::getenv("DISPLAY") != nullptr;
    display_ = declare_parameter("display", has_display);
    window_ = window;

    image_transport::TransportHints hints(this, "compressed");
    subscription_ = image_transport::create_subscription(
      this, topic,
      std::bind(&RealsenseCompressedReceiver::on_image, this, std::placeholders::_1),
      hints.getTransport(), rmw_qos_profile_sensor_data);
    RCLCPP_INFO(get_logger(), "Subscribing to %s using %s transport",
      topic.c_str(), hints.getTransport().c_str());
    if (!display_) {
      RCLCPP_INFO(get_logger(), "Display disabled; receiving frames without an OpenCV window");
    }
  }

  bool display_enabled() const { return display_; }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    try {
      cv::Mat frame = cv_bridge::toCvShare(message, "bgr8")->image;
      const auto current = now();
      const double elapsed = (current - last_frame_).seconds();
      if (elapsed > 0.0) {
        fps_ = 0.9 * fps_ + 0.1 / elapsed;
      }
      last_frame_ = current;
      ++frames_;

      if (display_) {
        cv::Mat display = frame.clone();
        cv::putText(display, cv::format("compressed %.1f FPS  %dx%d", fps_,
          display.cols, display.rows), cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
          0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::imshow(window_, display);
        if ((cv::waitKey(1) & 0xff) == 27) {
          rclcpp::shutdown();
        }
      }
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot convert image (%s): %s", message->encoding.c_str(), error.what());
    }
  }

  image_transport::Subscriber subscription_;
  std::string window_;
  bool display_{false};
  rclcpp::Time last_frame_;
  double fps_{0.0};
  uint64_t frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseCompressedReceiver>();
  if (node->display_enabled()) {
    cv::namedWindow(node->get_parameter("window").as_string(), cv::WINDOW_NORMAL);
  }
  rclcpp::spin(node);
  if (node->display_enabled()) {
    cv::destroyAllWindows();
  }
  rclcpp::shutdown();
  return 0;
}

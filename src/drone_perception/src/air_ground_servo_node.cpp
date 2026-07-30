#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "drone_perception/air_ground_servo_depth.hpp"

namespace drone_perception
{

class AirGroundServoNode : public rclcpp::Node
{
public:
  AirGroundServoNode()
  : Node("air_ground_servo_node")
  {
    rgbd_topic_ = declare_parameter<std::string>("rgbd_topic", "/camera/camera/rgbd");
    expected_width_ = declare_parameter<int>("expected_width", 640);
    expected_height_ = declare_parameter<int>("expected_height", 480);
    expected_fps_ = declare_parameter<double>("expected_fps", 30.0);
    center_x_ = declare_parameter<int>("center_x", 320);
    center_y_ = declare_parameter<int>("center_y", 240);
    sample_window_size_ = declare_parameter<int>("sample_window_size", 10);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.1);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.0);
    fps_smoothing_alpha_ = declare_parameter<double>("fps_smoothing_alpha", 0.1);
    window_name_ = declare_parameter<std::string>("window_name", "Air-Ground Servo RGBD");
    debug_view_ = declare_parameter<bool>("debug_view", true);
    log_throttle_ms_ = declare_parameter<int>("log_throttle_ms", 2000);

    validateParameters();

    if (debug_view_) {
      try {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        window_created_ = true;
      } catch (const cv::Exception &error) {
        throw std::runtime_error("failed to create RGBD debug window: " + std::string(error.what()));
      }
    }

    rgbd_sub_ = create_subscription<realsense2_camera_msgs::msg::RGBD>(
      rgbd_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AirGroundServoNode::handleRgbd, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Air-ground RGBD debug ready: topic=%s expected=%dx%d@%.1f center=(%d,%d) window=%dx%d",
      rgbd_topic_.c_str(), expected_width_, expected_height_, expected_fps_, center_x_, center_y_,
      sample_window_size_, sample_window_size_);
  }

  ~AirGroundServoNode() override
  {
    if (window_created_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void validateParameters() const
  {
    if (rgbd_topic_.empty()) {
      throw std::invalid_argument("rgbd_topic must not be empty");
    }
    if (expected_width_ <= 0 || expected_height_ <= 0 || expected_fps_ <= 0.0) {
      throw std::invalid_argument("expected_width, expected_height and expected_fps must be positive");
    }
    if (center_x_ < 0 || center_x_ >= expected_width_ ||
      center_y_ < 0 || center_y_ >= expected_height_)
    {
      throw std::invalid_argument("center_x and center_y must be inside the expected image");
    }
    if (sample_window_size_ <= 0 || sample_window_size_ > expected_width_ ||
      sample_window_size_ > expected_height_)
    {
      throw std::invalid_argument("sample_window_size must fit inside the expected image");
    }
    if (min_depth_m_ < 0.0 || max_depth_m_ <= min_depth_m_) {
      throw std::invalid_argument("depth range must satisfy 0 <= min_depth_m < max_depth_m");
    }
    if (fps_smoothing_alpha_ <= 0.0 || fps_smoothing_alpha_ > 1.0) {
      throw std::invalid_argument("fps_smoothing_alpha must be within (0, 1]");
    }
    if (log_throttle_ms_ <= 0) {
      throw std::invalid_argument("log_throttle_ms must be positive");
    }
  }

  void updateFps()
  {
    const SteadyClock::time_point now = SteadyClock::now();
    if (last_frame_time_ != SteadyClock::time_point{}) {
      const double interval_s = std::chrono::duration<double>(now - last_frame_time_).count();
      if (interval_s > 0.0) {
        const double instant_fps = 1.0 / interval_s;
        display_fps_ = display_fps_ <= 0.0 ? instant_fps :
          (1.0 - fps_smoothing_alpha_) * display_fps_ + fps_smoothing_alpha_ * instant_fps;
      }
    }
    last_frame_time_ = now;
  }

  void handleRgbd(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr message)
  {
    updateFps();

    cv_bridge::CvImagePtr color;
    cv_bridge::CvImageConstPtr depth;
    try {
      color = cv_bridge::toCvCopy(message->rgb, sensor_msgs::image_encodings::BGR8);
      depth = cv_bridge::toCvShare(message->depth, message);
    } catch (const cv_bridge::Exception &error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "RGBD cv_bridge conversion failed: %s", error.what());
      return;
    }

    if (!color || color->image.empty() || !depth || depth->image.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "RGBD message contains an empty image");
      return;
    }

    const bool matching_sizes = color->image.size() == depth->image.size();
    const bool expected_size = color->image.cols == expected_width_ &&
      color->image.rows == expected_height_;
    const bool depth_is_uint16 =
      message->depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1 &&
      depth->image.type() == CV_16UC1;
    const bool depth_is_float32 =
      message->depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1 &&
      depth->image.type() == CV_32FC1;
    const bool supported_depth = depth_is_uint16 || depth_is_float32;

    if (!matching_sizes) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "RGB/depth size mismatch: rgb=%dx%d depth=%dx%d",
        color->image.cols, color->image.rows, depth->image.cols, depth->image.rows);
    }
    if (!expected_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "Unexpected RGBD size: received=%dx%d expected=%dx%d",
        color->image.cols, color->image.rows, expected_width_, expected_height_);
    }
    if (!supported_depth) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "Unsupported depth encoding/type: encoding=%s cv_type=%d",
        message->depth.encoding.c_str(), depth->image.type());
    }

    DepthWindowSample sample;
    if (matching_sizes && supported_depth) {
      sample = sampleDepthWindow(
        depth->image, center_x_, center_y_, sample_window_size_, min_depth_m_, max_depth_m_);
    }

    if (debug_view_) {
      drawDebugView(color->image, sample, matching_sizes, expected_size, supported_depth);
    }

    const std::string log_depth = sample.valid ?
      cv::format("%.3fm", sample.depth_m) : "invalid";
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "RGBD fps=%.2f size=%dx%d center=(%d,%d) depth=%s valid=%zu/%zu",
      display_fps_, color->image.cols, color->image.rows, center_x_, center_y_,
      log_depth.c_str(),
      sample.valid_count, sample.total_count);
  }

  void drawDebugView(
    cv::Mat &display,
    const DepthWindowSample &sample,
    bool matching_sizes,
    bool expected_size,
    bool supported_depth)
  {
    const bool center_inside = center_x_ >= 0 && center_x_ < display.cols &&
      center_y_ >= 0 && center_y_ < display.rows;
    const cv::Scalar status_color = sample.valid ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);

    if (center_inside) {
      cv::drawMarker(
        display, cv::Point(center_x_, center_y_), status_color,
        cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
    }
    if (sample.roi.area() > 0) {
      cv::rectangle(display, sample.roi, status_color, 1, cv::LINE_AA);
    }

    const int panel_right = std::min(display.cols - 1, 445);
    const int panel_bottom = std::min(display.rows - 1, 132);
    if (panel_right > 12 && panel_bottom > 12) {
      cv::Mat overlay = display.clone();
      cv::rectangle(
        overlay, cv::Point(12, 12), cv::Point(panel_right, panel_bottom),
        cv::Scalar(25, 25, 25), cv::FILLED);
      cv::addWeighted(overlay, 0.62, display, 0.38, 0.0, display);
    }

    const cv::Scalar normal_text(255, 255, 255);
    const cv::Scalar warning_text(0, 80, 255);
    cv::putText(
      display, cv::format("FPS: %.1f / %.1f", display_fps_, expected_fps_),
      cv::Point(24, 38), cv::FONT_HERSHEY_SIMPLEX, 0.55, normal_text, 1, cv::LINE_AA);
    cv::putText(
      display,
      cv::format(
        "Frame: %dx%d  RGBD: %s", display.cols, display.rows,
        matching_sizes && expected_size ? "OK" : "CHECK"),
      cv::Point(24, 62), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      matching_sizes && expected_size ? normal_text : warning_text, 1, cv::LINE_AA);
    cv::putText(
      display,
      cv::format("Center: (%d,%d)  ROI: %dx%d", center_x_, center_y_,
        sample_window_size_, sample_window_size_),
      cv::Point(24, 86), cv::FONT_HERSHEY_SIMPLEX, 0.55, normal_text, 1, cv::LINE_AA);

    const std::string depth_text = sample.valid ?
      cv::format("Depth: %.3f m  valid=%zu/%zu", sample.depth_m,
        sample.valid_count, sample.total_count) :
      cv::format("Depth: invalid  valid=%zu/%zu", sample.valid_count, sample.total_count);
    cv::putText(
      display, supported_depth ? depth_text : "Depth: unsupported encoding",
      cv::Point(24, 110), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      sample.valid ? normal_text : warning_text, 1, cv::LINE_AA);

    cv::imshow(window_name_, display);
    const int key = cv::waitKey(1) & 0xff;
    if (key == 27 || key == 'q' || key == 'Q') {
      RCLCPP_INFO(get_logger(), "RGBD debug window requested shutdown");
      rclcpp::shutdown();
    }
  }

  std::string rgbd_topic_;
  int expected_width_ = 640;
  int expected_height_ = 480;
  double expected_fps_ = 30.0;
  int center_x_ = 320;
  int center_y_ = 240;
  int sample_window_size_ = 10;
  double min_depth_m_ = 0.1;
  double max_depth_m_ = 10.0;
  double fps_smoothing_alpha_ = 0.1;
  std::string window_name_;
  bool debug_view_ = true;
  int log_throttle_ms_ = 2000;
  bool window_created_ = false;
  SteadyClock::time_point last_frame_time_{};
  double display_fps_ = 0.0;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
};

}  // namespace drone_perception

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_perception::AirGroundServoNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("air_ground_servo_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

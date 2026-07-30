#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "drone_msgs/msg/vision_servo_target.hpp"
#include "drone_perception/air_ground_servo_depth.hpp"
#include "drone_perception/air_ground_target_geometry.hpp"

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
    view_mode_ = declare_parameter<std::string>("view_mode", "RGB");
    opencv_num_threads_ = declare_parameter<int>("opencv_num_threads", 2);
    log_throttle_ms_ = declare_parameter<int>("log_throttle_ms", 2000);
    servo_topic_ = declare_parameter<std::string>("servo_topic", "/vision/servo/target");
    geometry_config_.canny_low_threshold = declare_parameter<int>("canny_low_threshold", 50);
    geometry_config_.canny_high_threshold = declare_parameter<int>("canny_high_threshold", 150);
    geometry_config_.hough_threshold = declare_parameter<int>("hough_threshold", 35);
    geometry_config_.hough_min_line_length = declare_parameter<int>("hough_min_line_length", 45);
    geometry_config_.hough_max_line_gap = declare_parameter<int>("hough_max_line_gap", 12);
    geometry_config_.line_angle_tolerance_deg = declare_parameter<double>("line_angle_tolerance_deg", 20.0);
    geometry_config_.min_cross_angle_deg = declare_parameter<double>("min_cross_angle_deg", 60.0);
    geometry_config_.min_ring_radius_px = declare_parameter<int>("min_ring_radius_px", 20);
    geometry_config_.max_ring_radius_px = declare_parameter<int>("max_ring_radius_px", 220);
    geometry_config_.min_ring_separation_px = declare_parameter<int>("min_ring_separation_px", 15);
    geometry_config_.min_ring_coverage_ratio = declare_parameter<double>("min_ring_coverage_ratio", 0.30);
    trajectory_max_points_ = declare_parameter<int>("trajectory_max_points", 80);

    validateParameters();
    cv::setNumThreads(opencv_num_threads_);
    actual_opencv_num_threads_ = cv::getNumThreads();

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
    servo_pub_ = create_publisher<drone_msgs::msg::VisionServoTarget>(
      servo_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Air-ground RGBD target debug ready: topic=%s expected=%dx%d@%.1f manual=(%d,%d) "
      "window=%dx%d view=%s opencv_threads=%d servo_topic=%s",
      rgbd_topic_.c_str(), expected_width_, expected_height_, expected_fps_, center_x_, center_y_,
      sample_window_size_, sample_window_size_, view_mode_.c_str(), actual_opencv_num_threads_,
      servo_topic_.c_str());
  }

  ~AirGroundServoNode() override
  {
    if (window_created_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  struct ContourDebugInfo
  {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    int largest_index = -1;
    double largest_area = 0.0;
    std::size_t outer_count = 0U;
    std::size_t inner_count = 0U;
  };

  void validateParameters() const
  {
    if (rgbd_topic_.empty()) {
      throw std::invalid_argument("rgbd_topic must not be empty");
    }
    if (servo_topic_.empty()) {
      throw std::invalid_argument("servo_topic must not be empty");
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
    if (view_mode_ != "RGB" && view_mode_ != "GRAY" && view_mode_ != "BINARY") {
      throw std::invalid_argument("view_mode must be RGB, GRAY or BINARY");
    }
    if (opencv_num_threads_ <= 0) {
      throw std::invalid_argument("opencv_num_threads must be positive");
    }
    if (geometry_config_.canny_low_threshold <= 0 ||
      geometry_config_.canny_high_threshold <= geometry_config_.canny_low_threshold ||
      geometry_config_.hough_threshold <= 0 || geometry_config_.hough_min_line_length <= 0 ||
      geometry_config_.hough_max_line_gap < 0 || geometry_config_.line_angle_tolerance_deg <= 0.0 ||
      geometry_config_.line_angle_tolerance_deg >= 45.0 ||
      geometry_config_.min_cross_angle_deg <= 0.0 || geometry_config_.min_cross_angle_deg > 90.0)
    {
      throw std::invalid_argument("target geometry parameters are invalid");
    }
    if (geometry_config_.min_ring_radius_px <= 0 ||
      geometry_config_.max_ring_radius_px <= geometry_config_.min_ring_radius_px ||
      geometry_config_.min_ring_separation_px <= 0 ||
      geometry_config_.min_ring_coverage_ratio <= 0.0 ||
      geometry_config_.min_ring_coverage_ratio > 1.0)
    {
      throw std::invalid_argument("dual-ring parameters are invalid");
    }
    if (trajectory_max_points_ < 2)
    {
      throw std::invalid_argument("trajectory_max_points must be at least 2");
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

  bool acceptMeasurement(TargetGeometryResult *target, const cv::Size &image_size)
  {
    if (!target->valid) {
      return false;
    }
    if (last_measured_center_) {
      const double delta_x = std::abs(target->center.x - last_measured_center_->x);
      const double delta_y = std::abs(target->center.y - last_measured_center_->y);
      if (delta_x > image_size.width * 0.5 || delta_y > image_size.height * 0.5) {
        target->valid = false;
        return false;
      }
    }
    last_measured_center_ = target->center;
    return true;
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
    const auto &camera_info = message->rgb_camera_info;
    const bool camera_info_size = camera_info.width == static_cast<uint32_t>(color->image.cols) &&
      camera_info.height == static_cast<uint32_t>(color->image.rows);
    const bool camera_frame_ready = !camera_info.header.frame_id.empty();

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
    if (!camera_info_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "RGB CameraInfo size mismatch: info=%ux%u rgb=%dx%d",
        camera_info.width, camera_info.height, color->image.cols, color->image.rows);
    }
    if (!camera_frame_ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "RGB CameraInfo frame_id is empty");
    }

    TargetGeometryResult target = detectCrossTarget(color->image, geometry_config_);
    const bool measurement_accepted = acceptMeasurement(&target, color->image.size());
    if (measurement_accepted) {
      measured_trajectory_.emplace_back(
        static_cast<int>(std::lround(target.center.x)), static_cast<int>(std::lround(target.center.y)));
      while (measured_trajectory_.size() > static_cast<std::size_t>(trajectory_max_points_)) {
        measured_trajectory_.pop_front();
      }
    }
    const int sample_x = measurement_accepted ? static_cast<int>(std::lround(target.center.x)) : -1;
    const int sample_y = measurement_accepted ? static_cast<int>(std::lround(target.center.y)) : -1;
    const bool target_center_inside = sample_x >= 0 && sample_x < color->image.cols &&
      sample_y >= 0 && sample_y < color->image.rows;

    DepthWindowSample sample;
    if (target_center_inside && matching_sizes && supported_depth) {
      sample = sampleDepthWindow(
        depth->image, sample_x, sample_y, sample_window_size_, min_depth_m_, max_depth_m_);
    }

    CameraPointSample point;
    if (target_center_inside && matching_sizes && expected_size && supported_depth && camera_info_size &&
      camera_frame_ready && sample.valid)
    {
      point = projectPixelToCamera(
        sample_x, sample_y, sample.depth_m, color->image.cols, color->image.rows, camera_info);
    }

    const bool valid = point.valid;
    drone_msgs::msg::VisionServoTarget output;
    output.valid = valid;
    // 单帧确认：本帧完整通过双环十字、跳变、深度和反投影校验即确认。
    output.confirmed = valid;
    output.error_x = valid ? point.point.x : 0.0;
    output.error_y = valid ? point.point.y : 0.0;
    servo_pub_->publish(output);

    if (debug_view_) {
      cv::Mat display_image = color->image;
      double binary_threshold = -1.0;
      ContourDebugInfo contour_info;
      if (view_mode_ != "RGB") {
        cv::Mat gray_image;
        cv::cvtColor(color->image, gray_image, cv::COLOR_BGR2GRAY);
        cv::Mat processed_image = gray_image;
        if (view_mode_ == "BINARY") {
          binary_threshold = cv::threshold(
            gray_image, processed_image, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
          cv::Mat contour_input;
          cv::bitwise_not(processed_image, contour_input);
          cv::findContours(
            contour_input, contour_info.contours, contour_info.hierarchy, cv::RETR_TREE,
            cv::CHAIN_APPROX_SIMPLE);
          for (std::size_t index = 0; index < contour_info.contours.size(); ++index) {
            const bool is_outer = contour_info.hierarchy[index][3] < 0;
            if (is_outer) {
              ++contour_info.outer_count;
            } else {
              ++contour_info.inner_count;
            }
            const double area = cv::contourArea(contour_info.contours[index]);
            if (is_outer && area > contour_info.largest_area) {
              contour_info.largest_area = area;
              contour_info.largest_index = static_cast<int>(index);
            }
          }
        }
        cv::cvtColor(processed_image, display_image, cv::COLOR_GRAY2BGR);
      }
      drawDebugView(display_image, sample, point, camera_info, matching_sizes, expected_size,
        supported_depth, camera_info_size, binary_threshold, contour_info, target);
    }

    const std::string log_depth = sample.valid ?
      cv::format("%.3fm", sample.depth_m) : "invalid";
    const std::string log_point = point.valid ?
      cv::format("(%.3f,%.3f,%.3f)m", point.point.x, point.point.y, point.point.z) : "invalid";
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "RGBD fps=%.2f size=%dx%d target=%s depth=%s XYZ=%s valid=%zu/%zu",
      display_fps_, color->image.cols, color->image.rows,
      target_center_inside ? cv::format("(%d,%d)", sample_x, sample_y).c_str() : "invalid",
      log_depth.c_str(),
      log_point.c_str(), sample.valid_count, sample.total_count);
  }

  void drawDebugView(
    cv::Mat &display,
    const DepthWindowSample &sample,
    const CameraPointSample &point,
    const sensor_msgs::msg::CameraInfo &camera_info,
    bool matching_sizes,
    bool expected_size,
    bool supported_depth,
    bool camera_info_size,
    double binary_threshold,
    const ContourDebugInfo &contour_info,
    const TargetGeometryResult &target)
  {
    const int target_x = target.valid ? static_cast<int>(std::lround(target.center.x)) : -1;
    const int target_y = target.valid ? static_cast<int>(std::lround(target.center.y)) : -1;
    const bool center_inside = target_x >= 0 && target_x < display.cols &&
      target_y >= 0 && target_y < display.rows;
    const cv::Scalar status_color = sample.valid ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);

    const cv::Point image_center(display.cols / 2, display.rows / 2);
    for (const cv::Point &trajectory_point : measured_trajectory_) {
      cv::circle(display, trajectory_point, 2, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);
    }
    cv::drawMarker(
      display, image_center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);

    if (center_inside) {
      cv::drawMarker(
        display, cv::Point(target_x, target_y), status_color,
        cv::MARKER_CROSS, 24, 2, cv::LINE_AA);
      cv::line(
        display, image_center, cv::Point(target_x, target_y), cv::Scalar(0, 255, 255), 2,
        cv::LINE_AA);
      if (target.valid) {
        cv::line(
          display, cv::Point(target.horizontal_line[0], target.horizontal_line[1]),
          cv::Point(target.horizontal_line[2], target.horizontal_line[3]), cv::Scalar(0, 165, 255),
          2, cv::LINE_AA);
        cv::line(
          display, cv::Point(target.vertical_line[0], target.vertical_line[1]),
          cv::Point(target.vertical_line[2], target.vertical_line[3]), cv::Scalar(0, 165, 255),
          2, cv::LINE_AA);
      }
    }
    if (sample.roi.area() > 0) {
      cv::rectangle(display, sample.roi, status_color, 1, cv::LINE_AA);
    }
    if (view_mode_ == "BINARY") {
      for (std::size_t index = 0; index < contour_info.contours.size(); ++index) {
        const cv::Scalar color = contour_info.hierarchy[index][3] < 0 ?
          cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::drawContours(
          display, contour_info.contours, static_cast<int>(index), color, 1, cv::LINE_AA);
      }
      if (contour_info.largest_index >= 0) {
        cv::drawContours(
          display, contour_info.contours, contour_info.largest_index, cv::Scalar(0, 0, 255), 2,
          cv::LINE_AA);
      }
    }

    const int panel_right = std::min(display.cols - 1, 500);
    const int panel_bottom = std::min(display.rows - 1, 410);
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
        "Frame: %dx%d  View: %s  RGBD: %s", display.cols, display.rows,
        view_mode_.c_str(),
        matching_sizes && expected_size ? "OK" : "CHECK"),
      cv::Point(24, 62), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      matching_sizes && expected_size ? normal_text : warning_text, 1, cv::LINE_AA);
    cv::putText(
      display,
      target.valid ? cv::format("Target: (%d,%d)  ROI: %dx%d", target_x, target_y,
        sample_window_size_, sample_window_size_) : cv::format("Target: invalid  ROI: %dx%d",
        sample_window_size_, sample_window_size_),
      cv::Point(24, 86), cv::FONT_HERSHEY_SIMPLEX, 0.55, normal_text, 1, cv::LINE_AA);
    const int error_x = target_x - image_center.x;
    const int error_y = target_y - image_center.y;
    cv::putText(
      display,
      target.valid ? cv::format("Image center: (%d,%d)  Error: dx=%+d dy=%+d px",
        image_center.x, image_center.y, error_x, error_y) : "Image center: target unavailable",
      cv::Point(24, 110), cv::FONT_HERSHEY_SIMPLEX, 0.48,
      target.valid ? normal_text : warning_text, 1, cv::LINE_AA);

    const std::string depth_text = sample.valid ?
      cv::format("Depth: %.3f m  valid=%zu/%zu", sample.depth_m,
        sample.valid_count, sample.total_count) :
      cv::format("Depth: invalid  valid=%zu/%zu", sample.valid_count, sample.total_count);
    cv::putText(
      display, supported_depth ? depth_text : "Depth: unsupported encoding",
      cv::Point(24, 134), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      sample.valid ? normal_text : warning_text, 1, cv::LINE_AA);
    const cv::Scalar info_color = camera_info_size ? normal_text : warning_text;
    cv::putText(
      display,
      cv::format("CameraInfo: %ux%u %s", camera_info.width, camera_info.height,
        camera_info_size ? "OK" : "CHECK"),
      cv::Point(24, 158), cv::FONT_HERSHEY_SIMPLEX, 0.50, info_color, 1, cv::LINE_AA);
    cv::putText(
      display,
      cv::format("K: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", camera_info.k[0], camera_info.k[4],
        camera_info.k[2], camera_info.k[5]),
      cv::Point(24, 181), cv::FONT_HERSHEY_SIMPLEX, 0.45, info_color, 1, cv::LINE_AA);
    const std::string xyz_text = point.valid ?
      cv::format("XYZ: (%.3f, %.3f, %.3f) m", point.point.x, point.point.y, point.point.z) :
      "XYZ: invalid";
    cv::putText(
      display, xyz_text, cv::Point(24, 204), cv::FONT_HERSHEY_SIMPLEX, 0.50,
      point.valid ? normal_text : warning_text, 1, cv::LINE_AA);
    cv::putText(
      display,
      cv::format("Geometry: cross=%s rings=%s score=%.0f", target.lines.empty() ? "MISS" : "OK",
        target.valid ? "OK" : "MISS", target.score),
      cv::Point(24, 227), cv::FONT_HERSHEY_SIMPLEX, 0.45,
      target.valid ? normal_text : warning_text, 1, cv::LINE_AA);
    cv::putText(
      display, cv::format("Output: %s", target.valid ? "MEASURED" : "INVALID"), cv::Point(24, 250),
      cv::FONT_HERSHEY_SIMPLEX, 0.45, target.valid ? normal_text : warning_text, 1, cv::LINE_AA);
    cv::putText(
      display,
      cv::format("Canny: %d/%d  Hough: %d  Min angle: %.0f deg",
        geometry_config_.canny_low_threshold, geometry_config_.canny_high_threshold,
        geometry_config_.hough_threshold, geometry_config_.min_cross_angle_deg),
      cv::Point(24, 273), cv::FONT_HERSHEY_SIMPLEX, 0.42, normal_text, 1, cv::LINE_AA);
    if (view_mode_ == "BINARY") {
      cv::putText(
        display, cv::format("Otsu threshold: %.1f", binary_threshold), cv::Point(24, 296),
        cv::FONT_HERSHEY_SIMPLEX, 0.42, normal_text, 1, cv::LINE_AA);
      cv::putText(
        display,
        cv::format("Contours: outer=%zu internal=%zu", contour_info.outer_count,
          contour_info.inner_count),
        cv::Point(24, 318), cv::FONT_HERSHEY_SIMPLEX, 0.42, normal_text, 1, cv::LINE_AA);
      cv::putText(
        display, cv::format("Largest outer: %.0f px", contour_info.largest_area),
        cv::Point(24, 340), cv::FONT_HERSHEY_SIMPLEX, 0.42, normal_text, 1, cv::LINE_AA);
    }
    std::string frame_id = camera_info.header.frame_id;
    if (frame_id.size() > 48U) {
      frame_id = frame_id.substr(0, 45U) + "...";
    }
    const int threads_y = view_mode_ == "BINARY" ? 363 : 296;
    const int frame_y = view_mode_ == "BINARY" ? 385 : 319;
    cv::putText(
      display, cv::format("OpenCV threads: %d", actual_opencv_num_threads_),
      cv::Point(24, threads_y), cv::FONT_HERSHEY_SIMPLEX, 0.42, normal_text, 1, cv::LINE_AA);
    cv::putText(
      display, cv::format("Frame: %s", frame_id.c_str()), cv::Point(24, frame_y),
      cv::FONT_HERSHEY_SIMPLEX, 0.42, normal_text, 1, cv::LINE_AA);

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
  std::string view_mode_ = "RGB";
  int opencv_num_threads_ = 2;
  int actual_opencv_num_threads_ = 0;
  int log_throttle_ms_ = 2000;
  std::string servo_topic_;
  TargetGeometryConfig geometry_config_;
  int trajectory_max_points_ = 80;
  bool window_created_ = false;
  SteadyClock::time_point last_frame_time_{};
  double display_fps_ = 0.0;
  std::optional<cv::Point2f> last_measured_center_;
  std::deque<cv::Point> measured_trajectory_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
  rclcpp::Publisher<drone_msgs::msg::VisionServoTarget>::SharedPtr servo_pub_;
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

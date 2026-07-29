#include "drone_line_vision/line_vision_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>

namespace drone_line_vision
{
namespace
{
template<typename T>
T value(const YAML::Node & node, const char * key, const T & fallback)
{
  return node[key] ? node[key].as<T>() : fallback;
}

YAML::Node parametersNode(const YAML::Node & root)
{
  if (root["line_vision_node"] && root["line_vision_node"]["ros__parameters"]) {
    return root["line_vision_node"]["ros__parameters"];
  }
  return root;
}

char keyChar(const YAML::Node & node, const char * key, char fallback)
{
  const std::string text = value<std::string>(node, key, std::string(1, fallback));
  return text.empty() ? fallback : text.front();
}

std::string finiteText(double value)
{
  return std::isfinite(value) ? std::to_string(value) : "NaN";
}
}

LineVisionNode::LineVisionNode()
: Node("line_vision_node")
{
  publisher_ = create_publisher<drone_msgs::msg::LinePixelObservation>(
    "/line_vision/pixel_observation", rclcpp::QoS(10));
  declare_parameter<std::string>("config_file", "");
  std::string path = get_parameter("config_file").as_string();
  if (path.empty()) {
    path = ament_index_cpp::get_package_share_directory("drone_line_vision") + "/config/line_vision.yaml";
  }
  std::string error;
  if (!loadConfig(path, config_, error)) {
    throw std::runtime_error("Unable to load line vision config: " + error);
  }
  config_.config_file = path;
#ifdef DRONE_LINE_VISION_HAS_HBM
  image_subscription_ = create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
    config_.camera.source_topic, rclcpp::QoS(1).best_effort(),
    std::bind(&LineVisionNode::imageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "line vision ready: subscribing to %s (hobot_codec HBM NV12)",
    config_.camera.source_topic.c_str());
#else
  throw std::runtime_error("hbm_img_msgs is unavailable; build against the RDK TROS environment");
#endif
}

LineVisionNode::~LineVisionNode()
{
  running_.store(false);
  if (config_.display.enabled) {cv::destroyAllWindows();}
}

bool LineVisionNode::loadConfig(const std::string & path, LineVisionConfig & out, std::string & error) const
{
  try {
    const YAML::Node p = parametersNode(YAML::LoadFile(path));
    LineVisionConfig c;
    const auto camera = p["camera"];
    c.camera.source_topic = value<std::string>(camera, "source_topic", c.camera.source_topic);
    c.camera.device = value<std::string>(camera, "device", c.camera.device);
    c.camera.width = value<int>(camera, "width", c.camera.width);
    c.camera.height = value<int>(camera, "height", c.camera.height);
    c.camera.fps = value<int>(camera, "fps", c.camera.fps);
    c.camera.pixel_format = value<std::string>(camera, "pixel_format", c.camera.pixel_format);
    c.camera.io_method = value<std::string>(camera, "io_method", c.camera.io_method);
    c.camera.zero_copy = value<bool>(camera, "zero_copy", c.camera.zero_copy);
    c.camera.source_encoding = value<std::string>(camera, "source_encoding", c.camera.source_encoding);
    const auto roi = p["roi"];
    c.roi.enabled = value<bool>(roi, "enabled", c.roi.enabled);
    c.roi.x = value<int>(roi, "x", c.roi.x); c.roi.y = value<int>(roi, "y", c.roi.y);
    c.roi.width = value<int>(roi, "width", c.roi.width); c.roi.height = value<int>(roi, "height", c.roi.height);
    const auto threshold = p["threshold"];
    c.threshold.mode = value<std::string>(threshold, "mode", c.threshold.mode);
    c.threshold.gray_threshold = value<int>(threshold, "gray_threshold", c.threshold.gray_threshold);
    c.threshold.invert = value<bool>(threshold, "invert", c.threshold.invert);
    c.threshold.min_candidate_pixels = value<int>(threshold, "min_candidate_pixels", c.threshold.min_candidate_pixels);
    c.threshold.max_candidate_pixels = value<int>(threshold, "max_candidate_pixels", c.threshold.max_candidate_pixels);
    const auto morphology = p["morphology"];
    c.morphology.enabled = value<bool>(morphology, "enabled", c.morphology.enabled);
    c.morphology.kernel_size = value<int>(morphology, "kernel_size", c.morphology.kernel_size);
    c.morphology.open_iterations = value<int>(morphology, "open_iterations", c.morphology.open_iterations);
    c.morphology.close_iterations = value<int>(morphology, "close_iterations", c.morphology.close_iterations);
    const auto fit = p["line_fit"];
    c.line_fit.min_component_area = value<int>(fit, "min_component_area", c.line_fit.min_component_area);
    c.line_fit.max_fit_residual_px = value<double>(fit, "max_fit_residual_px", c.line_fit.max_fit_residual_px);
    c.line_fit.min_line_length_px = value<double>(fit, "min_line_length_px", c.line_fit.min_line_length_px);
    c.line_fit.near_scan_ratio = value<double>(fit, "near_scan_ratio", c.line_fit.near_scan_ratio);
    c.line_fit.far_scan_ratio = value<double>(fit, "far_scan_ratio", c.line_fit.far_scan_ratio);
    const auto display_cfg = p["display"];
    c.display.enabled = value<bool>(display_cfg, "enabled", c.display.enabled);
    c.display.window_width = value<int>(display_cfg, "window_width", c.display.window_width);
    c.display.window_height = value<int>(display_cfg, "window_height", c.display.window_height);
    c.display.display_fps_limit = value<int>(display_cfg, "display_fps_limit", c.display.display_fps_limit);
    c.display.show_debug_text = value<bool>(display_cfg, "show_debug_text", c.display.show_debug_text);
    c.display.show_binary_mask = value<bool>(display_cfg, "show_binary_mask", c.display.show_binary_mask);
    c.display.show_data_panel = value<bool>(display_cfg, "show_data_panel", c.display.show_data_panel);
    const auto logging = p["logging"];
    c.logging.enabled = value<bool>(logging, "enabled", c.logging.enabled);
    c.logging.output_directory = value<std::string>(logging, "output_directory", c.logging.output_directory);
    c.logging.csv_enabled = value<bool>(logging, "csv_enabled", c.logging.csv_enabled);
    c.logging.json_enabled = value<bool>(logging, "json_enabled", c.logging.json_enabled);
    c.logging.save_images_on_invalid = value<bool>(logging, "save_images_on_invalid", c.logging.save_images_on_invalid);
    const auto runtime = p["runtime"];
    c.runtime.reload_key = keyChar(runtime, "reload_key", c.runtime.reload_key);
    c.runtime.save_key = keyChar(runtime, "save_key", c.runtime.save_key);
    c.runtime.pause_key = keyChar(runtime, "pause_key", c.runtime.pause_key);
    c.runtime.quit_key = keyChar(runtime, "quit_key", c.runtime.quit_key);
    if (c.camera.width <= 0 || c.camera.height <= 0 || c.camera.fps <= 0) {
      throw std::runtime_error("camera dimensions and fps must be positive");
    }
    if (c.camera.source_topic.empty() || c.camera.source_encoding != "nv12") {
      throw std::runtime_error("source_topic must be set and source_encoding must be nv12");
    }
    if (c.roi.enabled && (c.roi.x < 0 || c.roi.y < 0 || c.roi.width <= 0 || c.roi.height <= 0 ||
      c.roi.x + c.roi.width > c.camera.width || c.roi.y + c.roi.height > c.camera.height)) {
      throw std::runtime_error("ROI is outside the configured image bounds");
    }
    out = c;
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

#ifdef DRONE_LINE_VISION_HAS_HBM
void LineVisionNode::imageCallback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr message)
{
  if (!message || message->width == 0 || message->height == 0 ||
    static_cast<int>(message->width) != config_.camera.width ||
    static_cast<int>(message->height) != config_.camera.height || message->step < message->width ||
    message->data_size < message->step * message->height || message->data.size() < message->data_size) {
    lost_frames_.fetch_add(1); return;
  }
  const std::string encoding(reinterpret_cast<const char *>(message->encoding.data()), message->encoding.size());
  if (encoding.find("nv12") == std::string::npos && encoding.find("NV12") == std::string::npos) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported HBM encoding: %s", encoding.c_str());
    lost_frames_.fetch_add(1); return;
  }
  const auto previous = last_frame_time_;
  const auto current = std::chrono::steady_clock::now();
  if (previous.time_since_epoch().count() != 0) {
    const double dt = std::chrono::duration<double>(current - previous).count();
    if (dt > 0.0) {stats_.addCapture(1.0 / dt);}
  }
  last_frame_time_ = current;
  cv::Mat y(static_cast<int>(message->height), static_cast<int>(message->width), CV_8UC1,
    message->data.data(), static_cast<size_t>(message->step));
  processFrame(y, rclcpp::Time(message->time_stamp), frame_id_.fetch_add(1));
}
#endif

void LineVisionNode::processFrame(const cv::Mat & y, const rclcpp::Time & stamp, uint64_t frame_id)
{
  const auto start = std::chrono::steady_clock::now();
  LineResult result = detectLine(y, config_);
  const double processing_us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - start).count();
  static auto previous_observation = std::chrono::steady_clock::now();
  const auto observation_now = std::chrono::steady_clock::now();
  const double observation_interval = std::chrono::duration<double>(observation_now - previous_observation).count();
  previous_observation = observation_now;
  stats_.addObservation(processing_us, observation_interval);
  if (!result.valid) {lost_frames_.fetch_add(1);} else {lost_frames_.store(0);}
  if (result.valid) {result.center_u += 0.0;}
  display(y, result.mask, result, processing_us, true);
  drone_msgs::msg::LinePixelObservation message;
  message.header.stamp = stamp; message.header.frame_id = "camera";
  message.valid = result.valid; message.line_center_u_px = result.valid ? static_cast<float>(result.center_u) : NAN;
  message.line_center_v_px = result.valid ? static_cast<float>(result.center_v) : NAN;
  message.image_center_u_px = static_cast<float>(y.cols / 2.0);
  message.error_u_px = result.valid ? static_cast<float>(result.center_u - y.cols / 2.0) : NAN;
  message.line_angle_rad = result.valid ? static_cast<float>(result.angle_rad) : NAN;
  message.confidence = static_cast<float>(result.valid ? result.confidence : 0.0);
  message.candidate_pixel_count = static_cast<uint32_t>(std::max(0, result.candidate_pixels));
  message.lost_frame_count = lost_frames_.load(); message.processing_time_us = static_cast<uint32_t>(processing_us);
  message.capture_fps = static_cast<float>(stats_.captureFps()); message.observation_fps = static_cast<float>(stats_.observationFps());
  message.decode_ok = true; message.roi_valid = !config_.roi.enabled || result.roi.area() > 0;
  publisher_->publish(message);
  if (config_.logging.enabled && config_.logging.csv_enabled) {
    std::filesystem::create_directories(config_.logging.output_directory);
    const auto path = std::filesystem::path(config_.logging.output_directory) / "observations.csv";
    const bool exists = std::filesystem::exists(path);
    std::ofstream csv(path, std::ios::app);
    if (!exists) {csv << "frame_id,valid,line_center_u_px,line_center_v_px,error_u_px,line_angle_rad,confidence,candidate_pixel_count,processing_time_us\n";}
    csv << frame_id << ',' << (result.valid ? 1 : 0) << ',' << finiteText(result.center_u) << ',' << finiteText(result.center_v) << ','
      << finiteText(result.valid ? result.center_u - y.cols / 2.0 : NAN) << ',' << finiteText(result.angle_rad) << ','
      << result.confidence << ',' << result.candidate_pixels << ',' << processing_us << '\n';
  }
}

void LineVisionNode::display(const cv::Mat & y, const cv::Mat & mask, const LineResult & result,
  double processing_us, bool decode_ok)
{
  if (!config_.display.enabled) {return;}
  const auto now = std::chrono::steady_clock::now();
  const double min_period = 1.0 / std::max(1, config_.display.display_fps_limit);
  if (last_display_time_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double>(now - last_display_time_).count() < min_period) {
    const int key = cv::waitKey(1) & 0xff;
    if (key == config_.runtime.quit_key || key == 27) {running_.store(false); rclcpp::shutdown();}
    return;
  }
  last_display_time_ = now;
  cv::Mat debug = y.clone();
  cv::line(debug, cv::Point(debug.cols / 2, 0), cv::Point(debug.cols / 2, debug.rows), cv::Scalar(220), 1);
  if (result.valid) {
    cv::circle(debug, cv::Point(static_cast<int>(result.center_u), static_cast<int>(result.center_v)), 7, cv::Scalar(200), -1);
    cv::line(debug, cv::Point(static_cast<int>(result.center_u), static_cast<int>(result.center_v)),
      cv::Point(static_cast<int>(result.center_u + 100 * std::cos(result.angle_rad)), static_cast<int>(result.center_v + 100 * std::sin(result.angle_rad))), cv::Scalar(150), 2);
  }
  cv::Mat combined = debug;
  if (config_.display.show_data_panel) {
    const int panel_width = std::min(520, combined.cols - 20);
    const int panel_height = std::min(360, combined.rows - 20);
    cv::Rect panel_rect(10, 10, panel_width, panel_height);
    cv::Mat panel = combined(panel_rect).clone();
    panel.setTo(cv::Scalar(25));
    auto text = [&](const std::string & value_text, int row) {
      cv::putText(panel, value_text, cv::Point(12, row), cv::FONT_HERSHEY_SIMPLEX, 0.48,
        cv::Scalar(235), 1, cv::LINE_AA);
    };
    text("Y Plane | FULL_IMAGE | NV12", 24);
    text("size: " + std::to_string(y.cols) + "x" + std::to_string(y.rows) + "  decode: " + std::to_string(decode_ok), 48);
    text("valid: " + std::to_string(result.valid) + "  confidence: " + std::to_string(result.confidence), 72);
    text("center: " + finiteText(result.valid ? result.center_u : NAN) + ", " + finiteText(result.valid ? result.center_v : NAN), 96);
    text("error_u_px: " + finiteText(result.valid ? result.center_u - y.cols / 2.0 : NAN), 120);
    text("angle_rad: " + finiteText(result.valid ? result.angle_rad : NAN), 144);
    text("candidates: " + std::to_string(result.candidate_pixels) + "  lost: " + std::to_string(lost_frames_.load()), 168);
    text("capture_fps: " + std::to_string(stats_.captureFps()), 192);
    text("observation_fps: " + std::to_string(stats_.observationFps()), 216);
    text("processing_us: " + std::to_string(processing_us), 240);
    text("p50/p95_us: " + std::to_string(stats_.p50Us()) + "/" + std::to_string(stats_.p95Us()), 264);
    text("threshold: " + std::to_string(config_.threshold.gray_threshold), 288);
    text("topic: " + config_.camera.source_topic, 312);
    cv::addWeighted(combined(panel_rect), 0.35, panel, 0.65, 0.0, combined(panel_rect));
  }
  if (config_.display.show_binary_mask && !mask.empty()) {
    const int inset_width = std::max(160, combined.cols / 5);
    const int inset_height = std::max(90, combined.rows / 5);
    cv::Mat inset;
    cv::resize(mask, inset, cv::Size(inset_width, inset_height));
    cv::Rect inset_rect(combined.cols - inset.cols - 12, combined.rows - inset.rows - 12, inset.cols, inset.rows);
    cv::addWeighted(combined(inset_rect), 0.35, inset, 0.65, 0.0, combined(inset_rect));
  }
  static bool window_created = false;
  if (!window_created) {
    cv::namedWindow("Line Vision Y Plane", cv::WINDOW_NORMAL);
    cv::resizeWindow("Line Vision Y Plane", config_.display.window_width, config_.display.window_height);
    window_created = true;
  }
  cv::imshow("Line Vision Y Plane", combined);
  const int key = cv::waitKey(1) & 0xff;
  if (key == config_.runtime.quit_key || key == 27) {running_.store(false); rclcpp::shutdown();}
  else if (key == config_.runtime.pause_key) {cv::waitKey(0);}
  else if (key == config_.runtime.reload_key) {
    LineVisionConfig reloaded;
    std::string error;
    if (loadConfig(config_.config_file, reloaded, error)) {
      reloaded.config_file = config_.config_file;
      reloaded.camera = config_.camera;
      config_ = reloaded;
      RCLCPP_INFO(get_logger(), "Reloaded algorithm/display parameters from %s", config_.config_file.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "YAML reload failed: %s", error.c_str());
    }
  } else if (key == config_.runtime.save_key) {
    saveCurrent(y, mask, debug, result, processing_us);
  }
}

void LineVisionNode::saveCurrent(const cv::Mat & y, const cv::Mat & mask, const cv::Mat & debug,
  const LineResult & result, double processing_us)
{
  const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const std::filesystem::path directory = std::filesystem::path(config_.logging.output_directory) /
    ("snapshot_" + std::to_string(stamp));
  std::filesystem::create_directories(directory);
  cv::imwrite((directory / "y_plane.pgm").string(), y);
  cv::imwrite((directory / "binary_mask.pgm").string(), mask);
  cv::imwrite((directory / "debug.pgm").string(), debug);
  std::ofstream json(directory / "observation.json");
  json << std::setprecision(10) << "{\n"
       << "  \"valid\": " << (result.valid ? "true" : "false") << ",\n"
       << "  \"line_center_u_px\": " << finiteText(result.valid ? result.center_u : NAN) << ",\n"
       << "  \"line_center_v_px\": " << finiteText(result.valid ? result.center_v : NAN) << ",\n"
       << "  \"error_u_px\": " << finiteText(result.valid ? result.center_u - y.cols / 2.0 : NAN) << ",\n"
       << "  \"line_angle_rad\": " << finiteText(result.valid ? result.angle_rad : NAN) << ",\n"
       << "  \"confidence\": " << result.confidence << ",\n"
       << "  \"candidate_pixel_count\": " << result.candidate_pixels << ",\n"
       << "  \"processing_time_us\": " << processing_us << "\n}\n";
  RCLCPP_INFO(get_logger(), "Saved snapshot to %s", directory.c_str());
}

}

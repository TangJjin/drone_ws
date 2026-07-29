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
#include <sstream>
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
  startPipeline();
  running_.store(true);
  capture_thread_ = std::thread(&LineVisionNode::captureLoop, this);
  RCLCPP_INFO(get_logger(), "line vision ready: %s", path.c_str());
}

LineVisionNode::~LineVisionNode()
{
  running_.store(false);
  stopPipeline();
  if (capture_thread_.joinable()) {capture_thread_.join();}
  if (config_.display.enabled) {cv::destroyAllWindows();}
}

bool LineVisionNode::loadConfig(const std::string & path, LineVisionConfig & out, std::string & error) const
{
  try {
    const YAML::Node p = parametersNode(YAML::LoadFile(path));
    LineVisionConfig c;
    const auto camera = p["camera"];
    c.camera.device = value<std::string>(camera, "device", c.camera.device);
    c.camera.width = value<int>(camera, "width", c.camera.width);
    c.camera.height = value<int>(camera, "height", c.camera.height);
    c.camera.fps = value<int>(camera, "fps", c.camera.fps);
    c.camera.input_format = value<std::string>(camera, "input_format", c.camera.input_format);
    c.camera.decoder = value<std::string>(camera, "decoder", c.camera.decoder);
    c.camera.decoder_output = value<std::string>(camera, "decoder_output", c.camera.decoder_output);
    c.camera.queue_size = value<int>(camera, "queue_size", c.camera.queue_size);
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
    if (c.camera.width <= 0 || c.camera.height <= 0 || c.camera.fps <= 0 || c.camera.queue_size <= 0) {
      throw std::runtime_error("camera dimensions, fps and queue_size must be positive");
    }
    if (c.camera.input_format != "MJPG" || c.camera.decoder != "mppjpegdec" || c.camera.decoder_output != "NV12") {
      throw std::runtime_error("only MJPG -> mppjpegdec -> NV12 is supported");
    }
    out = c;
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

void LineVisionNode::startPipeline()
{
  gst_init(nullptr, nullptr);
  if (gst_element_factory_find("mppjpegdec") == nullptr) {
    throw std::runtime_error("mppjpegdec GStreamer plugin is unavailable; refusing software decode");
  }
  std::ostringstream pipeline;
  pipeline << "v4l2src device=" << config_.camera.device << " io-mode=2 do-timestamp=true ! "
           << "image/jpeg,width=" << config_.camera.width << ",height=" << config_.camera.height
           << ",framerate=" << config_.camera.fps << "/1 ! queue max-size-buffers="
           << config_.camera.queue_size << " max-size-bytes=0 max-size-time=0 leaky=downstream ! "
           << "jpegparse ! mppjpegdec format=NV12 dma-feature=false ! video/x-raw,format=NV12 ! "
           << "appsink name=line_sink emit-signals=false sync=false max-buffers=1 drop=true";
  pipeline_description_ = pipeline.str();
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
  if (pipeline_ == nullptr || error != nullptr) {
    const std::string message = error == nullptr ? "unknown" : error->message;
    if (error != nullptr) {g_error_free(error);}
    throw std::runtime_error("GStreamer pipeline parse failed: " + message);
  }
  app_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "line_sink");
  if (app_sink_ == nullptr || !GST_IS_APP_SINK(app_sink_)) {throw std::runtime_error("appsink unavailable");}
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error("GStreamer pipeline failed to enter PLAYING");
  }
  GstSample * first = gst_app_sink_try_pull_sample(GST_APP_SINK(app_sink_), 2 * GST_SECOND);
  if (first == nullptr) {throw std::runtime_error("no first camera sample");}
  GstVideoInfo info;
  const bool valid = sampleInfo(first, info);
  gst_sample_unref(first);
  if (!valid) {throw std::runtime_error("first sample is not NV12 with expected dimensions");}
  RCLCPP_INFO(get_logger(), "MPP pipeline ready: %s", pipeline_description_.c_str());
}

bool LineVisionNode::sampleInfo(GstSample * sample, GstVideoInfo & info) const
{
  GstCaps * caps = gst_sample_get_caps(sample);
  if (caps == nullptr || !gst_video_info_from_caps(&info, caps)) {return false;}
  return GST_VIDEO_INFO_FORMAT(&info) == GST_VIDEO_FORMAT_NV12 &&
    static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)) == config_.camera.width &&
    static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info)) == config_.camera.height &&
    GST_VIDEO_INFO_N_PLANES(&info) >= 2;
}

void LineVisionNode::captureLoop()
{
  auto previous = std::chrono::steady_clock::now();
  while (running_.load() && rclcpp::ok()) {
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(app_sink_), 100 * GST_MSECOND);
    if (sample == nullptr) {lost_frames_.fetch_add(1); continue;}
    GstVideoInfo info;
    if (!sampleInfo(sample, info)) {gst_sample_unref(sample); lost_frames_.fetch_add(1); continue;}
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count(); previous = now;
    if (dt > 0.0) {stats_.addCapture(1.0 / dt);}
    processSample(sample, info, frame_id_.fetch_add(1));
    gst_sample_unref(sample);
  }
}

void LineVisionNode::processSample(GstSample * sample, const GstVideoInfo & info, uint64_t frame_id)
{
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstVideoFrame frame;
  if (buffer == nullptr || !gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
    lost_frames_.fetch_add(1); return;
  }
  const auto start = std::chrono::steady_clock::now();
  auto * data = GST_VIDEO_FRAME_PLANE_DATA(&frame, 0);
  const auto stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
  cv::Mat y(static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info)), static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)),
    CV_8UC1, data, static_cast<size_t>(stride));
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
  message.header.stamp = now(); message.header.frame_id = "camera";
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
  gst_video_frame_unmap(&frame);
}

void LineVisionNode::display(const cv::Mat & y, const cv::Mat & mask, const LineResult & result,
  double processing_us, bool decode_ok)
{
  if (!config_.display.enabled) {return;}
  cv::Mat debug = y.clone();
  cv::line(debug, cv::Point(debug.cols / 2, 0), cv::Point(debug.cols / 2, debug.rows), cv::Scalar(220), 1);
  if (result.valid) {
    cv::circle(debug, cv::Point(static_cast<int>(result.center_u), static_cast<int>(result.center_v)), 7, cv::Scalar(200), -1);
    cv::line(debug, cv::Point(static_cast<int>(result.center_u), static_cast<int>(result.center_v)),
      cv::Point(static_cast<int>(result.center_u + 100 * std::cos(result.angle_rad)), static_cast<int>(result.center_v + 100 * std::sin(result.angle_rad))), cv::Scalar(150), 2);
  }
  const int panel_width = 430;
  cv::Mat panel(y.rows, panel_width, CV_8UC1, cv::Scalar(20));
  auto text = [&](const std::string & value_text, int row) {cv::putText(panel, value_text, cv::Point(10, row), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230), 1, cv::LINE_AA);};
  if (config_.display.show_data_panel) {
  text("Y Plane / FULL_IMAGE", 25); text("device: " + config_.camera.device, 52);
  text("requested: " + std::to_string(config_.camera.width) + "x" + std::to_string(config_.camera.height), 79);
  text("actual: " + std::to_string(y.cols) + "x" + std::to_string(y.rows), 106);
  text("input: MJPG  decoder: mppjpegdec", 133); text("output: NV12  input: Y_PLANE", 160);
  text("decode_ok: " + std::to_string(decode_ok), 187);
  text("valid: " + std::to_string(result.valid), 214); text("center_u_px: " + finiteText(result.valid ? result.center_u : NAN), 241);
  text("center_v_px: " + finiteText(result.valid ? result.center_v : NAN), 268); text("error_u_px: " + finiteText(result.valid ? result.center_u - y.cols / 2.0 : NAN), 295);
  text("angle_rad: " + finiteText(result.valid ? result.angle_rad : NAN), 322); text("confidence: " + std::to_string(result.confidence), 349);
  text("candidate_pixels: " + std::to_string(result.candidate_pixels), 376); text("lost_frames: " + std::to_string(lost_frames_.load()), 403);
  text("capture_fps: " + std::to_string(stats_.captureFps()), 430); text("observation_fps: " + std::to_string(stats_.observationFps()), 457);
  text("processing_us: " + std::to_string(processing_us), 484); text("p50_us: " + std::to_string(stats_.p50Us()), 511);
  text("p95_us: " + std::to_string(stats_.p95Us()), 538); text("config: " + config_.config_file, 565);
  text("threshold: " + std::to_string(config_.threshold.gray_threshold), 592);
  text("roi_valid: " + std::to_string(!config_.roi.enabled || result.roi.area() > 0), 619);
  }
  cv::Mat left; cv::resize(debug, left, cv::Size(640, 360)); cv::Mat right;
  if (config_.display.show_binary_mask) {cv::resize(mask, right, cv::Size(320, 360));}
  else {right = cv::Mat::zeros(360, 320, CV_8UC1);}
  cv::Mat combined;
  if (config_.display.show_data_panel) {
    cv::Mat panel_small; cv::resize(panel, panel_small, cv::Size(430, 360));
    cv::hconcat(std::vector<cv::Mat>{left, right, panel_small}, combined);
  } else {
    cv::hconcat(left, right, combined);
  }
  static bool window_created = false;
  if (!window_created) {
    cv::namedWindow("Line Vision Y Plane", cv::WINDOW_NORMAL);
    cv::resizeWindow("Line Vision Y Plane", config_.display.window_width, config_.display.window_height);
    window_created = true;
  }
  cv::imshow("Line Vision Y Plane", combined);
  const int delay_ms = std::max(1, 1000 / std::max(1, config_.display.display_fps_limit));
  const int key = cv::waitKey(delay_ms) & 0xff;
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

void LineVisionNode::stopPipeline()
{
  if (pipeline_ != nullptr) {gst_element_set_state(pipeline_, GST_STATE_NULL); gst_object_unref(pipeline_); pipeline_ = nullptr; app_sink_ = nullptr;}
}
}

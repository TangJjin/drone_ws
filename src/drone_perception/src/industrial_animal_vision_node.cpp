#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <linux/videodev2.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drone_msgs/msg/industrial_camera_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "drone_msgs/msg/vision_servo_status.hpp"
#include "drone_msgs/msg/vision_servo_target.hpp"
#include "drone_perception/industrial_animal_vision_node.hpp"
#include "drone_perception/rknn_yolo_detector.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::size_t kWorkerCount = 3;
constexpr std::size_t kTaskQueueCapacity = 3;
// Long enough that normal inference jitter never trips it (frames arrive every
// ~11 ms at 90 fps) and short enough to stay under servo_stale_timeout_s, so a
// single abandoned frame never reaches the control side as a lost target.
constexpr int kDetectionFrameWaitMs = 100;

struct GstSampleDeleter
{
  void operator()(GstSample *sample) const
  {
    if (sample != nullptr) {
      gst_sample_unref(sample);
    }
  }
};

using SamplePtr = std::shared_ptr<GstSample>;

double elapsedSeconds(const Clock::time_point &start, const Clock::time_point &end)
{
  return std::chrono::duration<double>(end - start).count();
}

class IndustrialAnimalVisionNode : public rclcpp::Node
{
public:
  IndustrialAnimalVisionNode()
  : Node("industrial_animal_vision"), started_at_(Clock::now()), last_report_at_(started_at_)
  {
    declareParameters();
    readParameters();
    validateParameters();
    configureProcessAffinity();
    configureCameraControls();
    startPipeline();
    initializeDetectors();
    servo_target_pub_ = create_publisher<drone_msgs::msg::VisionServoTarget>(
      servo_target_topic_, rclcpp::SensorDataQoS());
    // QoS must match the control side publisher exactly: reliable + transient_local
    // depth 10, so the latched status survives when this node starts late.
    servo_status_sub_ = create_subscription<drone_msgs::msg::VisionServoStatus>(
      servo_status_topic_, rclcpp::QoS(10).reliable().transient_local(),
      [this](const drone_msgs::msg::VisionServoStatus::SharedPtr message) {
        handleServoStatus(message);
      });
    camera_params_sub_ = create_subscription<drone_msgs::msg::IndustrialCameraParams>(
      "/industrial_camera/params", rclcpp::QoS(1).reliable().transient_local(),
      [this](const drone_msgs::msg::IndustrialCameraParams::SharedPtr message) {
        handleCameraParams(message);
      });
    if (ground_projection_enabled_) {
      local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        local_pose_topic_, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
          handleLocalPose(message);
        });
    }

    detection_publish_thread_ = std::thread(&IndustrialAnimalVisionNode::detectionPublishLoop, this);
    servo_publish_thread_ = std::thread(&IndustrialAnimalVisionNode::servoPublishLoop, this);
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      workers_[index] = std::thread(&IndustrialAnimalVisionNode::workerLoop, this, index);
    }
    capture_thread_ = std::thread(&IndustrialAnimalVisionNode::captureLoop, this);
    if (display_enabled_) {
      ui_thread_ = std::thread(&IndustrialAnimalVisionNode::uiLoop, this);
    }

    RCLCPP_INFO(
      get_logger(),
      "Industrial animal vision ready: profile=%s model=%s camera=%s "
      "request=MJPEG %dx%d@%d display=%s preprocess=%s servo_target_topic=%s rate=%.1fHz",
      camera_profile_.c_str(), model_path_.c_str(), camera_device_.c_str(),
      camera_width_, camera_height_, camera_fps_, display_enabled_ ? "on" : "off",
      preprocess_mode_.c_str(), servo_target_topic_.c_str(), servo_publish_rate_hz_);
  }

  ~IndustrialAnimalVisionNode() override
  {
    running_.store(false);
    // Acquire and release each mutex before notifying: a waiter that evaluated
    // its predicate just before the store above has not blocked yet and would
    // miss a lock-free notification forever (detectionPublishLoop and workerLoop
    // wait without a timeout).
    {
      std::lock_guard<std::mutex> lock(task_mutex_);
    }
    task_ready_.notify_all();
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
    }
    result_ready_.notify_all();
    {
      std::lock_guard<std::mutex> lock(detection_result_mutex_);
    }
    detection_result_ready_.notify_all();
    {
      std::lock_guard<std::mutex> lock(servo_wake_mutex_);
    }
    servo_wake_.notify_all();

    if (pipeline_ != nullptr) {
      gst_element_send_event(pipeline_, gst_event_new_eos());
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    for (std::thread &worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    if (detection_publish_thread_.joinable()) {
      detection_publish_thread_.join();
    }
    if (servo_publish_thread_.joinable()) {
      servo_publish_thread_.join();
    }
    if (ui_thread_.joinable()) {
      ui_thread_.join();
    }
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (app_sink_ != nullptr) {
      gst_object_unref(app_sink_);
      app_sink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

private:
  struct FrameTask
  {
    std::uint64_t frame_id = 0;
    SamplePtr sample;
  };

  struct InferenceResult
  {
    std::uint64_t frame_id = 0;
    std::size_t worker_index = 0;
    SamplePtr sample;
    std::vector<Detection> detections;
    RknnYoloDetector::InferenceTimingStats timing;
    double api_run_ms = 0.0;
  };

  struct DetectionFrame
  {
    std::uint64_t frame_id = 0;
    int image_width = 0;
    int image_height = 0;
    // Stamped by the worker when inference finished, so track ageing measures
    // observation time rather than the delay before this loop drains the frame.
    Clock::time_point produced_at{};
    std::vector<Detection> detections;
    std::vector<std::string> labels;
  };

  struct TrackedDetection
  {
    std::uint64_t track_id = 0;
    int class_id = -1;
    cv::Rect box;
    std::uint64_t last_seen_frame_id = 0;
    // Wall-clock rather than a frame count: frame ids advance at the capture
    // rate, which was measured drifting between 127 and 370 fps, so a fixed
    // frame budget maps to a wildly varying amount of real time.
    Clock::time_point last_seen_at{};
    std::string label;
    // Streak of consecutive stable hits driving the confirmed flag. A missed frame
    // resets the streak but keeps confirmed: a target reappearing inside the miss
    // window must stay confirmed, otherwise the control-side settle timer restarts
    // on every dropped detection.
    std::uint32_t hit_streak = 0;
    bool confirmed = false;
    cv::Point last_center;
    int last_area = 0;
    float last_score = 0.0F;
  };

  struct TrackMatch
  {
    std::size_t detection_index = 0;
    std::size_t track_index = 0;
    float iou = 0.0F;
  };

  // Value-semantic handover from the tracking thread to the servo publish thread.
  // Guarded by servo_mutex_.
  struct ServoSnapshot
  {
    bool source_valid = false;  // at least one inference frame has been produced
    bool has_target = false;
    bool confirmed = false;
    // Kept filled even when the target is missing so the string stays byte-stable
    // for the whole servo action (the control side locks the first fresh id).
    std::string target_id;
    std::string label;
    double confidence = 0.0;
    double error_x = 0.0;
    double error_y = 0.0;
    double center_x = 0.0;
    double center_y = 0.0;
    std::uint32_t image_width = 0;
    std::uint32_t image_height = 0;
    std::uint64_t frame_id = 0;
    Clock::time_point produced_at{};      // steady clock, staleness checks only
    builtin_interfaces::msg::Time stamp;  // inference completion time, diagnostics
    // Target position on the ground plane of the local_position frame, for the
    // periodic log only. False whenever the pose feed is stale or absent.
    bool ground_valid = false;
    double ground_x = 0.0;
    double ground_y = 0.0;
  };

  struct CameraSettings
  {
    int exposure_auto = V4L2_EXPOSURE_APERTURE_PRIORITY;
    int exposure_absolute = 40;
    int exposure_auto_priority = 0;
    int gain = 190;
    int brightness = 128;
    int contrast = 65;
    int saturation = 90;
    int gamma = 130;
    int sharpness = 128;
    int backlight_compensation = 16;
    int white_balance_auto = 1;
    int white_balance_temperature = 4650;
    int power_line_frequency = 1;
    int focus_auto = 1;
    int focus_absolute = 0;
    int zoom_absolute = 120;
  };

  void declareParameters()
  {
    declare_parameter<std::string>("camera_profile", "default");
    declare_parameter<std::string>("camera_device", "/dev/video1");
    declare_parameter<std::string>("model_path", "");
    declare_parameter<int>("camera_width", 1280);
    declare_parameter<int>("camera_height", 720);
    declare_parameter<int>("camera_fps", 120);
    declare_parameter<int>("decode_width", 1280);
    declare_parameter<int>("decode_height", 720);
    declare_parameter<int>("exposure_auto", 3);
    declare_parameter<int>("exposure_absolute", 40);
    declare_parameter<int>("exposure_auto_priority", 0);
    declare_parameter<int>("gain", 190);
    declare_parameter<int>("brightness", 128);
    declare_parameter<int>("contrast", 65);
    declare_parameter<int>("saturation", 90);
    declare_parameter<int>("gamma", 130);
    declare_parameter<int>("sharpness", 128);
    declare_parameter<int>("backlight_compensation", 16);
    declare_parameter<int>("white_balance_auto", 1);
    declare_parameter<int>("white_balance_temperature", 4650);
    declare_parameter<int>("power_line_frequency", 1);
    declare_parameter<int>("focus_auto", 1);
    declare_parameter<int>("focus_absolute", 0);
    declare_parameter<int>("zoom_absolute", 120);
    declare_parameter<bool>("display_enabled", true);
    declare_parameter<double>("display_fps_limit", 60.0);
    declare_parameter<double>("confidence_threshold", 0.5);
    declare_parameter<double>("nms_threshold", 0.45);
    declare_parameter<bool>("enable_zero_copy", true);
    declare_parameter<bool>("enable_rga_preprocess", true);
    declare_parameter<bool>("cpu_affinity_enabled", true);
    declare_parameter<std::string>("servo_target_topic", "/vision/servo/target");
    declare_parameter<std::string>("servo_status_topic", "/control/vision_servo/status");
    declare_parameter<double>("servo_publish_rate_hz", 25.0);
    declare_parameter<double>("servo_stale_timeout_s", 0.25);
    declare_parameter<int>("servo_confirm_min_hits", 5);
    declare_parameter<double>("servo_confirm_min_score", 0.55);
    declare_parameter<double>("servo_confirm_max_center_jump", 0.12);
    declare_parameter<double>("servo_confirm_max_area_ratio", 1.8);
    declare_parameter<double>("servo_log_period_s", 1.0);
    declare_parameter<double>("track_iou_threshold", 0.3);
    declare_parameter<double>("track_max_missed_s", 1.5);
    declare_parameter<bool>("ground_projection_enabled", true);
    declare_parameter<std::string>("local_pose_topic", "/mavros/local_position/pose");
    declare_parameter<double>("camera_fx", 914.0);
    declare_parameter<double>("camera_fy", 914.0);
    declare_parameter<double>("camera_cx", 640.0);
    declare_parameter<double>("camera_cy", 360.0);
    declare_parameter<int>("camera_mount_yaw_deg", 0);
    declare_parameter<double>("camera_offset_x", 0.0);
    declare_parameter<double>("camera_offset_y", -0.08);
    declare_parameter<double>("camera_offset_z", 0.08);
    declare_parameter<double>("pose_stale_s", 0.3);
  }

  void readParameters()
  {
    camera_profile_ = get_parameter("camera_profile").as_string();
    camera_device_ = get_parameter("camera_device").as_string();
    model_path_ = get_parameter("model_path").as_string();
    camera_width_ = static_cast<int>(get_parameter("camera_width").as_int());
    camera_height_ = static_cast<int>(get_parameter("camera_height").as_int());
    camera_fps_ = static_cast<int>(get_parameter("camera_fps").as_int());
    decode_width_ = static_cast<int>(get_parameter("decode_width").as_int());
    decode_height_ = static_cast<int>(get_parameter("decode_height").as_int());
    camera_settings_.exposure_auto = static_cast<int>(get_parameter("exposure_auto").as_int());
    camera_settings_.exposure_absolute = static_cast<int>(get_parameter("exposure_absolute").as_int());
    camera_settings_.exposure_auto_priority = static_cast<int>(get_parameter("exposure_auto_priority").as_int());
    camera_settings_.gain = static_cast<int>(get_parameter("gain").as_int());
    camera_settings_.brightness = static_cast<int>(get_parameter("brightness").as_int());
    camera_settings_.contrast = static_cast<int>(get_parameter("contrast").as_int());
    camera_settings_.saturation = static_cast<int>(get_parameter("saturation").as_int());
    camera_settings_.gamma = static_cast<int>(get_parameter("gamma").as_int());
    camera_settings_.sharpness = static_cast<int>(get_parameter("sharpness").as_int());
    camera_settings_.backlight_compensation = static_cast<int>(get_parameter("backlight_compensation").as_int());
    camera_settings_.white_balance_auto = static_cast<int>(get_parameter("white_balance_auto").as_int());
    camera_settings_.white_balance_temperature = static_cast<int>(
      get_parameter("white_balance_temperature").as_int());
    camera_settings_.power_line_frequency = static_cast<int>(get_parameter("power_line_frequency").as_int());
    camera_settings_.focus_auto = static_cast<int>(get_parameter("focus_auto").as_int());
    camera_settings_.focus_absolute = static_cast<int>(get_parameter("focus_absolute").as_int());
    camera_settings_.zoom_absolute = static_cast<int>(get_parameter("zoom_absolute").as_int());
    display_enabled_ = get_parameter("display_enabled").as_bool();
    display_fps_limit_ = get_parameter("display_fps_limit").as_double();
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    nms_threshold_ = static_cast<float>(get_parameter("nms_threshold").as_double());
    enable_zero_copy_ = get_parameter("enable_zero_copy").as_bool();
    enable_rga_preprocess_ = get_parameter("enable_rga_preprocess").as_bool();
    cpu_affinity_enabled_ = get_parameter("cpu_affinity_enabled").as_bool();
    servo_target_topic_ = get_parameter("servo_target_topic").as_string();
    servo_status_topic_ = get_parameter("servo_status_topic").as_string();
    servo_publish_rate_hz_ = get_parameter("servo_publish_rate_hz").as_double();
    servo_stale_timeout_s_ = get_parameter("servo_stale_timeout_s").as_double();
    servo_confirm_min_hits_ = static_cast<int>(get_parameter("servo_confirm_min_hits").as_int());
    servo_confirm_min_score_ = get_parameter("servo_confirm_min_score").as_double();
    servo_confirm_max_center_jump_ = get_parameter("servo_confirm_max_center_jump").as_double();
    servo_confirm_max_area_ratio_ = get_parameter("servo_confirm_max_area_ratio").as_double();
    servo_log_period_s_ = get_parameter("servo_log_period_s").as_double();
    track_iou_threshold_ = static_cast<float>(get_parameter("track_iou_threshold").as_double());
    track_max_missed_s_ = get_parameter("track_max_missed_s").as_double();
    ground_projection_enabled_ = get_parameter("ground_projection_enabled").as_bool();
    local_pose_topic_ = get_parameter("local_pose_topic").as_string();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    camera_mount_yaw_deg_ = static_cast<int>(get_parameter("camera_mount_yaw_deg").as_int());
    camera_offset_x_ = get_parameter("camera_offset_x").as_double();
    camera_offset_y_ = get_parameter("camera_offset_y").as_double();
    camera_offset_z_ = get_parameter("camera_offset_z").as_double();
    pose_stale_s_ = get_parameter("pose_stale_s").as_double();
    const double mount_rad = static_cast<double>(camera_mount_yaw_deg_) * M_PI / 180.0;
    mount_cos_ = std::cos(mount_rad);
    mount_sin_ = std::sin(mount_rad);
  }

  void validateParameters() const
  {
    if (camera_device_.empty() || model_path_.empty()) {
      throw std::invalid_argument("camera_device and model_path must not be empty");
    }
    if (camera_device_.find_first_of(" \t\r\n!\"'") != std::string::npos) {
      throw std::invalid_argument("camera_device contains unsupported characters");
    }
    if (camera_width_ <= 0 || camera_height_ <= 0 || camera_fps_ <= 0 ||
      decode_width_ <= 0 || decode_height_ <= 0)
    {
      throw std::invalid_argument("camera/decode dimensions and camera_fps must be positive");
    }
    if (display_fps_limit_ < 0.0) {
      throw std::invalid_argument("display_fps_limit must be >= 0");
    }
    if (servo_target_topic_.empty() || servo_status_topic_.empty()) {
      throw std::invalid_argument("servo_target_topic and servo_status_topic must not be empty");
    }
    if (servo_publish_rate_hz_ < 15.0 || servo_publish_rate_hz_ > 60.0) {
      // The vision servo interface contract requires a sustained rate of >= 15 Hz.
      throw std::invalid_argument("servo_publish_rate_hz must be in [15, 60]");
    }
    if (servo_stale_timeout_s_ <= 0.0 || servo_stale_timeout_s_ > 0.5) {
      // Must stay well below the control side lost_timeout_s (1.0 s) so the
      // vision side declares target loss before the controller keeps flying on
      // stale errors.
      throw std::invalid_argument("servo_stale_timeout_s must be in (0, 0.5]");
    }
    if (servo_confirm_min_hits_ < 1 || servo_confirm_min_hits_ > 1000) {
      throw std::invalid_argument("servo_confirm_min_hits must be in [1, 1000]");
    }
    if (servo_confirm_min_score_ < 0.0 || servo_confirm_min_score_ > 1.0) {
      throw std::invalid_argument("servo_confirm_min_score must be in [0, 1]");
    }
    if (servo_confirm_max_center_jump_ <= 0.0 || servo_confirm_max_center_jump_ > 1.0) {
      throw std::invalid_argument("servo_confirm_max_center_jump must be in (0, 1]");
    }
    if (servo_confirm_max_area_ratio_ < 1.0) {
      throw std::invalid_argument("servo_confirm_max_area_ratio must be >= 1.0");
    }
    if (servo_log_period_s_ < 0.0) {
      throw std::invalid_argument("servo_log_period_s must be >= 0");
    }
    if (track_iou_threshold_ < 0.0F || track_iou_threshold_ > 1.0F) {
      throw std::invalid_argument("track_iou_threshold must be in [0, 1]");
    }
    if (track_max_missed_s_ < 1.0) {
      // Must cover the control side lost_timeout_s (1.0 s): a track that dies
      // inside that window changes target_id on reappearance and the controller,
      // which has permanently locked the old id, can never reacquire it.
      throw std::invalid_argument("track_max_missed_s must be >= 1.0");
    }
    if (local_pose_topic_.empty()) {
      throw std::invalid_argument("local_pose_topic must not be empty");
    }
    // 上限按 5-50mm 变焦镜头长焦端取：50mm 全幅缩放约 fx=10300，若 720p
    // 走裁切读出可到 3 万级，故放宽到 50000（仍能拦住数量级手误）。
    if (camera_fx_ < 50.0 || camera_fx_ > 50000.0 ||
      camera_fy_ < 50.0 || camera_fy_ > 50000.0)
    {
      throw std::invalid_argument("camera_fx/camera_fy must be in [50, 50000]");
    }
    if (camera_cx_ < 0.0 || camera_cy_ < 0.0) {
      throw std::invalid_argument("camera_cx/camera_cy must be >= 0");
    }
    if (camera_mount_yaw_deg_ != 0 && camera_mount_yaw_deg_ != 90 &&
      camera_mount_yaw_deg_ != 180 && camera_mount_yaw_deg_ != 270)
    {
      throw std::invalid_argument("camera_mount_yaw_deg must be 0, 90, 180 or 270");
    }
    if (pose_stale_s_ <= 0.0 || pose_stale_s_ > 5.0) {
      throw std::invalid_argument("pose_stale_s must be in (0, 5]");
    }
  }

  void configureProcessAffinity()
  {
    if (!cpu_affinity_enabled_) {
      return;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int cpu = 4; cpu <= 7; ++cpu) {
      CPU_SET(cpu, &set);
    }
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
      RCLCPP_WARN(get_logger(), "Unable to set process CPU affinity to CPU4-7: %s", strerror(errno));
    } else {
      RCLCPP_INFO(get_logger(), "Process CPU affinity set to CPU4-7");
    }
  }

  struct CameraControlDefinition
  {
    std::uint64_t mask;
    std::uint32_t id;
    const char *name;
  };

  static constexpr std::uint64_t kExposureAuto = 1ULL << 0;
  static constexpr std::uint64_t kExposureAbsolute = 1ULL << 1;
  static constexpr std::uint64_t kExposureAutoPriority = 1ULL << 2;
  static constexpr std::uint64_t kGain = 1ULL << 3;
  static constexpr std::uint64_t kBrightness = 1ULL << 4;
  static constexpr std::uint64_t kContrast = 1ULL << 5;
  static constexpr std::uint64_t kSaturation = 1ULL << 6;
  static constexpr std::uint64_t kGamma = 1ULL << 7;
  static constexpr std::uint64_t kSharpness = 1ULL << 8;
  static constexpr std::uint64_t kBacklightCompensation = 1ULL << 9;
  static constexpr std::uint64_t kWhiteBalanceAuto = 1ULL << 10;
  static constexpr std::uint64_t kWhiteBalanceTemperature = 1ULL << 11;
  static constexpr std::uint64_t kPowerLineFrequency = 1ULL << 12;
  static constexpr std::uint64_t kFocusAuto = 1ULL << 13;
  static constexpr std::uint64_t kFocusAbsolute = 1ULL << 14;
  static constexpr std::uint64_t kZoomAbsolute = 1ULL << 15;
  static constexpr std::uint64_t kAllCameraControls = (1ULL << 16) - 1ULL;

  static constexpr std::array<CameraControlDefinition, 16> kCameraControlDefinitions{{
      {kExposureAuto, V4L2_CID_EXPOSURE_AUTO, "exposure_auto"},
      {kExposureAbsolute, V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_absolute"},
      {kExposureAutoPriority, V4L2_CID_EXPOSURE_AUTO_PRIORITY, "exposure_auto_priority"},
      {kGain, V4L2_CID_GAIN, "gain"},
      {kBrightness, V4L2_CID_BRIGHTNESS, "brightness"},
      {kContrast, V4L2_CID_CONTRAST, "contrast"},
      {kSaturation, V4L2_CID_SATURATION, "saturation"},
      {kGamma, V4L2_CID_GAMMA, "gamma"},
      {kSharpness, V4L2_CID_SHARPNESS, "sharpness"},
      {kBacklightCompensation, V4L2_CID_BACKLIGHT_COMPENSATION, "backlight_compensation"},
      {kWhiteBalanceAuto, V4L2_CID_AUTO_WHITE_BALANCE, "white_balance_auto"},
      {kWhiteBalanceTemperature, V4L2_CID_WHITE_BALANCE_TEMPERATURE, "white_balance_temperature"},
      {kPowerLineFrequency, V4L2_CID_POWER_LINE_FREQUENCY, "power_line_frequency"},
      {kFocusAuto, V4L2_CID_FOCUS_AUTO, "focus_auto"},
      {kFocusAbsolute, V4L2_CID_FOCUS_ABSOLUTE, "focus_absolute"},
      {kZoomAbsolute, V4L2_CID_ZOOM_ABSOLUTE, "zoom_absolute"},
    }};

  static int getCameraSetting(const CameraSettings &settings, std::uint64_t mask)
  {
    switch (mask) {
      case kExposureAuto: return settings.exposure_auto;
      case kExposureAbsolute: return settings.exposure_absolute;
      case kExposureAutoPriority: return settings.exposure_auto_priority;
      case kGain: return settings.gain;
      case kBrightness: return settings.brightness;
      case kContrast: return settings.contrast;
      case kSaturation: return settings.saturation;
      case kGamma: return settings.gamma;
      case kSharpness: return settings.sharpness;
      case kBacklightCompensation: return settings.backlight_compensation;
      case kWhiteBalanceAuto: return settings.white_balance_auto;
      case kWhiteBalanceTemperature: return settings.white_balance_temperature;
      case kPowerLineFrequency: return settings.power_line_frequency;
      case kFocusAuto: return settings.focus_auto;
      case kFocusAbsolute: return settings.focus_absolute;
      case kZoomAbsolute: return settings.zoom_absolute;
      default: return 0;
    }
  }

  static void setCameraSetting(CameraSettings &settings, std::uint64_t mask, int value)
  {
    switch (mask) {
      case kExposureAuto: settings.exposure_auto = value; break;
      case kExposureAbsolute: settings.exposure_absolute = value; break;
      case kExposureAutoPriority: settings.exposure_auto_priority = value; break;
      case kGain: settings.gain = value; break;
      case kBrightness: settings.brightness = value; break;
      case kContrast: settings.contrast = value; break;
      case kSaturation: settings.saturation = value; break;
      case kGamma: settings.gamma = value; break;
      case kSharpness: settings.sharpness = value; break;
      case kBacklightCompensation: settings.backlight_compensation = value; break;
      case kWhiteBalanceAuto: settings.white_balance_auto = value; break;
      case kWhiteBalanceTemperature: settings.white_balance_temperature = value; break;
      case kPowerLineFrequency: settings.power_line_frequency = value; break;
      case kFocusAuto: settings.focus_auto = value; break;
      case kFocusAbsolute: settings.focus_absolute = value; break;
      case kZoomAbsolute: settings.zoom_absolute = value; break;
      default: break;
    }
  }

  static const CameraControlDefinition *findCameraControl(std::uint64_t mask)
  {
    for (const auto &definition : kCameraControlDefinitions) {
      if (definition.mask == mask) {
        return &definition;
      }
    }
    return nullptr;
  }

  bool queryCameraControl(int fd, const CameraControlDefinition &definition, v4l2_queryctrl &query) const
  {
    std::memset(&query, 0, sizeof(query));
    query.id = definition.id;
    return ioctl(fd, VIDIOC_QUERYCTRL, &query) == 0 &&
      (query.flags & V4L2_CTRL_FLAG_DISABLED) == 0;
  }

  bool readCameraControl(
    int fd, const CameraControlDefinition &definition, CameraSettings &settings,
    std::uint64_t *available_mask = nullptr, std::uint64_t *writable_mask = nullptr,
    std::uint64_t *active_mask = nullptr) const
  {
    v4l2_queryctrl query{};
    if (!queryCameraControl(fd, definition, query)) {
      return false;
    }
    if (available_mask != nullptr) {
      *available_mask |= definition.mask;
    }
    if ((query.flags & V4L2_CTRL_FLAG_READ_ONLY) == 0 && writable_mask != nullptr) {
      *writable_mask |= definition.mask;
    }
    if ((query.flags & V4L2_CTRL_FLAG_INACTIVE) == 0 && active_mask != nullptr) {
      *active_mask |= definition.mask;
    }
    v4l2_control control{};
    control.id = definition.id;
    if (ioctl(fd, VIDIOC_G_CTRL, &control) == 0) {
      setCameraSetting(settings, definition.mask, control.value);
    }
    return true;
  }

  void readAllCameraControls(
    int fd, CameraSettings &settings, std::uint64_t *available_mask = nullptr,
    std::uint64_t *writable_mask = nullptr, std::uint64_t *active_mask = nullptr) const
  {
    if (available_mask != nullptr) { *available_mask = 0U; }
    if (writable_mask != nullptr) { *writable_mask = 0U; }
    if (active_mask != nullptr) { *active_mask = 0U; }
    for (const auto &definition : kCameraControlDefinitions) {
      readCameraControl(fd, definition, settings, available_mask, writable_mask, active_mask);
    }
  }

  bool writeCameraControl(
    int fd, const CameraControlDefinition &definition, int requested, CameraSettings &settings,
    std::string &reason)
  {
    v4l2_queryctrl query{};
    if (!queryCameraControl(fd, definition, query)) {
      reason = std::string(definition.name) + " is unavailable";
      return false;
    }
    if ((query.flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE)) != 0) {
      reason = std::string(definition.name) + " is not writable in the current mode";
      return false;
    }
    int value = requested;
    if (query.type == V4L2_CTRL_TYPE_BOOLEAN) {
      value = requested == 0 ? 0 : 1;
    } else if (query.type == V4L2_CTRL_TYPE_MENU || query.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
      v4l2_querymenu menu{};
      menu.id = definition.id;
      menu.index = static_cast<std::uint32_t>(requested);
      if (requested < query.minimum || requested > query.maximum ||
        ioctl(fd, VIDIOC_QUERYMENU, &menu) != 0)
      {
        reason = std::string(definition.name) + " is not a valid menu value";
        return false;
      }
    } else {
      const int bounded = std::clamp(requested, query.minimum, query.maximum);
      const int step = std::max(1, query.step);
      value = query.minimum + ((bounded - query.minimum) / step) * step;
    }
    v4l2_control control{};
    control.id = definition.id;
    control.value = value;
    if (ioctl(fd, VIDIOC_S_CTRL, &control) != 0) {
      reason = std::string("failed to set ") + definition.name + ": " + strerror(errno);
      return false;
    }
    if (ioctl(fd, VIDIOC_G_CTRL, &control) != 0) {
      reason = std::string("failed to read back ") + definition.name + ": " + strerror(errno);
      return false;
    }
    setCameraSetting(settings, definition.mask, control.value);
    if (value != requested) {
      reason = std::string(definition.name) + " was clamped to " + std::to_string(value);
    }
    return true;
  }

  void configureCameraControls()
  {
    std::lock_guard<std::mutex> lock(camera_control_mutex_);
    const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
      throw std::runtime_error("failed to open camera controls on " + camera_device_ +
        ": " + strerror(errno));
    }
    applyCameraSettings(
      fd, camera_settings_, kAllCameraControls, camera_settings_, nullptr, nullptr, nullptr, false);
    close(fd);
  }

  void applyCameraSettings(
    int fd, const CameraSettings &requested, std::uint64_t update_mask, CameraSettings &settings,
    std::uint64_t *applied_result = nullptr, std::uint64_t *rejected_result = nullptr,
    std::vector<std::string> *reasons_result = nullptr, bool reject_inactive_dependents = true)
  {
    std::uint64_t applied_mask = 0U;
    std::uint64_t rejected_mask = 0U;
    std::vector<std::string> reasons;
    const auto apply_one = [&](std::uint64_t mask) {
        if ((update_mask & mask) == 0U) {
          return;
        }
        const CameraControlDefinition *definition = findCameraControl(mask);
        std::string reason;
        if (definition != nullptr && writeCameraControl(
            fd, *definition, getCameraSetting(requested, mask), settings, reason))
        {
          applied_mask |= mask;
          if (!reason.empty()) {
            reasons.push_back(reason);
          }
        } else {
          rejected_mask |= mask;
          reasons.push_back(reason.empty() ? "unknown camera control error" : reason);
        }
      };

    // Automatic modes must change first so the matching manual value can be set in one command.
    apply_one(kExposureAuto);
    apply_one(kWhiteBalanceAuto);
    apply_one(kFocusAuto);
    for (const auto &definition : kCameraControlDefinitions) {
      if (definition.mask == kExposureAuto || definition.mask == kWhiteBalanceAuto ||
        definition.mask == kFocusAuto || definition.mask == kExposureAbsolute ||
        definition.mask == kWhiteBalanceTemperature || definition.mask == kFocusAbsolute)
      {
        continue;
      }
      apply_one(definition.mask);
    }

    const auto apply_dependent = [&](std::uint64_t mask, bool enabled, const char *reason) {
        if ((update_mask & mask) == 0U) {
          return;
        }
        if (enabled) {
          apply_one(mask);
        } else if (reject_inactive_dependents) {
          rejected_mask |= mask;
          reasons.emplace_back(reason);
        }
      };
    apply_dependent(
      kExposureAbsolute, settings.exposure_auto == V4L2_EXPOSURE_MANUAL,
      "exposure_absolute requires manual exposure");
    apply_dependent(
      kWhiteBalanceTemperature, settings.white_balance_auto == 0,
      "white_balance_temperature requires manual white balance");
    apply_dependent(
      kFocusAbsolute, settings.focus_auto == 0,
      "focus_absolute requires manual focus");
    readAllCameraControls(fd, settings);

    if (applied_result != nullptr) { *applied_result = applied_mask; }
    if (rejected_result != nullptr) { *rejected_result = rejected_mask; }
    if (reasons_result != nullptr) { *reasons_result = std::move(reasons); }
  }

  static CameraSettings settingsFromParams(
    const drone_msgs::msg::IndustrialCameraParams &params)
  {
    CameraSettings settings;
    settings.exposure_auto = params.auto_exposure ? V4L2_EXPOSURE_APERTURE_PRIORITY :
      V4L2_EXPOSURE_MANUAL;
    settings.exposure_absolute = params.exposure_absolute;
    settings.exposure_auto_priority = params.auto_exposure_priority ? 1 : 0;
    settings.gain = params.gain;
    settings.brightness = params.brightness;
    settings.contrast = params.contrast;
    settings.saturation = params.saturation;
    settings.gamma = params.gamma;
    settings.sharpness = params.sharpness;
    settings.backlight_compensation = params.backlight_compensation;
    settings.white_balance_auto = params.auto_white_balance ? 1 : 0;
    settings.white_balance_temperature = params.white_balance_temperature;
    settings.power_line_frequency = static_cast<int>(params.power_line_frequency);
    settings.focus_auto = params.auto_focus ? 1 : 0;
    settings.focus_absolute = params.focus_absolute;
    settings.zoom_absolute = params.zoom_absolute;
    return settings;
  }

  void handleCameraParams(const drone_msgs::msg::IndustrialCameraParams::SharedPtr params)
  {
    std::lock_guard<std::mutex> lock(camera_control_mutex_);
    const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "Unable to open industrial camera controls: %s", strerror(errno));
      return;
    }

    std::uint64_t rejected_mask = 0U;
    std::vector<std::string> reasons;
    applyCameraSettings(
      fd, settingsFromParams(*params), kAllCameraControls, camera_settings_, nullptr,
      &rejected_mask, &reasons, false);
    close(fd);

    for (const std::string &reason : reasons) {
      RCLCPP_WARN(get_logger(), "Industrial camera control result: %s", reason.c_str());
    }
    if (rejected_mask != 0U) {
      RCLCPP_WARN(get_logger(), "Some industrial camera parameters were not applied");
    }
  }

  void initializeDetectors()
  {
    constexpr std::array<rknn_core_mask, kWorkerCount> masks{
      RKNN_NPU_CORE_0, RKNN_NPU_CORE_1, RKNN_NPU_CORE_2};
    const std::vector<std::string> class_names{
      "elephant", "tiger", "wolf", "peacock", "monkey"};
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      detectors_[index] = std::make_unique<RknnYoloDetector>(
        model_path_, masks[index], enable_zero_copy_, class_names,
        confidence_threshold_, nms_threshold_, 114, enable_rga_preprocess_);
      const rknn_mem_size memory = detectors_[index]->memorySize();
      weight_mib_ += static_cast<double>(memory.total_weight_size) / (1024.0 * 1024.0);
      internal_mib_ += static_cast<double>(memory.total_internal_size) / (1024.0 * 1024.0);
      dma_mib_ += static_cast<double>(memory.total_dma_allocated_size) / (1024.0 * 1024.0);
    }
    api_version_ = detectors_[0]->apiVersion();
    driver_version_ = detectors_[0]->driverVersion();
    zero_copy_mode_ = detectors_[0]->zeroCopyModeName();
    preprocess_mode_ = detectors_[0]->preprocessModeName();
    model_input_width_ = detectors_[0]->inputWidth();
    model_input_height_ = detectors_[0]->inputHeight();
    model_output_count_ = detectors_[0]->outputCount();
  }

  static float intersectionOverUnion(const cv::Rect &first, const cv::Rect &second)
  {
    const cv::Rect overlap = first & second;
    if (overlap.empty()) {
      return 0.0F;
    }
    const float intersection = static_cast<float>(overlap.area());
    const float union_area = static_cast<float>(first.area() + second.area() - overlap.area());
    return union_area > 0.0F ? intersection / union_area : 0.0F;
  }

  void enqueueDetectionFrame(
    std::uint64_t frame_id,
    int image_width,
    int image_height,
    const std::vector<Detection> &detections,
    const RknnYoloDetector &detector)
  {
    DetectionFrame frame;
    frame.frame_id = frame_id;
    frame.image_width = image_width;
    frame.image_height = image_height;
    frame.produced_at = Clock::now();
    frame.detections = detections;
    frame.labels.reserve(detections.size());
    for (const Detection &detection : detections) {
      frame.labels.push_back(detector.classLabel(detection.class_id));
    }

    {
      std::lock_guard<std::mutex> lock(detection_result_mutex_);
      // A frame whose slot was already abandoned by the timeout in
      // detectionPublishLoop must not re-enter the map: its id is below the
      // cursor and would drag the ordered publish sequence backwards.
      if (frame_id < next_detection_frame_id_) {
        return;
      }
      pending_detection_frames_.emplace(frame_id, std::move(frame));
    }
    detection_result_ready_.notify_one();
  }

  void markDetectionFrameSkipped(std::uint64_t frame_id)
  {
    {
      std::lock_guard<std::mutex> lock(detection_result_mutex_);
      if (frame_id < next_detection_frame_id_) {
        return;
      }
      skipped_detection_frames_.insert(frame_id);
    }
    detection_result_ready_.notify_one();
  }

  void detectionPublishLoop()
  {
    while (running_.load() && rclcpp::ok()) {
      std::optional<DetectionFrame> frame;
      {
        std::unique_lock<std::mutex> lock(detection_result_mutex_);
        // Bounded wait: a frame handed to a worker that never returns (an NPU
        // submit that stalls, for example) lands in neither the pending nor the
        // skipped set. An unbounded wait would freeze this loop -- and therefore
        // the servo snapshot -- permanently while the other workers keep running,
        // which looks perfectly healthy from the display and the frame counters.
        const bool ready = detection_result_ready_.wait_for(
          lock, std::chrono::milliseconds(kDetectionFrameWaitMs), [this] {
            return pending_detection_frames_.count(next_detection_frame_id_) > 0U ||
                   skipped_detection_frames_.count(next_detection_frame_id_) > 0U ||
                   !running_.load() || !rclcpp::ok();
          });
        if (!ready) {
          // Jump to the smallest frame id that can still make progress, from
          // either set; both only hold ids at or above the cursor.
          std::uint64_t jump_to = 0;
          bool has_jump = false;
          if (!pending_detection_frames_.empty()) {
            jump_to = pending_detection_frames_.begin()->first;
            has_jump = true;
          }
          if (!skipped_detection_frames_.empty()) {
            const std::uint64_t skipped_front = *skipped_detection_frames_.begin();
            if (!has_jump || skipped_front < jump_to) {
              jump_to = skipped_front;
              has_jump = true;
            }
          }
          if (has_jump && jump_to > next_detection_frame_id_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Detection frame %llu did not complete within %d ms; skipping to %llu",
              static_cast<unsigned long long>(next_detection_frame_id_),
              kDetectionFrameWaitMs,
              static_cast<unsigned long long>(jump_to));
            next_detection_frame_id_ = jump_to;
          }
          continue;
        }

        while (skipped_detection_frames_.erase(next_detection_frame_id_) > 0U) {
          ++next_detection_frame_id_;
        }
        const auto pending = pending_detection_frames_.find(next_detection_frame_id_);
        if (pending == pending_detection_frames_.end()) {
          continue;
        }
        frame = std::move(pending->second);
        pending_detection_frames_.erase(pending);
        ++next_detection_frame_id_;
      }

      try {
        updateTrackingAndSnapshot(*frame);
      } catch (const std::exception &error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Animal vision tracking update failed: %s", error.what());
      }
    }
  }

  void updateTrackingAndSnapshot(const DetectionFrame &frame)
  {
    std::vector<TrackMatch> candidates;
    candidates.reserve(frame.detections.size() * tracked_detections_.size());
    for (std::size_t detection_index = 0; detection_index < frame.detections.size(); ++detection_index) {
      for (std::size_t track_index = 0; track_index < tracked_detections_.size(); ++track_index) {
        const TrackedDetection &track = tracked_detections_[track_index];
        if (track.class_id != frame.detections[detection_index].class_id) {
          continue;
        }
        const float iou = intersectionOverUnion(frame.detections[detection_index].box, track.box);
        if (iou >= track_iou_threshold_) {
          candidates.push_back(TrackMatch{detection_index, track_index, iou});
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const TrackMatch &first, const TrackMatch &second) {
      return first.iou > second.iou;
    });

    const double image_diagonal = std::hypot(
      static_cast<double>(frame.image_width), static_cast<double>(frame.image_height));
    std::vector<std::uint64_t> track_ids(frame.detections.size(), 0);
    std::vector<bool> detection_matched(frame.detections.size(), false);
    std::vector<bool> track_matched(tracked_detections_.size(), false);
    for (const TrackMatch &candidate : candidates) {
      if (detection_matched[candidate.detection_index] || track_matched[candidate.track_index]) {
        continue;
      }
      TrackedDetection &track = tracked_detections_[candidate.track_index];
      const Detection &detection = frame.detections[candidate.detection_index];
      // A hit advances the confirmation streak only while score, center motion and
      // box area stay stable; an unstable hit restarts the streak at this frame.
      const double center_jump = image_diagonal > 0.0 ? std::hypot(
        static_cast<double>(detection.center.x - track.last_center.x),
        static_cast<double>(detection.center.y - track.last_center.y)) / image_diagonal : 1.0;
      const int area = std::max(1, detection.box.area());
      const int last_area = std::max(1, track.last_area);
      const double area_ratio = static_cast<double>(std::max(area, last_area)) /
        static_cast<double>(std::min(area, last_area));
      const bool stable_step =
        static_cast<double>(detection.score) >= servo_confirm_min_score_ &&
        center_jump <= servo_confirm_max_center_jump_ &&
        area_ratio <= servo_confirm_max_area_ratio_;
      track.hit_streak = stable_step ? track.hit_streak + 1U : 1U;
      if (!track.confirmed &&
        track.hit_streak >= static_cast<std::uint32_t>(servo_confirm_min_hits_))
      {
        track.confirmed = true;
        RCLCPP_INFO(get_logger(),
          "ANIMAL_SERVO_EVENT type=target_confirmed id=%s hits=%u frame=%llu",
          makeTargetId(track.label, track.track_id).c_str(), track.hit_streak,
          static_cast<unsigned long long>(frame.frame_id));
      }
      track.last_center = detection.center;
      track.last_area = detection.box.area();
      track.last_score = detection.score;
      track.box = detection.box;
      track.last_seen_frame_id = frame.frame_id;
      track.last_seen_at = frame.produced_at;
      track_ids[candidate.detection_index] = track.track_id;
      detection_matched[candidate.detection_index] = true;
      track_matched[candidate.track_index] = true;
    }

    // Must run before new tracks are appended: track_matched indexes the
    // pre-append vector.
    for (std::size_t track_index = 0; track_index < track_matched.size(); ++track_index) {
      if (!track_matched[track_index]) {
        tracked_detections_[track_index].hit_streak = 0U;
      }
    }

    for (std::size_t detection_index = 0; detection_index < frame.detections.size(); ++detection_index) {
      if (detection_matched[detection_index]) {
        continue;
      }
      if (next_track_id_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::runtime_error("animal vision track_id space exhausted");
      }
      const Detection &detection = frame.detections[detection_index];
      const std::uint64_t track_id = next_track_id_++;
      tracked_detections_.push_back(TrackedDetection{
        track_id,
        detection.class_id,
        detection.box,
        frame.frame_id,
        frame.produced_at,
        frame.labels[detection_index],
        1U,
        false,
        detection.center,
        detection.box.area(),
        detection.score});
      track_ids[detection_index] = track_id;
    }

    tracked_detections_.erase(
      std::remove_if(
        tracked_detections_.begin(), tracked_detections_.end(),
        [this, &frame](const TrackedDetection &track) {
          return elapsedSeconds(track.last_seen_at, frame.produced_at) >
                 track_max_missed_s_;
        }),
      tracked_detections_.end());

    updateServoSnapshot(frame, track_ids);
  }

  static std::string makeTargetId(const std::string &label, std::uint64_t track_id)
  {
    return (label.empty() ? std::string("target") : label) + "_" + std::to_string(track_id);
  }

  struct PoseSample
  {
    bool valid = false;
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
  };

  void handleLocalPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    // Runs on the executor thread: copy only, no I/O and no logging here.
    std::lock_guard<std::mutex> lock(servo_mutex_);
    pose_px_ = pose->pose.position.x;
    pose_py_ = pose->pose.position.y;
    pose_pz_ = pose->pose.position.z;
    pose_qw_ = pose->pose.orientation.w;
    pose_qx_ = pose->pose.orientation.x;
    pose_qy_ = pose->pose.orientation.y;
    pose_qz_ = pose->pose.orientation.z;
    pose_received_at_ = Clock::now();
    pose_received_ = true;
  }

  PoseSample currentPoseSample()
  {
    PoseSample sample;
    std::lock_guard<std::mutex> lock(servo_mutex_);
    if (!pose_received_ ||
      elapsedSeconds(pose_received_at_, Clock::now()) > pose_stale_s_)
    {
      return sample;  // no flight controller feed (bench) or a stale one: stay dormant
    }
    sample.valid = true;
    sample.px = pose_px_;
    sample.py = pose_py_;
    sample.pz = pose_pz_;
    sample.qw = pose_qw_;
    sample.qx = pose_qx_;
    sample.qy = pose_qy_;
    sample.qz = pose_qz_;
    return sample;
  }

  static std::array<double, 3> rotateByQuaternion(
    double qw, double qx, double qy, double qz, const std::array<double, 3> &v)
  {
    // v' = v + qw * t + q_vec x t, where t = 2 * q_vec x v.
    const double tx = 2.0 * (qy * v[2] - qz * v[1]);
    const double ty = 2.0 * (qz * v[0] - qx * v[2]);
    const double tz = 2.0 * (qx * v[1] - qy * v[0]);
    return {
      v[0] + qw * tx + (qy * tz - qz * ty),
      v[1] + qw * ty + (qz * tx - qx * tz),
      v[2] + qw * tz + (qx * ty - qy * tx)};
  }

  // Projects pixel (u, v) onto the ground plane (z = 0) of the local_position
  // frame. The pose z is the rangefinder-fused height above ground, so the
  // camera height is pose z plus the rotated lens-above-rangefinder offset.
  std::optional<std::pair<double, double>> projectPixelToGround(
    const PoseSample &pose, double u, double v) const
  {
    // Pixel to a unit-depth ray in the optical frame.
    const double xc = (u - camera_cx_) / camera_fx_;
    const double yc = (v - camera_cy_) / camera_fy_;
    // Mount yaw about the optical axis; 0 deg = image top toward the nose
    // (bench-verified: flying forward moves ground objects toward image bottom,
    // flying left moves them toward image right). The angle counts CLOCKWISE
    // seen from above (opposite to FLU body yaw): 90 = image top toward the
    // RIGHT wing, 270 = toward the LEFT wing.
    const double xr = mount_cos_ * xc - mount_sin_ * yc;
    const double yr = mount_sin_ * xc + mount_cos_ * yc;
    // Nadir camera to body FLU: front = -image_down, left = -image_right, up = -optical.
    const std::array<double, 3> ray_body{-yr, -xr, -1.0};
    const std::array<double, 3> ray_world =
      rotateByQuaternion(pose.qw, pose.qx, pose.qy, pose.qz, ray_body);
    if (ray_world[2] > -0.05) {
      return std::nullopt;  // extreme tilt: the ray misses the ground ahead
    }
    const std::array<double, 3> offset_world = rotateByQuaternion(
      pose.qw, pose.qx, pose.qy, pose.qz,
      {camera_offset_x_, camera_offset_y_, camera_offset_z_});
    const double cam_x = pose.px + offset_world[0];
    const double cam_y = pose.py + offset_world[1];
    const double cam_z = pose.pz + offset_world[2];
    if (cam_z < 0.1) {
      return std::nullopt;  // on the ground or bad height: no meaningful intersection
    }
    const double t = cam_z / -ray_world[2];
    return std::make_pair(cam_x + t * ray_world[0], cam_y + t * ray_world[1]);
  }

  const TrackedDetection *findTrackByTrackId(std::uint64_t track_id) const
  {
    for (const TrackedDetection &track : tracked_detections_) {
      if (track.track_id == track_id) {
        return &track;
      }
    }
    return nullptr;
  }

  bool trackAliveById(const std::string &target_id) const
  {
    for (const TrackedDetection &track : tracked_detections_) {
      if (makeTargetId(track.label, track.track_id) == target_id) {
        return true;
      }
    }
    return false;
  }

  std::optional<std::size_t> findDetectionIndexById(
    const DetectionFrame &frame, const std::vector<std::uint64_t> &track_ids,
    const std::string &target_id) const
  {
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
      if (makeTargetId(frame.labels[index], track_ids[index]) == target_id) {
        return index;
      }
    }
    return std::nullopt;
  }

  std::optional<std::size_t> pickBestDetection(
    const DetectionFrame &frame, const std::vector<std::uint64_t> &track_ids) const
  {
    // Deterministic ranking: confirmed first, then longest streak, then score,
    // then oldest track. Never "closest to image center" -- that would switch
    // the selected target while the drone moves.
    std::optional<std::size_t> best;
    const TrackedDetection *best_track = nullptr;
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
      const TrackedDetection *track = findTrackByTrackId(track_ids[index]);
      if (track == nullptr) {
        continue;
      }
      if (!best) {
        best = index;
        best_track = track;
        continue;
      }
      if (track->confirmed != best_track->confirmed) {
        if (track->confirmed) {
          best = index;
          best_track = track;
        }
        continue;
      }
      if (track->hit_streak != best_track->hit_streak) {
        if (track->hit_streak > best_track->hit_streak) {
          best = index;
          best_track = track;
        }
        continue;
      }
      if (frame.detections[index].score != frame.detections[*best].score) {
        if (frame.detections[index].score > frame.detections[*best].score) {
          best = index;
          best_track = track;
        }
        continue;
      }
      if (track->track_id < best_track->track_id) {
        best = index;
        best_track = track;
      }
    }
    return best;
  }

  void updateServoSnapshot(const DetectionFrame &frame, const std::vector<std::uint64_t> &track_ids)
  {
    std::string requested_id;
    std::string control_locked_id;
    std::uint64_t session_epoch = 0;
    bool status_active = false;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      requested_id = servo_status_requested_id_;
      control_locked_id = servo_status_tracked_id_;
      session_epoch = servo_session_epoch_;
      status_active = servo_status_active_;
    }
    if (session_epoch != servo_seen_session_epoch_) {
      // active true->false edge: the servo session ended, release the self lock.
      servo_seen_session_epoch_ = session_epoch;
      if (!servo_sticky_target_id_.empty()) {
        RCLCPP_INFO(get_logger(), "ANIMAL_SERVO_EVENT type=session_end released_id=%s",
          servo_sticky_target_id_.c_str());
        servo_sticky_target_id_.clear();
      }
    }

    // The control side permanently locks the first fresh target_id of an action,
    // so its tracked id must win over the mission request and our own choice.
    // Gated on active: the transient_local latch retains the final status of a
    // finished action (active=false, tracked id still set), which must not
    // re-lock the target the session_end release just dropped.
    std::string desired;
    if (status_active) {
      desired = !control_locked_id.empty() ? control_locked_id : requested_id;
    }
    std::optional<std::size_t> chosen;
    std::string snapshot_target_id;
    if (!desired.empty()) {
      snapshot_target_id = desired;
      servo_sticky_target_id_ = desired;
      chosen = findDetectionIndexById(frame, track_ids, desired);
    } else if (!servo_sticky_target_id_.empty()) {
      chosen = findDetectionIndexById(frame, track_ids, servo_sticky_target_id_);
      if (chosen || trackAliveById(servo_sticky_target_id_)) {
        // Missing this frame but still tracked: publish valid=false and wait for
        // it instead of silently switching to another target.
        snapshot_target_id = servo_sticky_target_id_;
      } else {
        servo_sticky_target_id_.clear();
      }
    }
    if (!chosen && desired.empty() && servo_sticky_target_id_.empty()) {
      chosen = pickBestDetection(frame, track_ids);
      if (chosen) {
        servo_sticky_target_id_ = makeTargetId(frame.labels[*chosen], track_ids[*chosen]);
        snapshot_target_id = servo_sticky_target_id_;
        RCLCPP_INFO(get_logger(),
          "ANIMAL_SERVO_EVENT type=target_selected id=%s label=%s frame=%llu",
          servo_sticky_target_id_.c_str(), frame.labels[*chosen].c_str(),
          static_cast<unsigned long long>(frame.frame_id));
      }
    }

    ServoSnapshot snapshot;
    snapshot.source_valid = true;
    snapshot.target_id = snapshot_target_id;
    snapshot.image_width = static_cast<std::uint32_t>(frame.image_width);
    snapshot.image_height = static_cast<std::uint32_t>(frame.image_height);
    snapshot.frame_id = frame.frame_id;
    snapshot.produced_at = Clock::now();
    snapshot.stamp = now();
    if (chosen) {
      const Detection &detection = frame.detections[*chosen];
      const TrackedDetection *track = findTrackByTrackId(track_ids[*chosen]);
      const double half_width = static_cast<double>(frame.image_width) / 2.0;
      const double half_height = static_cast<double>(frame.image_height) / 2.0;
      snapshot.has_target = true;
      snapshot.confirmed = track != nullptr && track->confirmed;
      snapshot.label = frame.labels[*chosen];
      snapshot.confidence = static_cast<double>(detection.score);
      snapshot.center_x = static_cast<double>(detection.center.x);
      snapshot.center_y = static_cast<double>(detection.center.y);
      snapshot.error_x = half_width > 0.0 ?
        std::clamp((snapshot.center_x - half_width) / half_width, -1.0, 1.0) : 0.0;
      snapshot.error_y = half_height > 0.0 ?
        std::clamp((snapshot.center_y - half_height) / half_height, -1.0, 1.0) : 0.0;
      if (ground_projection_enabled_) {
        const PoseSample pose = currentPoseSample();
        if (pose.valid) {
          if (const auto ground = projectPixelToGround(
              pose, snapshot.center_x, snapshot.center_y))
          {
            snapshot.ground_valid = true;
            snapshot.ground_x = ground->first;
            snapshot.ground_y = ground->second;
          }
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      servo_snapshot_ = std::move(snapshot);
    }
    servo_snapshot_count_.fetch_add(1);
  }

  void handleServoStatus(const drone_msgs::msg::VisionServoStatus::SharedPtr status)
  {
    // Runs on the executor thread: copy only, no I/O and no logging here.
    std::lock_guard<std::mutex> lock(servo_mutex_);
    const bool was_active = servo_status_active_;
    servo_status_active_ = status->active;
    servo_status_state_ = status->state;
    servo_status_requested_id_ = status->requested_target_id;
    servo_status_tracked_id_ = status->tracked_target_id;
    if (was_active && !status->active) {
      ++servo_session_epoch_;
    }
  }

  void servoPublishLoop()
  {
    const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / servo_publish_rate_hz_));
    Clock::time_point next_wake = Clock::now();
    while (running_.load() && rclcpp::ok()) {
      next_wake += period;
      {
        std::unique_lock<std::mutex> lock(servo_wake_mutex_);
        servo_wake_.wait_until(lock, next_wake, [this] {
          return !running_.load() || !rclcpp::ok();
        });
      }
      if (!running_.load() || !rclcpp::ok()) {
        break;
      }
      const Clock::time_point now_at = Clock::now();
      if (now_at > next_wake + period) {
        next_wake = now_at;  // re-align after a long stall instead of bursting
      }
      try {
        publishServoTarget();
      } catch (const std::exception &error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Vision servo target publication failed: %s", error.what());
      }
    }
  }

  void publishServoTarget()
  {
    ServoSnapshot snapshot;
    bool status_active = false;
    std::string status_state;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      snapshot = servo_snapshot_;
      status_active = servo_status_active_;
      status_state = servo_status_state_;
    }
    const double age_s = snapshot.source_valid ?
      elapsedSeconds(snapshot.produced_at, Clock::now()) : -1.0;
    const bool fresh = snapshot.source_valid && age_s >= 0.0 && age_s <= servo_stale_timeout_s_;

    drone_msgs::msg::VisionServoTarget message;
    message.sequence = ++servo_sequence_;  // increments on no-target heartbeats too
    message.valid = fresh && snapshot.has_target;
    message.confirmed = message.valid && snapshot.confirmed;
    message.target_id = snapshot.target_id;
    // The control side checks isfinite(error) before checking valid, so every
    // numeric field must be a finite literal on the valid=false path. Stale
    // coordinates are structurally unreachable here: values are only copied out
    // of the snapshot when it is fresh.
    message.confidence = message.valid ? snapshot.confidence : 0.0;
    message.error_x = message.valid ? snapshot.error_x : 0.0;
    message.error_y = message.valid ? snapshot.error_y : 0.0;
    message.center_x = message.valid ? snapshot.center_x : 0.0;
    message.center_y = message.valid ? snapshot.center_y : 0.0;
    message.image_width = snapshot.source_valid ? snapshot.image_width :
      static_cast<std::uint32_t>(std::max(0, actual_width_.load()));
    message.image_height = snapshot.source_valid ? snapshot.image_height :
      static_cast<std::uint32_t>(std::max(0, actual_height_.load()));
    if (message.valid) {
      message.stamp = snapshot.stamp;
    } else {
      message.stamp = now();
    }
    servo_target_pub_->publish(message);

    const bool id_changed = message.target_id != last_pub_target_id_;
    if (message.valid != last_pub_valid_ || id_changed) {
      RCLCPP_INFO(get_logger(),
        "ANIMAL_SERVO_EVENT type=%s seq=%u id=%s valid=%d confirmed=%d err=(%.4f,%.4f) "
        "age_ms=%.1f active=%d state=%s",
        message.valid ? (id_changed ? "target_switch" : "target_acquired") : "target_lost",
        message.sequence, message.target_id.c_str(), message.valid ? 1 : 0,
        message.confirmed ? 1 : 0, message.error_x, message.error_y,
        age_s >= 0.0 ? age_s * 1000.0 : -1.0, status_active ? 1 : 0,
        status_state.empty() ? "none" : status_state.c_str());
    }
    last_pub_valid_ = message.valid;
    last_pub_target_id_ = message.target_id;

    servo_published_count_ += 1U;
    if (message.valid) {
      servo_valid_count_ += 1U;
    }
    if (snapshot.source_valid && !fresh) {
      servo_stale_count_ += 1U;
    }
    if (servo_log_period_s_ <= 0.0) {
      return;
    }
    const Clock::time_point log_now = Clock::now();
    if (last_servo_log_at_ != Clock::time_point{} &&
      elapsedSeconds(last_servo_log_at_, log_now) < servo_log_period_s_)
    {
      return;
    }
    const double interval = last_servo_log_at_ == Clock::time_point{} ?
      0.0 : elapsedSeconds(last_servo_log_at_, log_now);
    const std::uint64_t snapshots = servo_snapshot_count_.load();
    const std::uint64_t published_delta = servo_published_count_ - last_log_published_;
    RCLCPP_INFO(get_logger(),
      "ANIMAL_SERVO seq=%u frame=%llu id=%s label=%s valid=%d confirmed=%d conf=%.3f "
      "center=(%.1f,%.1f)px err=(%.4f,%.4f) ground_valid=%d ground=(%.2f,%.2f) "
      "img=%ux%u age_ms=%.1f active=%d state=%s",
      message.sequence, static_cast<unsigned long long>(snapshot.frame_id),
      message.target_id.c_str(), snapshot.label.c_str(), message.valid ? 1 : 0,
      message.confirmed ? 1 : 0, message.confidence, message.center_x, message.center_y,
      message.error_x, message.error_y, snapshot.ground_valid ? 1 : 0,
      snapshot.ground_x, snapshot.ground_y, message.image_width, message.image_height,
      age_s >= 0.0 ? age_s * 1000.0 : -1.0, status_active ? 1 : 0,
      status_state.empty() ? "none" : status_state.c_str());
    RCLCPP_INFO(get_logger(),
      "ANIMAL_SERVO_RATE pub_hz=%.2f snapshot_hz=%.2f valid_ratio=%.3f stale=%llu published=%llu",
      interval > 0.0 ? static_cast<double>(published_delta) / interval : 0.0,
      interval > 0.0 ? static_cast<double>(snapshots - last_log_snapshots_) / interval : 0.0,
      published_delta > 0U ?
        static_cast<double>(servo_valid_count_ - last_log_valid_) /
        static_cast<double>(published_delta) : 0.0,
      static_cast<unsigned long long>(servo_stale_count_),
      static_cast<unsigned long long>(servo_published_count_));
    last_servo_log_at_ = log_now;
    last_log_published_ = servo_published_count_;
    last_log_valid_ = servo_valid_count_;
    last_log_snapshots_ = snapshots;
  }

  std::string buildPipelineDescription() const
  {
    std::ostringstream pipeline;
    pipeline << "v4l2src device=" << camera_device_
             << " io-mode=2 do-timestamp=true ! "
             << "image/jpeg,width=" << camera_width_
             << ",height=" << camera_height_
             << ",framerate=" << camera_fps_ << "/1 ! "
             << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
             << "jpegparse ! mppjpegdec name=jpeg_decoder format=RGB dma-feature=false ! "
             << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
             << "appsink name=camera_sink emit-signals=false sync=false max-buffers=1 drop=true";
    return pipeline.str();
  }

  void startPipeline()
  {
    if (g_getenv("GST_MPP_NO_RGA") != nullptr) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring inherited GST_MPP_NO_RGA because MPP resize requires RGA");
      g_unsetenv("GST_MPP_NO_RGA");
    }
    int argc = 0;
    char **argv = nullptr;
    gst_init(&argc, &argv);
    pipeline_description_ = buildPipelineDescription();
    RCLCPP_INFO(get_logger(), "Starting internal GStreamer/MPP pipeline: %s",
      pipeline_description_.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
    if (pipeline_ == nullptr || error != nullptr) {
      const std::string message = error == nullptr ? "unknown parse error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("GStreamer pipeline parse failed: " + message);
    }

    GstElement *jpeg_decoder = gst_bin_get_by_name(GST_BIN(pipeline_), "jpeg_decoder");
    if (jpeg_decoder == nullptr) {
      throw std::runtime_error("GStreamer MPP decoder jpeg_decoder not found");
    }
    GObjectClass *decoder_class = G_OBJECT_GET_CLASS(jpeg_decoder);
    guint property_count = 0;
    GParamSpec **properties = g_object_class_list_properties(
      decoder_class, &property_count);
    std::ostringstream property_names;
    for (guint index = 0; index < property_count; ++index) {
      property_names << (index == 0 ? "" : ",") << properties[index]->name;
    }
    g_free(properties);
    RCLCPP_INFO(
      get_logger(), "MPP decoder type=%s properties=[%s]",
      G_OBJECT_TYPE_NAME(jpeg_decoder), property_names.str().c_str());
    const bool supports_resize =
      g_object_class_find_property(decoder_class, "width") != nullptr &&
      g_object_class_find_property(decoder_class, "height") != nullptr;
    if (!supports_resize) {
      gst_object_unref(jpeg_decoder);
      throw std::runtime_error(
              "MPP decoder started without width/height properties; "
              "refusing the known-bad 1280x720 fallback");
    }
    g_object_set(
      G_OBJECT(jpeg_decoder),
      "width", static_cast<guint>(decode_width_),
      "height", static_cast<guint>(decode_height_),
      nullptr);
    RCLCPP_INFO(
      get_logger(), "MPP decoder resize configured through GObject: %dx%d",
      decode_width_, decode_height_);
    gst_object_unref(jpeg_decoder);

    app_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "camera_sink");
    if (app_sink_ == nullptr || !GST_IS_APP_SINK(app_sink_)) {
      throw std::runtime_error("GStreamer appsink camera_sink not found");
    }
    const GstStateChangeReturn state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("GStreamer pipeline failed to enter PLAYING state");
    }

    GstSample *first_sample = gst_app_sink_try_pull_sample(
      GST_APP_SINK(app_sink_), 2 * GST_SECOND);
    if (first_sample == nullptr) {
      throw std::runtime_error(
              "GStreamer pipeline produced no camera frame within 2 seconds");
    }
    SamplePtr sample(first_sample, GstSampleDeleter{});
    GstVideoInfo info;
    if (!sampleVideoInfo(sample.get(), info)) {
      throw std::runtime_error("GStreamer first camera frame has invalid caps");
    }
    const int width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&info));
    const int height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info));
    if (width != decode_width_ || height != decode_height_) {
      throw std::runtime_error(
              "GStreamer first camera frame has unexpected size " +
              std::to_string(width) + "x" + std::to_string(height) +
              ", expected " + std::to_string(decode_width_) + "x" +
              std::to_string(decode_height_));
    }
    actual_width_.store(width);
    actual_height_.store(height);
    RCLCPP_INFO(get_logger(), "GStreamer first camera frame ready: RGB %dx%d", width, height);
  }

  void captureLoop()
  {
    while (running_.load() && rclcpp::ok()) {
      GstSample *raw_sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(app_sink_), 100 * GST_MSECOND);
      if (raw_sample == nullptr) {
        if (gst_app_sink_is_eos(GST_APP_SINK(app_sink_))) {
          RCLCPP_ERROR(get_logger(), "GStreamer camera pipeline reached EOS");
          rclcpp::shutdown();
          break;
        }
        reportBusError();
        continue;
      }
      SamplePtr sample(raw_sample, GstSampleDeleter{});
      GstVideoInfo info;
      if (!sampleVideoInfo(sample.get(), info)) {
        continue;
      }
      const auto now = Clock::now();
      if (last_capture_at_ != Clock::time_point{}) {
        const double seconds = elapsedSeconds(last_capture_at_, now);
        if (seconds > 0.0) {
          const double instant_fps = 1.0 / seconds;
          const double old_fps = capture_fps_.load();
          capture_fps_.store(old_fps <= 0.0 ? instant_fps : 0.9 * old_fps + 0.1 * instant_fps);
        }
      }
      last_capture_at_ = now;
      actual_width_.store(static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)));
      actual_height_.store(static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info)));
      received_count_.fetch_add(1);

      FrameTask task;
      task.frame_id = next_frame_id_.fetch_add(1);
      task.sample = std::move(sample);
      {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (task_queue_.size() >= kTaskQueueCapacity) {
          markDetectionFrameSkipped(task_queue_.front().frame_id);
          task_queue_.pop_front();
          dropped_count_.fetch_add(1);
        }
        task_queue_.push_back(std::move(task));
      }
      task_ready_.notify_one();
    }
  }

  bool sampleVideoInfo(GstSample *sample, GstVideoInfo &info)
  {
    GstCaps *caps = gst_sample_get_caps(sample);
    if (caps == nullptr || !gst_video_info_from_caps(&info, caps)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unable to read video caps from GStreamer sample");
      return false;
    }
    if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "MPP output format is %s, expected RGB",
        gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&info)));
      return false;
    }
    return true;
  }

  void reportBusError()
  {
    GstBus *bus = gst_element_get_bus(pipeline_);
    GstMessage *message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return;
    }
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError *error = nullptr;
      gchar *debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      RCLCPP_ERROR(get_logger(), "GStreamer error from %s: %s; debug=%s",
        GST_OBJECT_NAME(message->src), error == nullptr ? "unknown" : error->message,
        debug == nullptr ? "n/a" : debug);
      g_clear_error(&error);
      g_free(debug);
      running_.store(false);
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR(get_logger(), "GStreamer bus reported EOS");
      running_.store(false);
      rclcpp::shutdown();
    }
    gst_message_unref(message);
  }

  void workerLoop(std::size_t worker_index)
  {
    configureWorkerAffinity(worker_index);
    while (running_.load() && rclcpp::ok()) {
      FrameTask task;
      {
        std::unique_lock<std::mutex> lock(task_mutex_);
        task_ready_.wait(lock, [this] {
          return !task_queue_.empty() || !running_.load() || !rclcpp::ok();
        });
        if (!running_.load() || !rclcpp::ok()) {
          break;
        }
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      }
      processTask(std::move(task), worker_index);
    }
  }

  void configureWorkerAffinity(std::size_t worker_index)
  {
    if (!cpu_affinity_enabled_) {
      return;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(5 + static_cast<int>(worker_index), &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      RCLCPP_WARN(get_logger(), "Unable to bind RKNN worker %zu to CPU%zu",
        worker_index, worker_index + 5);
    }
  }

  void processTask(FrameTask task, std::size_t worker_index)
  {
    bool detection_frame_enqueued = false;
    try {
      GstVideoInfo info;
      if (!sampleVideoInfo(task.sample.get(), info)) {
        markDetectionFrameSkipped(task.frame_id);
        return;
      }
      GstVideoFrame frame;
      std::memset(&frame, 0, sizeof(frame));
      GstBuffer *buffer = gst_sample_get_buffer(task.sample.get());
      if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
        throw std::runtime_error("gst_video_frame_map failed");
      }
      struct FrameUnmapper
      {
        GstVideoFrame *frame;
        ~FrameUnmapper()
        {
          gst_video_frame_unmap(frame);
        }
      } frame_unmapper{&frame};
      cv::Mat rgb(
        static_cast<int>(GST_VIDEO_FRAME_HEIGHT(&frame)),
        static_cast<int>(GST_VIDEO_FRAME_WIDTH(&frame)), CV_8UC3,
        GST_VIDEO_FRAME_PLANE_DATA(&frame, 0),
        static_cast<std::size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0)));
      RknnYoloDetector &detector = *detectors_[worker_index];
      std::vector<Detection> detections = detector.infer(rgb);

      InferenceResult result;
      result.frame_id = task.frame_id;
      result.worker_index = worker_index;
      result.sample = std::move(task.sample);
      result.detections = std::move(detections);
      result.timing = detector.lastTiming();
      result.api_run_ms = detector.lastRknnRunMs();
      core_run_ms_[worker_index].store(result.api_run_ms);
      processed_count_.fetch_add(1);
      enqueueDetectionFrame(
        result.frame_id, rgb.cols, rgb.rows, result.detections, detector);
      detection_frame_enqueued = true;

      if (display_enabled_) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result.frame_id <= displayed_frame_id_ ||
          (latest_result_ && result.frame_id <= latest_result_->frame_id))
        {
          stale_result_count_.fetch_add(1);
          return;
        }
        if (latest_result_) {
          stale_result_count_.fetch_add(1);
        }
        latest_result_ = std::move(result);
        result_ready_.notify_one();
      } else {
        reportBaseline(result);
      }
    } catch (const std::exception &error) {
      if (!detection_frame_enqueued) {
        markDetectionFrameSkipped(task.frame_id);
      }
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Industrial camera RKNN inference failed: %s", error.what());
    }
  }

  void uiLoop()
  {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    while (running_.load() && rclcpp::ok()) {
      std::optional<InferenceResult> result;
      {
        std::unique_lock<std::mutex> lock(result_mutex_);
        result_ready_.wait_for(lock, std::chrono::milliseconds(10), [this] {
          return latest_result_.has_value() || !running_.load() || !rclcpp::ok();
        });
        const auto now = Clock::now();
        const bool interval_ready = display_fps_limit_ <= 0.0 ||
          last_display_at_ == Clock::time_point{} ||
          elapsedSeconds(last_display_at_, now) >= 1.0 / display_fps_limit_;
        if (latest_result_ && interval_ready) {
          result = std::move(latest_result_);
          latest_result_.reset();
          displayed_frame_id_ = result->frame_id;
        }
      }
      if (result) {
        displayResult(*result);
      }
      const int key = cv::waitKey(1) & 0xff;
      if (key == 's' && !last_display_image_.empty()) {
        const std::string path = "/tmp/industrial_animal_" +
          std::to_string(last_displayed_frame_) + ".jpg";
        cv::imwrite(path, last_display_image_);
        RCLCPP_INFO(get_logger(), "Saved annotated frame: %s", path.c_str());
      } else if (key == 'q' || key == 27) {
        rclcpp::shutdown();
        break;
      }
    }
    cv::destroyWindow(window_name_);
  }

  void displayResult(const InferenceResult &result)
  {
    GstVideoInfo info;
    if (!sampleVideoInfo(result.sample.get(), info)) {
      return;
    }
    GstVideoFrame frame;
    std::memset(&frame, 0, sizeof(frame));
    if (!gst_video_frame_map(
        &frame, &info, gst_sample_get_buffer(result.sample.get()), GST_MAP_READ))
    {
      return;
    }
    cv::Mat rgb(
      static_cast<int>(GST_VIDEO_FRAME_HEIGHT(&frame)),
      static_cast<int>(GST_VIDEO_FRAME_WIDTH(&frame)), CV_8UC3,
      GST_VIDEO_FRAME_PLANE_DATA(&frame, 0),
      static_cast<std::size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0)));
    cv::Mat display;
    cv::cvtColor(rgb, display, cv::COLOR_RGB2BGR);
    gst_video_frame_unmap(&frame);

    for (const Detection &detection : result.detections) {
      cv::rectangle(display, detection.box, cv::Scalar(0, 255, 0), 2);
      const std::string label = cv::format("%s %.2f",
        detectors_[result.worker_index]->classLabel(detection.class_id).c_str(), detection.score);
      cv::putText(display, label, cv::Point(detection.box.x, std::max(20, detection.box.y - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
    }

    const auto now = Clock::now();
    if (last_display_at_ != Clock::time_point{}) {
      const double seconds = elapsedSeconds(last_display_at_, now);
      if (seconds > 0.0) {
        const double current = 1.0 / seconds;
        display_fps_ = display_fps_ <= 0.0 ? current : 0.9 * display_fps_ + 0.1 * current;
      }
    }
    last_display_at_ = now;
    drawHud(display, result);
    cv::imshow(window_name_, display);
    last_display_image_ = display;
    last_displayed_frame_ = result.frame_id;
    reportBaseline(result);
  }

  void drawHud(cv::Mat &display, const InferenceResult &result)
  {
    CameraSettings camera_settings;
    {
      std::lock_guard<std::mutex> lock(camera_control_mutex_);
      camera_settings = camera_settings_;
    }
    std::array<double, kWorkerCount> core_ms{};
    double npu_capacity_fps = 0.0;
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      core_ms[index] = core_run_ms_[index].load();
      if (core_ms[index] > 0.0) {
        npu_capacity_fps += 1000.0 / core_ms[index];
      }
    }
    const cv::Scalar color(255, 0, 255);
    std::vector<std::string> lines{
      cv::format("Capture %.1f  Process %.1f  Display %.1f  NPU cap %.1f FPS",
        capture_fps_.load(), process_fps_.load(), display_fps_, npu_capacity_fps),
      cv::format("Queue drop %llu  Stale result %llu  Frame %llu  Worker C%zu",
        static_cast<unsigned long long>(dropped_count_.load()),
        static_cast<unsigned long long>(stale_result_count_.load()),
        static_cast<unsigned long long>(result.frame_id), result.worker_index),
      cv::format("Core0 %.2f  Core1 %.2f  Core2 %.2f ms",
        core_ms[0], core_ms[1], core_ms[2]),
      cv::format("Pre %.2f  Input %.2f  Run %.2f ms",
        result.timing.preprocess_ms, result.timing.input_prepare_ms,
        result.timing.rknn_run_ms),
      cv::format("Output %.2f  Post %.2f  Total %.2f ms",
        result.timing.output_get_ms,
        result.timing.postprocess_ms, result.timing.detector_total_ms),
      cv::format("Profile %s  request MJPEG %dx%d@%d",
        camera_profile_.c_str(), camera_width_, camera_height_, camera_fps_),
      cv::format("Actual RGB %dx%d %.1f FPS  exposure %d/%d  gain %d",
        actual_width_.load(), actual_height_.load(), capture_fps_.load(),
        camera_settings.exposure_auto, camera_settings.exposure_absolute, camera_settings.gain),
      cv::format("RKNN %dx%d in / %u out  zc=%s  pre=%s  detections %zu",
        model_input_width_, model_input_height_, model_output_count_, zero_copy_mode_.c_str(),
        detectors_[result.worker_index]->preprocessModeName(), result.detections.size()),
      cv::format("API %s  driver %s", api_version_.c_str(), driver_version_.c_str()),
      cv::format("3-context memory: weight %.1f MiB  internal %.1f MiB  DMA %.1f MiB",
        weight_mib_, internal_mib_, dma_mib_),
      "q/Esc: quit  s: save annotated frame"
    };
    int y = 22;
    for (const std::string &line : lines) {
      cv::putText(display, line, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX,
        0.43, color, 1, cv::LINE_AA);
      y += 21;
    }
  }

  void reportBaseline(const InferenceResult &result)
  {
    std::lock_guard<std::mutex> lock(report_mutex_);
    const auto now = Clock::now();
    const double report_seconds = elapsedSeconds(last_report_at_, now);
    if (report_seconds < 1.0) {
      return;
    }
    const std::uint64_t processed = processed_count_.load();
    const double process_fps = static_cast<double>(processed - last_report_processed_) /
      report_seconds;
    process_fps_.store(process_fps);
    std::array<double, kWorkerCount> core_ms{};
    double npu_capacity_fps = 0.0;
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      core_ms[index] = core_run_ms_[index].load();
      if (core_ms[index] > 0.0) {
        npu_capacity_fps += 1000.0 / core_ms[index];
      }
    }
    RCLCPP_INFO(get_logger(),
      "ANIMAL_RKNN_BASELINE profile=%s request=MJPEG_%dx%d@%d actual=RGB_%dx%d "
      "frames=%llu received=%llu queue_drop=%llu stale_result=%llu capture_fps=%.2f "
      "process_fps=%.2f display_fps=%.2f npu_capacity_fps=%.2f "
      "core_ms=[%.2f,%.2f,%.2f] pre_ms=%.2f input_ms=%.2f run_ms=%.2f "
      "output_ms=%.2f post_ms=%.2f total_ms=%.2f latest_frame=%llu worker=%zu detections=%zu",
      camera_profile_.c_str(), camera_width_, camera_height_, camera_fps_,
      actual_width_.load(), actual_height_.load(),
      static_cast<unsigned long long>(processed),
      static_cast<unsigned long long>(received_count_.load()),
      static_cast<unsigned long long>(dropped_count_.load()),
      static_cast<unsigned long long>(stale_result_count_.load()),
      capture_fps_.load(), process_fps, display_fps_, npu_capacity_fps,
      core_ms[0], core_ms[1], core_ms[2], result.timing.preprocess_ms,
      result.timing.input_prepare_ms, result.timing.rknn_run_ms,
      result.timing.output_get_ms, result.timing.postprocess_ms,
      result.timing.detector_total_ms,
      static_cast<unsigned long long>(result.frame_id), result.worker_index,
      result.detections.size());
    last_report_processed_ = processed;
    last_report_at_ = now;
  }

  std::array<std::unique_ptr<RknnYoloDetector>, kWorkerCount> detectors_;
  std::array<std::thread, kWorkerCount> workers_;
  std::thread capture_thread_;
  std::thread ui_thread_;
  std::thread detection_publish_thread_;
  std::thread servo_publish_thread_;
  GstElement *pipeline_ = nullptr;
  GstElement *app_sink_ = nullptr;
  std::mutex task_mutex_;
  std::condition_variable task_ready_;
  std::deque<FrameTask> task_queue_;
  std::mutex result_mutex_;
  std::condition_variable result_ready_;
  std::optional<InferenceResult> latest_result_;
  std::mutex report_mutex_;
  std::mutex camera_control_mutex_;
  std::mutex detection_result_mutex_;
  std::condition_variable detection_result_ready_;
  std::map<std::uint64_t, DetectionFrame> pending_detection_frames_;
  std::set<std::uint64_t> skipped_detection_frames_;
  std::vector<TrackedDetection> tracked_detections_;
  // servo_mutex_ is a leaf lock guarding the snapshot and the status cache.
  // Never acquire it while holding task_mutex_ or detection_result_mutex_.
  std::mutex servo_mutex_;
  ServoSnapshot servo_snapshot_;
  bool servo_status_active_ = false;
  std::string servo_status_state_;
  std::string servo_status_requested_id_;
  std::string servo_status_tracked_id_;
  std::uint64_t servo_session_epoch_ = 0;
  std::mutex servo_wake_mutex_;
  std::condition_variable servo_wake_;
  std::atomic<std::uint64_t> servo_snapshot_count_{0};
  // Owned by the detection publish thread only.
  std::string servo_sticky_target_id_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  // Local pose cache, guarded by servo_mutex_ (leaf lock; copy-only accesses).
  bool pose_received_ = false;
  double pose_px_ = 0.0;
  double pose_py_ = 0.0;
  double pose_pz_ = 0.0;
  double pose_qw_ = 1.0;
  double pose_qx_ = 0.0;
  double pose_qy_ = 0.0;
  double pose_qz_ = 0.0;
  Clock::time_point pose_received_at_{};
  bool ground_projection_enabled_ = true;
  std::string local_pose_topic_ = "/mavros/local_position/pose";
  double camera_fx_ = 914.0;
  double camera_fy_ = 914.0;
  double camera_cx_ = 640.0;
  double camera_cy_ = 360.0;
  int camera_mount_yaw_deg_ = 0;
  double camera_offset_x_ = 0.0;
  double camera_offset_y_ = -0.08;
  double camera_offset_z_ = 0.08;
  double pose_stale_s_ = 0.3;
  double mount_cos_ = 1.0;
  double mount_sin_ = 0.0;
  std::uint64_t servo_seen_session_epoch_ = 0;
  // Owned by the servo publish thread only.
  std::uint32_t servo_sequence_ = 0;
  bool last_pub_valid_ = false;
  std::string last_pub_target_id_;
  std::uint64_t servo_published_count_ = 0;
  std::uint64_t servo_valid_count_ = 0;
  std::uint64_t servo_stale_count_ = 0;
  Clock::time_point last_servo_log_at_{};
  std::uint64_t last_log_published_ = 0;
  std::uint64_t last_log_valid_ = 0;
  std::uint64_t last_log_snapshots_ = 0;
  rclcpp::Publisher<drone_msgs::msg::VisionServoTarget>::SharedPtr servo_target_pub_;
  rclcpp::Subscription<drone_msgs::msg::VisionServoStatus>::SharedPtr servo_status_sub_;
  rclcpp::Subscription<drone_msgs::msg::IndustrialCameraParams>::SharedPtr camera_params_sub_;
  std::atomic<bool> running_{true};
  std::atomic<std::uint64_t> next_frame_id_{1};
  std::atomic<std::uint64_t> received_count_{0};
  std::atomic<std::uint64_t> dropped_count_{0};
  std::atomic<std::uint64_t> stale_result_count_{0};
  std::atomic<std::uint64_t> processed_count_{0};
  std::array<std::atomic<double>, kWorkerCount> core_run_ms_{};
  std::atomic<double> capture_fps_{0.0};
  std::atomic<double> process_fps_{0.0};
  std::atomic<int> actual_width_{0};
  std::atomic<int> actual_height_{0};
  std::uint64_t next_detection_frame_id_ = 1;
  std::uint64_t next_track_id_ = 1;
  Clock::time_point started_at_;
  Clock::time_point last_capture_at_{};
  Clock::time_point last_report_at_;
  Clock::time_point last_display_at_{};
  std::uint64_t last_report_processed_ = 0;
  std::uint64_t displayed_frame_id_ = 0;
  std::uint64_t last_displayed_frame_ = 0;
  double display_fps_ = 0.0;
  double weight_mib_ = 0.0;
  double internal_mib_ = 0.0;
  double dma_mib_ = 0.0;
  cv::Mat last_display_image_;
  std::string camera_profile_;
  std::string camera_device_;
  std::string model_path_;
  std::string pipeline_description_;
  std::string api_version_;
  std::string driver_version_;
  std::string zero_copy_mode_;
  std::string preprocess_mode_;
  std::string servo_target_topic_ = "/vision/servo/target";
  std::string servo_status_topic_ = "/control/vision_servo/status";
  double servo_publish_rate_hz_ = 25.0;
  double servo_stale_timeout_s_ = 0.25;
  int servo_confirm_min_hits_ = 5;
  double servo_confirm_min_score_ = 0.55;
  double servo_confirm_max_center_jump_ = 0.12;
  double servo_confirm_max_area_ratio_ = 1.8;
  double servo_log_period_s_ = 1.0;
  int camera_width_ = 1280;
  int camera_height_ = 720;
  int camera_fps_ = 120;
  int decode_width_ = 640;
  int decode_height_ = 360;
  CameraSettings camera_settings_;
  int model_input_width_ = 0;
  int model_input_height_ = 0;
  std::uint32_t model_output_count_ = 0;
  bool display_enabled_ = true;
  bool enable_zero_copy_ = true;
  bool enable_rga_preprocess_ = true;
  bool cpu_affinity_enabled_ = true;
  double display_fps_limit_ = 60.0;
  float confidence_threshold_ = 0.5F;
  float nms_threshold_ = 0.45F;
  float track_iou_threshold_ = 0.3F;
  double track_max_missed_s_ = 1.5;
  const std::string window_name_ = "Industrial Camera Animal RKNN";
};
}  // namespace

namespace drone_perception
{
std::shared_ptr<rclcpp::Node> makeIndustrialAnimalVisionNode()
{
  return std::make_shared<IndustrialAnimalVisionNode>();
}
}  // namespace drone_perception

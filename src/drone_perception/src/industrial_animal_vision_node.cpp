#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <cstdlib>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <linux/videodev2.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_map.hpp>
#include <sched.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drone_msgs/msg/animal_detection.hpp"
#include "drone_msgs/msg/animal_detections.hpp"
#include "drone_msgs/msg/industrial_camera_control_capabilities.hpp"
#include "drone_msgs/msg/industrial_camera_control_command.hpp"
#include "drone_msgs/msg/industrial_camera_control_descriptor.hpp"
#include "drone_msgs/msg/industrial_camera_control_state.hpp"
#include "drone_perception/industrial_animal_vision_node.hpp"
#include "drone_perception/rknn_yolo_detector.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::size_t kWorkerCount = 3;
constexpr std::size_t kTaskQueueCapacity = 3;

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
    project_default_settings_ = camera_settings_;
    loadProjectDefaultSettings(project_default_settings_);
    CameraSettings saved_settings = camera_settings_;
    if (loadSavedCameraSettings(saved_settings)) {
      camera_settings_ = saved_settings;
      loaded_saved_settings_ = true;
    }
    validateParameters();
    configureProcessAffinity();
    configureCameraControls();
    startPipeline();
    initializeDetectors();
    detections_pub_ = create_publisher<drone_msgs::msg::AnimalDetections>(
      detections_topic_, rclcpp::QoS(10).reliable());
    const auto control_qos = rclcpp::QoS(1).reliable().transient_local();
    camera_capabilities_pub_ = create_publisher<drone_msgs::msg::IndustrialCameraControlCapabilities>(
      "/industrial_camera/control/capabilities", control_qos);
    camera_state_pub_ = create_publisher<drone_msgs::msg::IndustrialCameraControlState>(
      "/industrial_camera/control/state", control_qos);
    camera_command_sub_ = create_subscription<drone_msgs::msg::IndustrialCameraControlCommand>(
      "/industrial_camera/control/command", rclcpp::QoS(10).reliable(),
      [this](const drone_msgs::msg::IndustrialCameraControlCommand::SharedPtr message) {
        handleCameraControlCommand(message);
      });
    publishCameraCapabilities();
    publishCameraState(0U, true, 0U, 0U, "camera controls ready");

    detection_publish_thread_ = std::thread(&IndustrialAnimalVisionNode::detectionPublishLoop, this);
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
      "request=MJPEG %dx%d@%d display=%s preprocess=%s detections_topic=%s",
      camera_profile_.c_str(), model_path_.c_str(), camera_device_.c_str(),
      camera_width_, camera_height_, camera_fps_, display_enabled_ ? "on" : "off",
      preprocess_mode_.c_str(), detections_topic_.c_str());
  }

  ~IndustrialAnimalVisionNode() override
  {
    running_.store(false);
    task_ready_.notify_all();
    result_ready_.notify_all();
    detection_result_ready_.notify_all();

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
    std::vector<Detection> detections;
    std::vector<std::string> labels;
  };

  struct TrackedDetection
  {
    std::uint64_t track_id = 0;
    int class_id = -1;
    cv::Rect box;
    std::uint64_t last_seen_frame_id = 0;
  };

  struct TrackMatch
  {
    std::size_t detection_index = 0;
    std::size_t track_index = 0;
    float iou = 0.0F;
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
    declare_parameter<int>("exposure_auto", 1);
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
    declare_parameter<std::string>("detections_topic", "/animal_vision/detections");
    declare_parameter<double>("track_iou_threshold", 0.3);
    declare_parameter<int>("track_max_missed_frames", 15);
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
    detections_topic_ = get_parameter("detections_topic").as_string();
    track_iou_threshold_ = static_cast<float>(get_parameter("track_iou_threshold").as_double());
    track_max_missed_frames_ = static_cast<int>(
      get_parameter("track_max_missed_frames").as_int());
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
    if (detections_topic_.empty()) {
      throw std::invalid_argument("detections_topic must not be empty");
    }
    if (track_iou_threshold_ < 0.0F || track_iou_threshold_ > 1.0F) {
      throw std::invalid_argument("track_iou_threshold must be in [0, 1]");
    }
    if (track_max_missed_frames_ < 0) {
      throw std::invalid_argument("track_max_missed_frames must be >= 0");
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

  static std::string joinReasons(const std::vector<std::string> &reasons)
  {
    std::ostringstream stream;
    for (std::size_t index = 0; index < reasons.size(); ++index) {
      if (index > 0U) {
        stream << "; ";
      }
      stream << reasons[index];
    }
    return stream.str();
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
    std::uint64_t rejected_mask = 0U;
    applyCameraSettings(
      fd, camera_settings_, kAllCameraControls, camera_settings_, nullptr, &rejected_mask, nullptr, false);
    if (loaded_saved_settings_ && rejected_mask != 0U) {
      RCLCPP_WARN(
        get_logger(), "Saved camera settings are incompatible with the active camera; using project defaults");
      camera_settings_ = project_default_settings_;
      applyCameraSettings(
        fd, camera_settings_, kAllCameraControls, camera_settings_, nullptr, nullptr, nullptr, false);
    }
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

  std::string savedCameraSettingsPath() const
  {
    const char *home = std::getenv("HOME");
    const std::filesystem::path root = home == nullptr || *home == '\0' ? "." : home;
    return (root / ".ros" / "drone_perception" / "industrial_camera_saved.yaml").string();
  }

  bool loadCameraSettingsFile(const std::string &path, CameraSettings &settings, std::string &error) const
  {
    try {
      const rclcpp::ParameterMap parameters = rclcpp::parameter_map_from_yaml_file(path);
      std::uint64_t found_mask = 0U;
      for (const auto &node_parameters : parameters) {
        for (const rclcpp::Parameter &parameter : node_parameters.second) {
          for (const auto &definition : kCameraControlDefinitions) {
            if (parameter.get_name() == definition.name) {
              if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                error = parameter.get_name() + " must be an integer";
                return false;
              }
              setCameraSetting(
                settings, definition.mask, static_cast<int>(parameter.as_int()));
              found_mask |= definition.mask;
            }
          }
        }
      }
      if (found_mask != kAllCameraControls) {
        error = "missing one or more camera control fields";
        return false;
      }
      return true;
    } catch (const std::exception &exception) {
      error = exception.what();
      return false;
    }
  }

  bool loadProjectDefaultSettings(CameraSettings &settings)
  {
    try {
      const std::string path = (std::filesystem::path(
        ament_index_cpp::get_package_share_directory("drone_perception")) /
        "config" / "industrial_default.yaml").string();
      std::string error;
      if (loadCameraSettingsFile(path, settings, error)) {
        return true;
      }
      RCLCPP_WARN(get_logger(), "Unable to read project camera defaults from %s: %s",
        path.c_str(), error.c_str());
    } catch (const std::exception &exception) {
      RCLCPP_WARN(get_logger(), "Unable to locate project camera defaults: %s", exception.what());
    }
    return false;
  }

  bool loadSavedCameraSettings(CameraSettings &settings)
  {
    const std::string path = savedCameraSettingsPath();
    if (!std::filesystem::exists(path)) {
      return false;
    }
    std::string error;
    if (loadCameraSettingsFile(path, settings, error)) {
      RCLCPP_INFO(get_logger(), "Loaded saved industrial camera settings from %s", path.c_str());
      return true;
    }
    RCLCPP_WARN(get_logger(), "Ignoring saved industrial camera settings %s: %s",
      path.c_str(), error.c_str());
    return false;
  }

  bool saveCameraSettingsFile(const CameraSettings &settings, std::string &error) const
  {
    const std::filesystem::path path(savedCameraSettingsPath());
    std::error_code filesystem_error;
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
    if (filesystem_error) {
      error = "failed to create settings directory: " + filesystem_error.message();
      return false;
    }
    const std::filesystem::path temporary_path = path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
      error = "failed to open settings file for writing";
      return false;
    }
    stream << "industrial_animal_vision:\n  ros__parameters:\n";
    for (const auto &definition : kCameraControlDefinitions) {
      stream << "    " << definition.name << ": " <<
        getCameraSetting(settings, definition.mask) << "\n";
    }
    stream.close();
    if (!stream) {
      error = "failed while writing settings file";
      return false;
    }
    std::filesystem::rename(temporary_path, path, filesystem_error);
    if (filesystem_error) {
      std::error_code temporary_remove_error;
      std::filesystem::remove(temporary_path, temporary_remove_error);
      error = "failed to replace settings file: " + filesystem_error.message();
      return false;
    }
    return true;
  }

  static void setStateValues(
    drone_msgs::msg::IndustrialCameraControlState &message, const CameraSettings &settings)
  {
    message.exposure_auto = settings.exposure_auto;
    message.exposure_absolute = settings.exposure_absolute;
    message.exposure_auto_priority = settings.exposure_auto_priority;
    message.gain = settings.gain;
    message.brightness = settings.brightness;
    message.contrast = settings.contrast;
    message.saturation = settings.saturation;
    message.gamma = settings.gamma;
    message.sharpness = settings.sharpness;
    message.backlight_compensation = settings.backlight_compensation;
    message.white_balance_auto = settings.white_balance_auto;
    message.white_balance_temperature = settings.white_balance_temperature;
    message.power_line_frequency = settings.power_line_frequency;
    message.focus_auto = settings.focus_auto;
    message.focus_absolute = settings.focus_absolute;
    message.zoom_absolute = settings.zoom_absolute;
  }

  static CameraSettings settingsFromCommand(
    const drone_msgs::msg::IndustrialCameraControlCommand &command, const CameraSettings &base)
  {
    CameraSettings settings = base;
    settings.exposure_auto = command.exposure_auto;
    settings.exposure_absolute = command.exposure_absolute;
    settings.exposure_auto_priority = command.exposure_auto_priority;
    settings.gain = command.gain;
    settings.brightness = command.brightness;
    settings.contrast = command.contrast;
    settings.saturation = command.saturation;
    settings.gamma = command.gamma;
    settings.sharpness = command.sharpness;
    settings.backlight_compensation = command.backlight_compensation;
    settings.white_balance_auto = command.white_balance_auto;
    settings.white_balance_temperature = command.white_balance_temperature;
    settings.power_line_frequency = command.power_line_frequency;
    settings.focus_auto = command.focus_auto;
    settings.focus_absolute = command.focus_absolute;
    settings.zoom_absolute = command.zoom_absolute;
    return settings;
  }

  void publishCameraState(
    std::uint64_t request_id, bool success, std::uint64_t applied_mask,
    std::uint64_t rejected_mask, const std::string &message)
  {
    if (!camera_state_pub_) {
      return;
    }
    std::lock_guard<std::mutex> lock(camera_control_mutex_);
    CameraSettings settings = camera_settings_;
    std::uint64_t available_mask = 0U;
    std::uint64_t writable_mask = 0U;
    std::uint64_t active_mask = 0U;
    const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd >= 0) {
      readAllCameraControls(fd, settings, &available_mask, &writable_mask, &active_mask);
      close(fd);
      camera_settings_ = settings;
    }
    drone_msgs::msg::IndustrialCameraControlState state;
    state.stamp = now();
    state.request_id = request_id;
    state.success = success;
    state.applied_mask = applied_mask;
    state.rejected_mask = rejected_mask;
    state.available_mask = available_mask;
    state.writable_mask = writable_mask;
    state.active_mask = active_mask;
    state.message = message;
    setStateValues(state, camera_settings_);
    camera_state_pub_->publish(state);
  }

  static std::string menuLabel(const v4l2_querymenu &menu, __u32 type)
  {
    if (type == V4L2_CTRL_TYPE_INTEGER_MENU) {
      return std::to_string(menu.value);
    }
    return reinterpret_cast<const char *>(menu.name);
  }

  void publishCameraCapabilities()
  {
    if (!camera_capabilities_pub_) {
      return;
    }
    std::lock_guard<std::mutex> lock(camera_control_mutex_);
    drone_msgs::msg::IndustrialCameraControlCapabilities capabilities;
    capabilities.stamp = now();
    capabilities.camera_device = camera_device_;
    const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "Unable to publish camera capabilities: %s", strerror(errno));
      camera_capabilities_pub_->publish(capabilities);
      return;
    }
    for (const auto &definition : kCameraControlDefinitions) {
      drone_msgs::msg::IndustrialCameraControlDescriptor descriptor;
      descriptor.name = definition.name;
      descriptor.update_mask = definition.mask;
      v4l2_queryctrl query{};
      if (!queryCameraControl(fd, definition, query)) {
        capabilities.controls.push_back(descriptor);
        continue;
      }
      descriptor.available = true;
      descriptor.writable = (query.flags & V4L2_CTRL_FLAG_READ_ONLY) == 0;
      descriptor.active = (query.flags & V4L2_CTRL_FLAG_INACTIVE) == 0;
      descriptor.minimum = query.minimum;
      descriptor.maximum = query.maximum;
      descriptor.step = std::max(1, query.step);
      descriptor.default_value = query.default_value;
      v4l2_control current{};
      current.id = definition.id;
      if (ioctl(fd, VIDIOC_G_CTRL, &current) == 0) {
        descriptor.current_value = current.value;
      }
      if (query.type == V4L2_CTRL_TYPE_BOOLEAN) {
        descriptor.control_type = drone_msgs::msg::IndustrialCameraControlDescriptor::CONTROL_TYPE_BOOLEAN;
      } else if (query.type == V4L2_CTRL_TYPE_MENU || query.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
        descriptor.control_type = drone_msgs::msg::IndustrialCameraControlDescriptor::CONTROL_TYPE_MENU;
        for (int value = query.minimum; value <= query.maximum; ++value) {
          v4l2_querymenu menu{};
          menu.id = definition.id;
          menu.index = static_cast<std::uint32_t>(value);
          if (ioctl(fd, VIDIOC_QUERYMENU, &menu) == 0) {
            descriptor.menu_values.push_back(value);
            descriptor.menu_labels.push_back(menuLabel(menu, query.type));
          }
        }
      } else {
        descriptor.control_type = drone_msgs::msg::IndustrialCameraControlDescriptor::CONTROL_TYPE_INTEGER;
      }
      capabilities.controls.push_back(std::move(descriptor));
    }
    close(fd);
    camera_capabilities_pub_->publish(capabilities);
  }

  void handleCameraControlCommand(
    const drone_msgs::msg::IndustrialCameraControlCommand::SharedPtr command)
  {
    std::uint64_t applied_mask = 0U;
    std::uint64_t rejected_mask = 0U;
    bool success = false;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(camera_control_mutex_);
      if (command->command == drone_msgs::msg::IndustrialCameraControlCommand::COMMAND_SAVE_CURRENT) {
        const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
        if (fd >= 0) {
          readAllCameraControls(fd, camera_settings_);
          close(fd);
          success = saveCameraSettingsFile(camera_settings_, message);
          if (success) {
            message = "current camera settings saved";
          }
        } else {
          message = std::string("failed to open camera controls: ") + strerror(errno);
        }
      } else if (command->command ==
        drone_msgs::msg::IndustrialCameraControlCommand::COMMAND_RESTORE_PROJECT_DEFAULTS)
      {
        const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
          message = std::string("failed to open camera controls: ") + strerror(errno);
        } else {
          std::vector<std::string> reasons;
          applyCameraSettings(fd, project_default_settings_, kAllCameraControls, camera_settings_,
            &applied_mask, &rejected_mask, &reasons, false);
          close(fd);
          if (rejected_mask == 0U) {
            success = saveCameraSettingsFile(project_default_settings_, message);
            if (success) {
              message = "project defaults restored and saved";
            }
          } else {
            message = joinReasons(reasons);
          }
        }
      } else if (command->command == drone_msgs::msg::IndustrialCameraControlCommand::COMMAND_APPLY) {
        const std::uint64_t update_mask = command->update_mask & kAllCameraControls;
        rejected_mask = command->update_mask & ~kAllCameraControls;
        if (update_mask == 0U) {
          success = rejected_mask == 0U;
          message = success ? "no camera controls requested" : "command contains unsupported mask bits";
        } else {
          const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
          if (fd < 0) {
            rejected_mask |= update_mask;
            message = std::string("failed to open camera controls: ") + strerror(errno);
          } else {
            std::vector<std::string> reasons;
            const CameraSettings requested = settingsFromCommand(*command, camera_settings_);
            applyCameraSettings(fd, requested, update_mask, camera_settings_,
              &applied_mask, &rejected_mask, &reasons);
            close(fd);
            message = reasons.empty() ? "camera controls applied" : joinReasons(reasons);
          }
          success = rejected_mask == 0U;
        }
      } else {
        rejected_mask = command->update_mask;
        message = "unknown camera command";
      }
    }
    publishCameraCapabilities();
    publishCameraState(command->request_id, success, applied_mask, rejected_mask, message);
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
    frame.detections = detections;
    frame.labels.reserve(detections.size());
    for (const Detection &detection : detections) {
      frame.labels.push_back(detector.classLabel(detection.class_id));
    }

    {
      std::lock_guard<std::mutex> lock(detection_result_mutex_);
      pending_detection_frames_.emplace(frame_id, std::move(frame));
    }
    detection_result_ready_.notify_one();
  }

  void markDetectionFrameSkipped(std::uint64_t frame_id)
  {
    {
      std::lock_guard<std::mutex> lock(detection_result_mutex_);
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
        detection_result_ready_.wait(lock, [this] {
          return pending_detection_frames_.count(next_detection_frame_id_) > 0U ||
                 skipped_detection_frames_.count(next_detection_frame_id_) > 0U ||
                 !running_.load() || !rclcpp::ok();
        });

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
        publishDetectionFrame(*frame);
      } catch (const std::exception &error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Animal vision detection publication failed: %s", error.what());
      }
    }
  }

  void publishDetectionFrame(const DetectionFrame &frame)
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

    std::vector<std::uint64_t> track_ids(frame.detections.size(), 0);
    std::vector<bool> detection_matched(frame.detections.size(), false);
    std::vector<bool> track_matched(tracked_detections_.size(), false);
    for (const TrackMatch &candidate : candidates) {
      if (detection_matched[candidate.detection_index] || track_matched[candidate.track_index]) {
        continue;
      }
      TrackedDetection &track = tracked_detections_[candidate.track_index];
      track.box = frame.detections[candidate.detection_index].box;
      track.last_seen_frame_id = frame.frame_id;
      track_ids[candidate.detection_index] = track.track_id;
      detection_matched[candidate.detection_index] = true;
      track_matched[candidate.track_index] = true;
    }

    for (std::size_t detection_index = 0; detection_index < frame.detections.size(); ++detection_index) {
      if (detection_matched[detection_index]) {
        continue;
      }
      if (next_track_id_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::runtime_error("animal vision track_id space exhausted");
      }
      const std::uint64_t track_id = next_track_id_++;
      tracked_detections_.push_back(TrackedDetection{
        track_id,
        frame.detections[detection_index].class_id,
        frame.detections[detection_index].box,
        frame.frame_id});
      track_ids[detection_index] = track_id;
    }

    tracked_detections_.erase(
      std::remove_if(
        tracked_detections_.begin(), tracked_detections_.end(),
        [this, &frame](const TrackedDetection &track) {
          return frame.frame_id - track.last_seen_frame_id >
                 static_cast<std::uint64_t>(track_max_missed_frames_);
        }),
      tracked_detections_.end());

    drone_msgs::msg::AnimalDetections message;
    message.stamp = now();
    message.frame_seq = frame.frame_id;
    message.image_width = static_cast<std::uint32_t>(frame.image_width);
    message.image_height = static_cast<std::uint32_t>(frame.image_height);
    message.targets.reserve(frame.detections.size());
    const int image_center_x = frame.image_width / 2;
    const int image_center_y = frame.image_height / 2;
    const double half_width = static_cast<double>(frame.image_width) / 2.0;
    const double half_height = static_cast<double>(frame.image_height) / 2.0;
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
      const Detection &detection = frame.detections[index];
      drone_msgs::msg::AnimalDetection target;
      target.label = frame.labels[index];
      target.track_id = track_ids[index];
      target.score = detection.score;
      target.cx = detection.center.x;
      target.cy = detection.center.y;
      target.err_x = detection.center.x - image_center_x;
      target.err_y = detection.center.y - image_center_y;
      target.norm_x = half_width > 0.0 ? static_cast<double>(target.err_x) / half_width : 0.0;
      target.norm_y = half_height > 0.0 ? static_cast<double>(target.err_y) / half_height : 0.0;
      target.x1 = detection.box.x;
      target.y1 = detection.box.y;
      target.x2 = detection.box.x + detection.box.width;
      target.y2 = detection.box.y + detection.box.height;
      target.bbox_w = detection.box.width;
      target.bbox_h = detection.box.height;
      target.bbox_area = detection.box.area();
      message.targets.push_back(std::move(target));
    }
    message.target_count = static_cast<std::uint32_t>(message.targets.size());
    detections_pub_->publish(message);
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
  rclcpp::Publisher<drone_msgs::msg::AnimalDetections>::SharedPtr detections_pub_;
  rclcpp::Publisher<drone_msgs::msg::IndustrialCameraControlCapabilities>::SharedPtr
    camera_capabilities_pub_;
  rclcpp::Publisher<drone_msgs::msg::IndustrialCameraControlState>::SharedPtr camera_state_pub_;
  rclcpp::Subscription<drone_msgs::msg::IndustrialCameraControlCommand>::SharedPtr
    camera_command_sub_;
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
  std::string detections_topic_ = "/animal_vision/detections";
  int camera_width_ = 1280;
  int camera_height_ = 720;
  int camera_fps_ = 120;
  int decode_width_ = 640;
  int decode_height_ = 360;
  CameraSettings camera_settings_;
  CameraSettings project_default_settings_;
  bool loaded_saved_settings_ = false;
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
  int track_max_missed_frames_ = 15;
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

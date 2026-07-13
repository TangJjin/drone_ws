#include "drone_perception/qr_vision_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if DRONE_PERCEPTION_HAS_RKNN
#include <cstdlib>
#endif

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <zbar.h>

namespace
{
using SteadyClock = std::chrono::steady_clock;
static constexpr int kBpuInputWidthPx = 640;
static constexpr int kBpuInputHeightPx = 640;
static constexpr int kDefaultSampleRadiusPx = 10;
static constexpr int kDefaultShelfCodeStableFrames = 3;
static constexpr int kDefaultShelfCodeLostToleranceFrames = 2;
static constexpr int kDefaultBarcodeCaptureJpegQuality = 90;
static constexpr float kDefaultOcrYoloPaddingRatio = 0.15F;
static constexpr float kDefaultPackageCapturePaddingRatio = 0.05F;
static constexpr double kDefaultCaptureCollectionDurationS = 2.0;
static constexpr double kDefaultPackageCaptureDeadzoneRatio = 0.25;
static constexpr double kDefaultPackageCaptureMinScore = 0.50;
static constexpr const char *kDefaultBarcodeCaptureTopic = "/drone/image";
static constexpr const char *kDefaultHoverActiveTopic = "/mission/hover_active";
static constexpr const char *kJpegImageFormat = "jpeg";
static constexpr const char *kMissingCaptureField = "NAN";
static constexpr std::size_t kCameraExposureControlIndex = 0U;
static constexpr std::size_t kCameraGainControlIndex = 1U;
static constexpr std::size_t kCameraWhiteBalanceControlIndex = 2U;
static constexpr std::size_t kCameraAutoExposureControlIndex = 0U;
static constexpr std::size_t kCameraAutoWhiteBalanceControlIndex = 1U;
static constexpr std::size_t kCameraAutoExposurePriorityControlIndex = 2U;
static constexpr int kCameraControlsPanelWidthPx = 720;
static constexpr int kCameraControlsPanelHeightPx = 360;
static constexpr int kCameraControlsSendDebounceMs = 120;
// 路径 B / BPU 共用类别 ID
static constexpr int32_t kYoloClassQrcode = 0;
static constexpr int32_t kYoloClassPackage = 1;
static constexpr int32_t kYoloClassShelfTag = 2;
static constexpr float kVisualCodeRoiPaddingXRatio = 0.15F;
static constexpr float kVisualCodeRoiPaddingYRatio = 0.15F;
// ZBar 三阶段预处理（BPU / RKNN 共用）
static constexpr int kQrAdaptiveThresholdBlockSizePx = 31;
static constexpr double kQrAdaptiveThresholdOffset = 5.0;
#if DRONE_PERCEPTION_HAS_BPU
static constexpr int kQrPreprocessPreviewMaxSizePx = 180;
#endif
static constexpr std::size_t kBpuInputYSize =
    static_cast<std::size_t>(kBpuInputWidthPx) *
    static_cast<std::size_t>(kBpuInputHeightPx);
static constexpr std::size_t kBpuInputUvSize = kBpuInputYSize / 4;
static constexpr std::size_t kBpuInputNv12Size = kBpuInputYSize + kBpuInputUvSize * 2;

double elapsedMs(
    const SteadyClock::time_point &start,
    const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

bool isValidCapturePrefixedCode(
    const std::string &code,
    const std::string &prefix)
{
  if (code.rfind(prefix, 0U) != 0U || code.size() <= prefix.size()) {
    return false;
  }

  bool has_payload = false;

  for (std::size_t i = prefix.size(); i < code.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(code[i]);

    if (std::isalnum(value) != 0 || code[i] == '_') {
      has_payload = true;
      continue;
    }

    return false;
  }

  return has_payload;
}

bool isValidShelfCaptureCode(const std::string &code)
{
  if (code.size() < 5U) {
    return false;
  }

  if (code[0] != 'A' && code[0] != 'B') {
    return false;
  }

  const std::size_t first_dash = code.find('-');
  const std::size_t second_dash = code.find(
      '-',
      first_dash == std::string::npos ? 0U : first_dash + 1U);

  if (first_dash != 1U || second_dash == std::string::npos) {
    return false;
  }

  if (code.find('-', second_dash + 1U) != std::string::npos) {
    return false;
  }

  const std::string middle = code.substr(first_dash + 1U, second_dash - first_dash - 1U);
  const std::string tail = code.substr(second_dash + 1U);

  if (middle.empty() || tail.empty()) {
    return false;
  }

  const auto is_digit = [](char value) {
    return std::isdigit(static_cast<unsigned char>(value)) != 0;
  };

  return std::all_of(middle.begin(), middle.end(), is_digit) &&
      std::all_of(tail.begin(), tail.end(), is_digit);
}

// ZBar 解析辅助（BPU / RKNN 共用）
struct ParsedVisualCode
{
  std::string category;
  std::string code;
};

struct VisualCodeScanStats
{
  int symbol_count{0};
  int accepted_count{0};
};

void configureVisualCodeScanner(
    zbar::ImageScanner &scanner)
{
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

std::string trimAndUppercase(const std::string &text)
{
  const auto begin = std::find_if_not(
      text.begin(),
      text.end(),
      [](unsigned char value) {
        return std::isspace(value) != 0;
      });

  const auto end = std::find_if_not(
      text.rbegin(),
      text.rend(),
      [](unsigned char value) {
        return std::isspace(value) != 0;
      }).base();

  if (begin >= end) {
    return {};
  }

  std::string normalized(begin, end);
  std::transform(
      normalized.begin(),
      normalized.end(),
      normalized.begin(),
      [](unsigned char value) {
        return static_cast<char>(std::toupper(value));
      });

  return normalized;
}

bool isValidShortPrefixedCode(
    const std::string &code,
    const std::string &prefix)
{
  if (code.rfind(prefix, 0U) != 0U || code.size() <= prefix.size()) {
    return false;
  }

  bool has_payload = false;

  for (std::size_t i = prefix.size(); i < code.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(code[i]);

    if (std::isalnum(value) != 0 || code[i] == '_') {
      has_payload = true;
      continue;
    }

    return false;
  }

  return has_payload;
}

void appendParsedVisualCode(
    const std::string &code,
    std::vector<ParsedVisualCode> &parsed_codes)
{
  if (isValidShortPrefixedCode(code, "SKU-")) {
    parsed_codes.push_back({"sku", code});
    return;
  }

  if (isValidShortPrefixedCode(code, "PKG-")) {
    parsed_codes.push_back({"pkg", code});
    return;
  }
}

std::vector<ParsedVisualCode> parseVisualCodes(const std::string &code)
{
  std::vector<ParsedVisualCode> parsed_codes;
  std::size_t begin = 0U;
  std::size_t token_count = 0U;

  while (begin <= code.size()) {
    const std::size_t separator = code.find('|', begin);
    const std::size_t end =
        separator == std::string::npos ? code.size() : separator;
    const std::string token = trimAndUppercase(code.substr(begin, end - begin));

    if (!token.empty()) {
      ++token_count;
      appendParsedVisualCode(token, parsed_codes);
    }

    if (separator == std::string::npos) {
      break;
    }

    begin = separator + 1U;
  }

  const bool has_sku = std::any_of(
      parsed_codes.begin(),
      parsed_codes.end(),
      [](const ParsedVisualCode &parsed_code) {
        return parsed_code.category == "sku";
      });
  const bool has_pkg = std::any_of(
      parsed_codes.begin(),
      parsed_codes.end(),
      [](const ParsedVisualCode &parsed_code) {
        return parsed_code.category == "pkg";
      });

  if (token_count != 2U || parsed_codes.size() != 2U || !has_sku || !has_pkg) {
    return {};
  }

  return parsed_codes;
}

std::string formatParsedVisualCodeCategories(
    const std::vector<ParsedVisualCode> &parsed_codes)
{
  if (parsed_codes.empty()) {
    return "ignored";
  }

  std::string categories;

  for (const ParsedVisualCode &parsed_code : parsed_codes) {
    if (!categories.empty()) {
      categories += ",";
    }

    categories += parsed_code.category;
  }

  return categories;
}
}  // namespace

QrVisionNode::QrVisionNode()
    : Node("qr_vision_node")
{
  declareParameters();
  barcode_capture_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
      barcode_capture_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  initializeRknnDetector();
  initializeBpuDetector();
  // 本阶段不接 OCR：即使编译了 BPU 也默认关；initialize 内部会再检查 enable_bpu_ocr_
  initializeBpuOcrPipeline();
  initializeSubscriptions();

  if (debug_view_)
  {
    try
    {
      cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      debug_window_created_ = true;
    }
    catch (const cv::Exception &e)
    {
      debug_view_ = false;
      RCLCPP_WARN(get_logger(), "Cannot open debug window: %s", e.what());
    }
  }

  RCLCPP_INFO(
      get_logger(),
      "QR D435i video stream ready. mode=%s qr_decode=true yolo_rknn=%s yolo_bpu=%s "
      "ocr_rec_bpu=%s require_depth=%s require_camera_info=%s color=%s depth=%s "
      "rgbd=%s camera_info=%s rknn_model=%s",
      use_rgbd_ ? "rgbd" : (require_depth_ ? "synced" : "color"),
      enable_rknn_ ? "true" : "false",
      enable_bpu_ ? "true" : "false",
      enable_bpu_ocr_ ? "true" : "false",
      require_depth_ ? "true" : "false",
      require_camera_info_ ? "true" : "false",
      color_topic_.c_str(),
      depth_topic_.c_str(),
      rgbd_topic_.c_str(),
      camera_info_topic_.c_str(),
      rknn_model_path_.c_str());
}

QrVisionNode::~QrVisionNode()
{
  camera_controls_timer_.reset();
#if DRONE_PERCEPTION_HAS_RKNN
  stopRknnWorkers();
#endif

  if (debug_window_created_)
  {
    cv::destroyWindow(window_name_);
  }
}

void QrVisionNode::declareParameters()
{
  color_topic_ = this->declare_parameter<std::string>(
      "color_topic", "/camera/camera/color/image_raw");
  depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
  camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/color/camera_info");
  // require_depth=true：硬 ApproximateTime 同步 color+depth（BPU/兼容路径）
  // RKNN 三 worker 默认 color 驱动 + depth 软对齐（最新 depth，见 kDepthSoftMatchNs）
  require_depth_ = this->declare_parameter<bool>("require_depth", false);
  require_camera_info_ = this->declare_parameter<bool>("require_camera_info", true);
  rgbd_topic_ = this->declare_parameter<std::string>(
      "rgbd_topic", "/camera/camera/rgbd");
  window_name_ = this->declare_parameter<std::string>(
      "window_name", "QR RKNN Detection");
  debug_view_ = this->declare_parameter<bool>("debug_view", true);
  qr_preprocess_enabled_ = this->declare_parameter<bool>(
      "qr_preprocess_enabled",
      true);
  camera_controls_enabled_ = this->declare_parameter<bool>(
      "camera_controls_enabled",
      false);
  camera_param_node_ = this->declare_parameter<std::string>(
      "camera_param_node",
      "/camera/camera");
  (void)this->declare_parameter<std::string>(
      "camera_controls_window_name",
      "D435i Controls");
  camera_controls_update_period_ms_ = this->declare_parameter<int>(
      "camera_controls_update_period_ms",
      200);
  barcode_capture_topic_ = this->declare_parameter<std::string>(
      "barcode_capture_topic",
      kDefaultBarcodeCaptureTopic);
  hover_active_topic_ = this->declare_parameter<std::string>(
      "hover_active_topic",
      kDefaultHoverActiveTopic);
  use_rgbd_ = this->declare_parameter<bool>("use_rgbd", false);
  log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 2000);
  sample_radius_px_ = this->declare_parameter<int>(
      "sample_radius_px",
      kDefaultSampleRadiusPx);
  shelf_code_stable_frames_ = this->declare_parameter<int>(
      "shelf_code_stable_frames",
      kDefaultShelfCodeStableFrames);
  shelf_code_lost_tolerance_frames_ = this->declare_parameter<int>(
      "shelf_code_lost_tolerance_frames",
      kDefaultShelfCodeLostToleranceFrames);
  barcode_capture_jpeg_quality_ = this->declare_parameter<int>(
      "barcode_capture_jpeg_quality",
      kDefaultBarcodeCaptureJpegQuality);

  // OrangePi：优先 RKNN 路径 B；OCR 本阶段默认关闭
  enable_rknn_ = this->declare_parameter<bool>(
      "enable_rknn",
      DRONE_PERCEPTION_HAS_RKNN != 0);
  enable_bpu_ = this->declare_parameter<bool>(
      "enable_bpu",
      DRONE_PERCEPTION_HAS_BPU != 0 && DRONE_PERCEPTION_HAS_RKNN == 0);
  enable_bpu_ocr_ = this->declare_parameter<bool>(
      "enable_bpu_ocr",
      false);
  bpu_model_path_ = this->declare_parameter<std::string>(
      "bpu_model_path",
      "/home/sunrise/drone_ws/src/drone_perception/new_640x640_nv12.bin");
  {
    std::string default_rknn_model;
    try {
      default_rknn_model =
          ament_index_cpp::get_package_share_directory("drone_perception") +
          "/models/package_qrcode_shelf_tag_fp16.rknn";
    } catch (const std::exception &) {
      default_rknn_model = "package_qrcode_shelf_tag_fp16.rknn";
    }
    rknn_model_path_ = this->declare_parameter<std::string>(
        "rknn_model_path",
        default_rknn_model);
  }
  ocr_rec_model_path_ = this->declare_parameter<std::string>(
      "ocr_rec_model_path",
      "/home/sunrise/drone_ws/src/drone_perception/en_PP-OCRv3_rec_48x320_rgb.bin");
  ocr_yolo_padding_ratio_ = static_cast<float>(this->declare_parameter<double>(
      "ocr_yolo_padding_ratio",
      kDefaultOcrYoloPaddingRatio));
  package_capture_padding_ratio_ = static_cast<float>(this->declare_parameter<double>(
      "package_capture_padding_ratio",
      kDefaultPackageCapturePaddingRatio));
  capture_collection_duration_s_ = this->declare_parameter<double>(
      "capture_collection_duration_s",
      kDefaultCaptureCollectionDurationS);
  package_capture_deadzone_ratio_ = this->declare_parameter<double>(
      "package_capture_deadzone_ratio",
      kDefaultPackageCaptureDeadzoneRatio);
  package_capture_min_score_ = this->declare_parameter<double>(
      "package_capture_min_score",
      kDefaultPackageCaptureMinScore);

  if (log_throttle_ms_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "log_throttle_ms must be greater than 0, reset to 2000");
    log_throttle_ms_ = 2000;
  }

  if (camera_controls_update_period_ms_ < 30)
  {
    RCLCPP_WARN(
        get_logger(),
        "camera_controls_update_period_ms must be at least 30, reset to 200");
    camera_controls_update_period_ms_ = 200;
  }

  if (sample_radius_px_ < 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "sample_radius_px must be greater than or equal to 0, reset to %d",
        kDefaultSampleRadiusPx);
    sample_radius_px_ = kDefaultSampleRadiusPx;
  }

  if (shelf_code_stable_frames_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "shelf_code_stable_frames must be greater than 0, reset to %d",
        kDefaultShelfCodeStableFrames);
    shelf_code_stable_frames_ = kDefaultShelfCodeStableFrames;
  }

  if (shelf_code_lost_tolerance_frames_ < 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "shelf_code_lost_tolerance_frames must be greater than or equal to 0, reset to %d",
        kDefaultShelfCodeLostToleranceFrames);
    shelf_code_lost_tolerance_frames_ = kDefaultShelfCodeLostToleranceFrames;
  }

  if (ocr_yolo_padding_ratio_ < 0.0F)
  {
    RCLCPP_WARN(
        get_logger(),
        "ocr_yolo_padding_ratio must be greater than or equal to 0, reset to %.2f",
        kDefaultOcrYoloPaddingRatio);
    ocr_yolo_padding_ratio_ = kDefaultOcrYoloPaddingRatio;
  }

  if (package_capture_padding_ratio_ < 0.0F)
  {
    RCLCPP_WARN(
        get_logger(),
        "package_capture_padding_ratio must be greater than or equal to 0, reset to %.2f",
        kDefaultPackageCapturePaddingRatio);
    package_capture_padding_ratio_ = kDefaultPackageCapturePaddingRatio;
  }

  if (capture_collection_duration_s_ <= 0.0)
  {
    RCLCPP_WARN(
        get_logger(),
        "capture_collection_duration_s must be greater than 0, reset to %.2f",
        kDefaultCaptureCollectionDurationS);
    capture_collection_duration_s_ = kDefaultCaptureCollectionDurationS;
  }

  if (package_capture_deadzone_ratio_ <= 0.0 ||
      package_capture_deadzone_ratio_ > 0.5)
  {
    RCLCPP_WARN(
        get_logger(),
        "package_capture_deadzone_ratio must be in (0, 0.5], reset to %.2f",
        kDefaultPackageCaptureDeadzoneRatio);
    package_capture_deadzone_ratio_ = kDefaultPackageCaptureDeadzoneRatio;
  }

  if (package_capture_min_score_ < 0.0 ||
      package_capture_min_score_ > 1.0)
  {
    RCLCPP_WARN(
        get_logger(),
        "package_capture_min_score must be in [0, 1], reset to %.2f",
        kDefaultPackageCaptureMinScore);
    package_capture_min_score_ = kDefaultPackageCaptureMinScore;
  }

  if (barcode_capture_jpeg_quality_ < 1 || barcode_capture_jpeg_quality_ > 100)
  {
    RCLCPP_WARN(
        get_logger(),
        "barcode_capture_jpeg_quality must be in [1, 100], reset to %d",
        kDefaultBarcodeCaptureJpegQuality);
    barcode_capture_jpeg_quality_ = kDefaultBarcodeCaptureJpegQuality;
  }
}

void QrVisionNode::initializeRknnDetector()
{
#if DRONE_PERCEPTION_HAS_RKNN
  if (!enable_rknn_) {
    return;
  }

  if (rknn_model_path_.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "RKNN inference is enabled but rknn_model_path is empty");
    enable_rknn_ = false;
    return;
  }

  try {
    constexpr std::array<rknn_core_mask, kRknnWorkerCount> core_masks{
        RKNN_NPU_CORE_0, RKNN_NPU_CORE_1, RKNN_NPU_CORE_2};
    rknn_weight_mib_ = 0.0;
    rknn_internal_mib_ = 0.0;
    rknn_dma_mib_ = 0.0;
    for (std::size_t i = 0; i < kRknnWorkerCount; ++i) {
      rknn_detectors_[i] = std::make_unique<RknnYoloDetector>(
          rknn_model_path_, core_masks[i]);
      try {
        const rknn_mem_size memory = rknn_detectors_[i]->memorySize();
        rknn_weight_mib_ +=
            static_cast<double>(memory.total_weight_size) / (1024.0 * 1024.0);
        rknn_internal_mib_ +=
            static_cast<double>(memory.total_internal_size) / (1024.0 * 1024.0);
        rknn_dma_mib_ +=
            static_cast<double>(memory.total_dma_allocated_size) / (1024.0 * 1024.0);
      } catch (const std::exception &) {
      }
      rknn_core_run_ms_[i].store(0.0);
    }
    startRknnWorkers();
    rknn_consume_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&QrVisionNode::consumeRknnResults, this));
    RCLCPP_INFO(
        get_logger(),
        "RKNN path-B 3-worker pipeline ready. model=%s conf=%.2f "
        "workers=%zu queue_cap=%zu soft_depth_match_ms=%.0f "
        "mem weight=%.1fMiB internal=%.1fMiB dma=%.1fMiB",
        rknn_model_path_.c_str(),
        RknnYoloDetector::kConfThresh,
        kRknnWorkerCount,
        kRknnTaskQueueCapacity,
        static_cast<double>(kDepthSoftMatchNs) / 1.0e6,
        rknn_weight_mib_,
        rknn_internal_mib_,
        rknn_dma_mib_);
  } catch (const std::exception &e) {
    stopRknnWorkers();
    for (auto &detector : rknn_detectors_) {
      detector.reset();
    }
    enable_rknn_ = false;
    RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize RKNN 3-worker pipeline: %s",
        e.what());
  }
#else
  if (enable_rknn_) {
    RCLCPP_WARN(
        get_logger(),
        "RKNN inference requested but this build has no RKNN runtime "
        "(set RKNN_INCLUDE/RKNN_LIB when building)");
    enable_rknn_ = false;
  }
#endif
}

#if DRONE_PERCEPTION_HAS_RKNN
void QrVisionNode::startRknnWorkers()
{
  stopRknnWorkers();
  rknn_running_.store(true);
  for (std::size_t i = 0; i < kRknnWorkerCount; ++i) {
    rknn_workers_[i] = std::thread(&QrVisionNode::rknnWorkerLoop, this, i);
  }
}

void QrVisionNode::stopRknnWorkers()
{
  rknn_running_.store(false);
  rknn_task_cv_.notify_all();
  if (rknn_consume_timer_) {
    rknn_consume_timer_->cancel();
    rknn_consume_timer_.reset();
  }
  for (std::thread &worker : rknn_workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
  {
    std::lock_guard<std::mutex> lock(rknn_task_mutex_);
    rknn_task_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(rknn_result_mutex_);
    rknn_latest_result_.reset();
  }
}

bool QrVisionNode::depthSoftMatchesColor(
    const sensor_msgs::msg::Image &color,
    const sensor_msgs::msg::Image &depth)
{
  const std::int64_t color_ns = rclcpp::Time(color.header.stamp).nanoseconds();
  const std::int64_t depth_ns = rclcpp::Time(depth.header.stamp).nanoseconds();
  return std::llabs(color_ns - depth_ns) <= kDepthSoftMatchNs;
}

void QrVisionNode::handleDepthFrameSoft(
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
{
  std::lock_guard<std::mutex> lock(rknn_depth_mutex_);
  rknn_latest_depth_ = depth_msg;
}

void QrVisionNode::enqueueRknnColorFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &color_msg)
{
  if (!enable_rknn_ || !rknn_running_.load()) {
    return;
  }
  if (require_camera_info_ && !has_camera_info_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "waiting for camera_info on %s before RKNN enqueue",
        camera_info_topic_.c_str());
    return;
  }

  RknnFrameTask task;
  task.frame_id = rknn_next_frame_id_++;
  task.color = color_msg;
  {
    std::lock_guard<std::mutex> lock(rknn_depth_mutex_);
    if (rknn_latest_depth_ &&
        depthSoftMatchesColor(*color_msg, *rknn_latest_depth_))
    {
      task.depth = rknn_latest_depth_;
    } else if (rknn_latest_depth_) {
      // 软对齐：超出时间窗仍可附上最新 depth 做中心深度参考（与探针 latest_depth 类似）
      task.depth = rknn_latest_depth_;
    }
  }

  {
    std::lock_guard<std::mutex> lock(rknn_task_mutex_);
    if (rknn_task_queue_.size() >= kRknnTaskQueueCapacity) {
      rknn_task_queue_.pop_front();
      rknn_dropped_count_.fetch_add(1);
    }
    rknn_task_queue_.push_back(std::move(task));
  }
  rknn_task_cv_.notify_one();
}

void QrVisionNode::rknnWorkerLoop(std::size_t worker_index)
{
  while (rknn_running_.load() && rclcpp::ok()) {
    RknnFrameTask task;
    {
      std::unique_lock<std::mutex> lock(rknn_task_mutex_);
      rknn_task_cv_.wait(lock, [this] {
        return !rknn_task_queue_.empty() || !rknn_running_.load() || !rclcpp::ok();
      });
      if (!rknn_running_.load() || !rclcpp::ok()) {
        break;
      }
      task = std::move(rknn_task_queue_.front());
      rknn_task_queue_.pop_front();
    }

    try {
      RknnYoloDetector &detector = *rknn_detectors_[worker_index];
      const auto color_bridge = cv_bridge::toCvShare(task.color, "bgr8");
      cv::Mat rgb;
      cv::cvtColor(color_bridge->image, rgb, cv::COLOR_BGR2RGB);
      const auto wall_t0 = SteadyClock::now();
      std::vector<Detection> detections = detector.infer(rgb);
      const auto wall_t1 = SteadyClock::now();
      const double wall_ms = elapsedMs(wall_t0, wall_t1);

      // 深度：在所有预测框中取分数最高框，采样其几何中心深度（软对齐 depth）
      DepthSampleResult center_depth{};
      int best_det_index = -1;
      float best_det_score = -1.0F;
      for (std::size_t di = 0; di < detections.size(); ++di) {
        if (detections[di].score > best_det_score) {
          best_det_score = detections[di].score;
          best_det_index = static_cast<int>(di);
        }
      }
      if (task.depth && best_det_index >= 0) {
        try {
          const auto depth_bridge = cv_bridge::toCvShare(task.depth);
          if (depth_bridge && !depth_bridge->image.empty() &&
              depth_bridge->image.cols == color_bridge->image.cols &&
              depth_bridge->image.rows == color_bridge->image.rows)
          {
            const Detection &best = detections[static_cast<std::size_t>(best_det_index)];
            center_depth = depth_processor_.sampleAt(
                depth_bridge->image,
                best.center.x,
                best.center.y,
                sample_radius_px_);
            if (center_depth.has_valid_depth) {
              detections[static_cast<std::size_t>(best_det_index)].has_depth = true;
              detections[static_cast<std::size_t>(best_det_index)].depth_m =
                  center_depth.depth_m;
            }
          }
        } catch (const cv_bridge::Exception &) {
        }
      }

      RknnInferResult result;
      result.frame_id = task.frame_id;
      result.worker_index = worker_index;
      result.detections = std::move(detections);
      result.timing = detector.lastTiming();
      result.center_depth = center_depth;
      result.bgr_image = color_bridge->image.clone();
      rknn_core_run_ms_[worker_index].store(result.timing.rknn_run_ms);
      rknn_processed_count_.fetch_add(1);
      (void)wall_ms;

      {
        std::lock_guard<std::mutex> lock(rknn_result_mutex_);
        if (rknn_latest_result_ &&
            result.frame_id <= rknn_latest_result_->frame_id)
        {
          rknn_stale_count_.fetch_add(1);
          continue;
        }
        rknn_latest_result_ = std::move(result);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "RKNN worker %zu failed: %s",
          worker_index,
          e.what());
    }
  }
}

void QrVisionNode::consumeRknnResults()
{
  if (!enable_rknn_) {
    return;
  }

  std::optional<RknnInferResult> result;
  {
    std::lock_guard<std::mutex> lock(rknn_result_mutex_);
    if (!rknn_latest_result_) {
      return;
    }
    if (rknn_latest_result_->frame_id <= rknn_consumed_frame_id_) {
      return;
    }
    result = std::move(rknn_latest_result_);
    rknn_latest_result_.reset();
  }

  rknn_consumed_frame_id_ = result->frame_id;
  last_rknn_detections_ = result->detections;
  last_rknn_timing_ = result->timing;
  last_rknn_center_depth_ = result->center_depth;
  last_input_mode_ = "rknn3w";
  last_callback_ms_ = result->timing.detector_total_ms;
  updateFps();

  // ZBar 只在最新结果上跑一次，避免 3 worker 重复解码
  updateVisualCodeStability(
      decodeVisualCodesFromRknnDetections(result->bgr_image, last_rknn_detections_));
  publishBarcodeCapture(result->bgr_image);

  if (debug_view_) {
    displayDebugFrame(result->bgr_image, last_rknn_center_depth_);
  }

  RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "RKNN 3-worker mode=rknn3w fps=%.1f total_ms=%.2f run_ms=%.2f prepare_ms=%.2f "
      "det=%zu drop=%llu stale=%llu processed=%llu worker=%zu depth=%s",
      smoothed_fps_,
      last_rknn_timing_.detector_total_ms,
      last_rknn_timing_.rknn_run_ms,
      last_rknn_timing_.input_prepare_ms,
      last_rknn_detections_.size(),
      static_cast<unsigned long long>(rknn_dropped_count_.load()),
      static_cast<unsigned long long>(rknn_stale_count_.load()),
      static_cast<unsigned long long>(rknn_processed_count_.load()),
      result->worker_index,
      last_rknn_center_depth_.has_valid_depth ? "soft_ok" : "n/a");
}
#endif

void QrVisionNode::initializeBpuDetector()
{
#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_bpu_) {
    return;
  }

  if (bpu_model_path_.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "BPU inference is enabled but bpu_model_path is empty");
    enable_bpu_ = false;
    return;
  }

  try {
    bpu_detector_ = std::make_unique<BpuYoloDetector>(bpu_model_path_);

    RCLCPP_INFO(
        get_logger(),
        "BPU detector initialized. model=%s",
        bpu_model_path_.c_str());
  } catch (const std::exception &e) {
    bpu_detector_.reset();
    enable_bpu_ = false;

    RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize BPU detector: %s",
        e.what());
  }
#else
  if (enable_bpu_) {
    RCLCPP_WARN(
        get_logger(),
        "BPU inference requested but this build has no RDK BPU SDK");
    enable_bpu_ = false;
  }
#endif
}

void QrVisionNode::initializeBpuOcrPipeline()
{
#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_bpu_ocr_) {
    return;
  }

  try {
    BpuOcrConfig config{};
    config.rec_model_path = ocr_rec_model_path_;
    config.enable_detection_model = false;
    bpu_ocr_pipeline_ = std::make_unique<BpuOcrPipeline>(config);

    RCLCPP_INFO(
        get_logger(),
        "BPU OCR rec pipeline initialized. rec_model=%s yolo_padding_ratio=%.2f",
        config.rec_model_path.empty() ? "<auto>" : config.rec_model_path.c_str(),
        ocr_yolo_padding_ratio_);
  } catch (const std::exception &e) {
    bpu_ocr_pipeline_.reset();
    enable_bpu_ocr_ = false;

    RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize BPU OCR pipeline: %s",
        e.what());
  }
#else
  if (enable_bpu_ocr_) {
    RCLCPP_WARN(
        get_logger(),
        "BPU OCR requested but this build has no RDK BPU SDK");
  }

  enable_bpu_ocr_ = false;
#endif
}

bool QrVisionNode::prepareBpuInput(const cv::Mat &color_image)
{
  if (color_image.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "BPU input image is empty");

    return false;
  }

  if (color_image.channels() != 3) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "BPU input expects BGR image with 3 channels, got %d",
        color_image.channels());

    return false;
  }

  try {
    const float scale = std::min(
        static_cast<float>(kBpuInputWidthPx) / static_cast<float>(color_image.cols),
        static_cast<float>(kBpuInputHeightPx) / static_cast<float>(color_image.rows));
    const int resized_width = std::clamp(
        static_cast<int>(std::round(static_cast<float>(color_image.cols) * scale)),
        1,
        kBpuInputWidthPx);
    const int resized_height = std::clamp(
        static_cast<int>(std::round(static_cast<float>(color_image.rows) * scale)),
        1,
        kBpuInputHeightPx);
    const int pad_x = (kBpuInputWidthPx - resized_width) / 2;
    const int pad_y = (kBpuInputHeightPx - resized_height) / 2;

    cv::Mat resized_bgr;
    cv::resize(
        color_image,
        resized_bgr,
        cv::Size(resized_width, resized_height),
        0.0,
        0.0,
        cv::INTER_LINEAR);

    cv::Mat letterbox_bgr(
        kBpuInputHeightPx,
        kBpuInputWidthPx,
        color_image.type(),
        cv::Scalar(114, 114, 114));
    resized_bgr.copyTo(letterbox_bgr(cv::Rect(
        pad_x,
        pad_y,
        resized_width,
        resized_height)));

    bpu_letterbox_.scale = scale;
    bpu_letterbox_.pad_x = pad_x;
    bpu_letterbox_.pad_y = pad_y;

    cv::Mat yuv_i420;
    cv::cvtColor(letterbox_bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    if (!yuv_i420.isContinuous()) {
      yuv_i420 = yuv_i420.clone();
    }

    bpu_input_nv12_.resize(kBpuInputNv12Size);

    const uint8_t *y_plane = yuv_i420.ptr<uint8_t>(0);
    const uint8_t *u_plane = y_plane + kBpuInputYSize;
    const uint8_t *v_plane = u_plane + kBpuInputUvSize;
    uint8_t *nv12_data = bpu_input_nv12_.data();

    std::memcpy(nv12_data, y_plane, kBpuInputYSize);

    uint8_t *uv_plane = nv12_data + kBpuInputYSize;

    for (std::size_t i = 0; i < kBpuInputUvSize; ++i) {
      uv_plane[i * 2] = u_plane[i];
      uv_plane[i * 2 + 1] = v_plane[i];
    }
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Failed to prepare BPU input: %s",
        e.what());

    return false;
  }

  return true;
}

void QrVisionNode::updateVisualCodeStability(
    const std::vector<DecodedVisualCode> &decoded_codes)
{
  updateVisualCodeCategoryStability(decoded_codes, "sku", sku_code_state_, "qr");
  updateVisualCodeCategoryStability(decoded_codes, "pkg", pkg_code_state_, "qr");
}

std::string QrVisionNode::composeCaptureBarcode(bool require_complete) const
{
  const std::string pkg_code =
      isValidCapturePrefixedCode(pkg_code_state_.stable_code, "PKG-")
          ? pkg_code_state_.stable_code
          : "";
  const std::string sku_code =
      isValidCapturePrefixedCode(sku_code_state_.stable_code, "SKU-")
          ? sku_code_state_.stable_code
          : "";
  const std::string shelf_code =
      isValidShelfCaptureCode(shelf_code_state_.stable_code)
          ? shelf_code_state_.stable_code
          : "";

  const bool complete_capture =
      !pkg_code.empty() && !sku_code.empty() && !shelf_code.empty();

  if (require_complete && !complete_capture) {
    return {};
  }

  if (!require_complete &&
      pkg_code.empty() && sku_code.empty() && shelf_code.empty()) {
    return {};
  }

  return pkg_code + "|" + sku_code + "|" + shelf_code;
}

std::string QrVisionNode::composeCaptureBarcodeWithNan(bool allow_all_nan) const
{
  const bool has_qr_group =
      isValidCapturePrefixedCode(pkg_code_state_.stable_code, "PKG-") &&
      isValidCapturePrefixedCode(sku_code_state_.stable_code, "SKU-");
  const bool has_shelf_code =
      isValidShelfCaptureCode(shelf_code_state_.stable_code);

  if (!allow_all_nan && !has_qr_group && !has_shelf_code) {
    return {};
  }

  const std::string pkg_code =
      has_qr_group ? pkg_code_state_.stable_code : kMissingCaptureField;
  const std::string sku_code =
      has_qr_group ? sku_code_state_.stable_code : kMissingCaptureField;
  const std::string shelf_code =
      has_shelf_code ? shelf_code_state_.stable_code : kMissingCaptureField;

  return pkg_code + "|" + sku_code + "|" + shelf_code;
}

void QrVisionNode::handleHoverActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool next_hover_active = msg->data;

  if (next_hover_active == hover_active_) {
    return;
  }

  if (hover_active_ && !next_hover_active && !full_capture_sent_in_hover_) {
#if DRONE_PERCEPTION_HAS_BPU
    if (!publishBestBufferedCapture()) {
      publishTextOnlyCapture(composeCaptureBarcodeWithNan(false));
    }
#else
    publishTextOnlyCapture(composeCaptureBarcodeWithNan(false));
#endif
  }

  hover_active_ = next_hover_active;

  if (hover_active_) {
    resetHoverCaptureState();
  } else {
#if DRONE_PERCEPTION_HAS_BPU
    resetCaptureCandidateState();
#endif
  }
}

void QrVisionNode::resetHoverCaptureState()
{
  full_capture_sent_in_hover_ = false;
  last_published_package_barcode_.clear();
#if DRONE_PERCEPTION_HAS_BPU
  resetCaptureCandidateState();
  hover_capture_start_time_ = this->now();
#endif
}

void QrVisionNode::publishBarcodeCapture(const cv::Mat &color_image)
{
  if (!hover_active_) {
    return;
  }

  const std::string barcode = composeCaptureBarcode(true);

  if (barcode.empty() || barcode == last_published_package_barcode_) {
    return;
  }

  (void)publishFullFrameCapture(barcode, color_image);
}

void QrVisionNode::publishTextOnlyCapture(const std::string &barcode)
{
  if (!barcode_capture_pub_ || barcode.empty()) {
    return;
  }

  drone_msgs::msg::BarcodeCapture msg;
  msg.stamp = this->now();
  msg.barcode = barcode;
  barcode_capture_pub_->publish(msg);

  RCLCPP_INFO(
      get_logger(),
      "published partial barcode capture barcode=%s",
      barcode.c_str());
}

bool QrVisionNode::publishFullFrameCapture(
    const std::string &barcode,
    const cv::Mat &color_image)
{
  if (!barcode_capture_pub_ || barcode.empty()) {
    return false;
  }

#if DRONE_PERCEPTION_HAS_BPU
  std::vector<uint8_t> jpeg_data;

  if (!encodeFrameCaptureJpeg(color_image, jpeg_data)) {
    return false;
  }

  drone_msgs::msg::BarcodeCapture msg;
  msg.stamp = this->now();
  msg.barcode = barcode;
  msg.image_format = kJpegImageFormat;
  msg.image_data = std::move(jpeg_data);

  barcode_capture_pub_->publish(msg);
  last_published_package_barcode_ = barcode;
  full_capture_sent_in_hover_ = true;

  RCLCPP_INFO(
      get_logger(),
      "published frame barcode capture barcode=%s bytes=%zu",
      barcode.c_str(),
      msg.image_data.size());
  return true;
#else
  (void)color_image;
  RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "full frame barcode capture requested but this build has no BPU support");
  return false;
#endif
}

void QrVisionNode::updateVisualCodeCategoryStability(
    const std::vector<DecodedVisualCode> &decoded_codes,
    const std::string &category,
    CodeStabilityState &state,
    const char *source_name)
{
  const auto selected_code = std::find_if(
      decoded_codes.begin(),
      decoded_codes.end(),
      [&category](const DecodedVisualCode &decoded_code) {
        return decoded_code.category == category;
      });

  if (selected_code == decoded_codes.end()) {
    ++state.lost_count;

    if (state.lost_count > shelf_code_lost_tolerance_frames_) {
      state.candidate_code.clear();
      state.candidate_symbol_type.clear();
      state.stable_code.clear();
      state.stable_symbol_type.clear();
      state.candidate_count = 0;
    }

    return;
  }

  state.lost_count = 0;

  if (selected_code->code == state.candidate_code) {
    ++state.candidate_count;
  } else {
    state.candidate_code = selected_code->code;
    state.candidate_symbol_type = selected_code->symbol_type;
    state.candidate_count = 1;
  }

  if (state.candidate_count < shelf_code_stable_frames_) {
    return;
  }

  state.stable_code = state.candidate_code;
  state.stable_symbol_type = state.candidate_symbol_type;

  RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "stable %s code category=%s symbol_type=%s code=%s stable_frames=%d",
      source_name,
      category.c_str(),
      state.stable_symbol_type.c_str(),
      state.stable_code.c_str(),
      state.candidate_count);
}

#if DRONE_PERCEPTION_HAS_BPU
std::vector<QrVisionNode::DecodedVisualCode> QrVisionNode::decodeVisualCodesFromDetections(
    const cv::Mat &color_image,
    const std::vector<BpuYoloDetection> &detections)
{
  std::vector<DecodedVisualCode> decoded_codes;
  debug_qr_preprocess_preview_.release();
  debug_qr_preprocess_mode_.clear();

  if (color_image.empty() || detections.empty()) {
    return decoded_codes;
  }

  zbar::ImageScanner scanner;
  configureVisualCodeScanner(scanner);

  const cv::Point2f image_center(
      static_cast<float>(color_image.cols) * 0.5F,
      static_cast<float>(color_image.rows) * 0.5F);

  for (const BpuYoloDetection &detection : detections) {
    if (detection.class_id != kYoloClassQrcode) {
      continue;
    }

    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        color_image.cols,
        color_image.rows);
    const float x_min = image_rect.x_min;
    const float y_min = image_rect.y_min;
    const float x_max = image_rect.x_max;
    const float y_max = image_rect.y_max;
    const float width = x_max - x_min;
    const float height = y_max - y_min;

    if (width <= 1.0F || height <= 1.0F) {
      continue;
    }

    const float pad_x = width * kVisualCodeRoiPaddingXRatio;
    const float pad_y = height * kVisualCodeRoiPaddingYRatio;
    const int roi_x_min = std::max(0, static_cast<int>(x_min - pad_x));
    const int roi_y_min = std::max(0, static_cast<int>(y_min - pad_y));
    const int roi_x_max = std::min(color_image.cols, static_cast<int>(x_max + pad_x));
    const int roi_y_max = std::min(color_image.rows, static_cast<int>(y_max + pad_y));
    const cv::Rect roi(
        roi_x_min,
        roi_y_min,
        roi_x_max - roi_x_min,
        roi_y_max - roi_y_min);

    if (roi.width <= 1 || roi.height <= 1) {
      continue;
    }

    cv::Mat raw_gray_roi;
    cv::cvtColor(color_image(roi), raw_gray_roi, cv::COLOR_BGR2GRAY);

    if (!raw_gray_roi.isContinuous()) {
      raw_gray_roi = raw_gray_roi.clone();
    }

    auto scan_gray_roi = [&](
        const cv::Mat &scan_roi,
        float coordinate_scale,
        const char *scan_mode) {
      cv::Mat continuous_roi = scan_roi;

      if (!continuous_roi.isContinuous()) {
        continuous_roi = continuous_roi.clone();
      }

      debug_qr_preprocess_preview_ = continuous_roi.clone();
      debug_qr_preprocess_mode_ = scan_mode;

      zbar::Image zbar_image(
          static_cast<unsigned int>(continuous_roi.cols),
          static_cast<unsigned int>(continuous_roi.rows),
          "Y800",
          continuous_roi.data,
          static_cast<unsigned long>(continuous_roi.total()));

      const int raw_count = scanner.scan(zbar_image);
      VisualCodeScanStats stats{};
      stats.symbol_count = raw_count;

      if (raw_count <= 0) {
        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar no code scan=%s roi=%dx%d at=%d,%d detection_score=%.3f",
            scan_mode,
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y,
            detection.score);
      }

      for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
        const std::string raw_code = symbol->get_data();
        const std::string code = trimAndUppercase(raw_code);
        const std::string symbol_type = symbol->get_type_name();
        const bool is_qr_code = symbol_type.find("QR") != std::string::npos;
        const std::vector<ParsedVisualCode> parsed_codes = parseVisualCodes(code);
        const std::string categories = formatParsedVisualCodeCategories(parsed_codes);

        // Capture raw zbar output for debug display (before mode/parse filter)
        debug_raw_symbol_ = code;
        debug_raw_symbol_type_ = symbol_type;

        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar raw qr scan=%s symbol_type=%s qr_match=%s raw=%s normalized=%s category=%s roi=%dx%d at=%d,%d",
            scan_mode,
            symbol_type.c_str(),
            is_qr_code ? "true" : "false",
            raw_code.c_str(),
            code.c_str(),
            categories.c_str(),
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y);

        if (!is_qr_code || parsed_codes.empty()) {
          continue;
        }

        cv::Point2f code_center(
            static_cast<float>(roi.x) + static_cast<float>(roi.width) * 0.5F,
            static_cast<float>(roi.y) + static_cast<float>(roi.height) * 0.5F);

        const int location_size = symbol->get_location_size();

        if (location_size > 0) {
          code_center = cv::Point2f(0.0F, 0.0F);

          for (int i = 0; i < location_size; ++i) {
            code_center.x += static_cast<float>(roi.x) +
                static_cast<float>(symbol->get_location_x(i)) / coordinate_scale;
            code_center.y += static_cast<float>(roi.y) +
                static_cast<float>(symbol->get_location_y(i)) / coordinate_scale;
          }

          code_center.x /= static_cast<float>(location_size);
          code_center.y /= static_cast<float>(location_size);
        }

        const float dx = code_center.x - image_center.x;
        const float dy = code_center.y - image_center.y;

        for (const ParsedVisualCode &parsed_code : parsed_codes) {
          decoded_codes.push_back({
              parsed_code.code,
              parsed_code.category,
              symbol_type,
              static_cast<double>(dx * dx + dy * dy)});
          ++stats.accepted_count;
        }
      }

      zbar_image.set_data(nullptr, 0U);

      return stats;
    };

    VisualCodeScanStats scan_stats = scan_gray_roi(raw_gray_roi, 1.0F, "raw_gray");

    if (scan_stats.accepted_count > 0 || !qr_preprocess_enabled_) {
      continue;
    }

    try {
      cv::Mat clahe_roi;
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(raw_gray_roi, clahe_roi);

      scan_stats = scan_gray_roi(clahe_roi, 1.0F, "clahe_gray");

      if (scan_stats.accepted_count > 0) {
        continue;
      }

      const int min_side_px = std::min(clahe_roi.cols, clahe_roi.rows);
      int adaptive_block_size_px = std::min(kQrAdaptiveThresholdBlockSizePx, min_side_px);

      if (adaptive_block_size_px % 2 == 0) {
        --adaptive_block_size_px;
      }

      if (adaptive_block_size_px >= 3) {
        cv::Mat adaptive_binary_roi;
        cv::adaptiveThreshold(
            clahe_roi,
            adaptive_binary_roi,
            255.0,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY,
            adaptive_block_size_px,
            kQrAdaptiveThresholdOffset);
        (void)scan_gray_roi(adaptive_binary_roi, 1.0F, "adaptive_binary");
      }
    } catch (const cv::Exception &e) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "QR preprocess failed: %s",
          e.what());
    }
  }

  std::sort(
      decoded_codes.begin(),
      decoded_codes.end(),
      [](const DecodedVisualCode &a, const DecodedVisualCode &b) {
        return a.center_distance_sq < b.center_distance_sq;
      });

  return decoded_codes;
}
#endif

#if DRONE_PERCEPTION_HAS_RKNN
std::vector<QrVisionNode::DecodedVisualCode>
QrVisionNode::decodeVisualCodesFromRknnDetections(
    const cv::Mat &color_image,
    const std::vector<Detection> &detections)
{
  std::vector<DecodedVisualCode> decoded_codes;
  debug_qr_preprocess_preview_.release();
  debug_qr_preprocess_mode_.clear();

  if (color_image.empty() || detections.empty()) {
    return decoded_codes;
  }

  zbar::ImageScanner scanner;
  configureVisualCodeScanner(scanner);

  const cv::Point2f image_center(
      static_cast<float>(color_image.cols) * 0.5F,
      static_cast<float>(color_image.rows) * 0.5F);

  for (const Detection &detection : detections) {
    if (detection.class_id != kYoloClassQrcode) {
      continue;
    }

    const cv::Rect &box = detection.box;
    if (box.width <= 1 || box.height <= 1) {
      continue;
    }

    const float pad_x = static_cast<float>(box.width) * kVisualCodeRoiPaddingXRatio;
    const float pad_y = static_cast<float>(box.height) * kVisualCodeRoiPaddingYRatio;
    const int roi_x_min = std::max(0, static_cast<int>(box.x - pad_x));
    const int roi_y_min = std::max(0, static_cast<int>(box.y - pad_y));
    const int roi_x_max = std::min(
        color_image.cols, static_cast<int>(box.x + box.width + pad_x));
    const int roi_y_max = std::min(
        color_image.rows, static_cast<int>(box.y + box.height + pad_y));
    const cv::Rect roi(
        roi_x_min,
        roi_y_min,
        std::max(1, roi_x_max - roi_x_min),
        std::max(1, roi_y_max - roi_y_min));

    if (roi.x + roi.width > color_image.cols ||
        roi.y + roi.height > color_image.rows)
    {
      continue;
    }

    cv::Mat raw_gray_roi;
    cv::cvtColor(color_image(roi), raw_gray_roi, cv::COLOR_BGR2GRAY);
    if (!raw_gray_roi.isContinuous()) {
      raw_gray_roi = raw_gray_roi.clone();
    }

    // 与 BPU 路径一致：raw_gray → clahe_gray → adaptive_binary
    auto scan_gray_roi = [&](
        const cv::Mat &scan_roi,
        float coordinate_scale,
        const char *scan_mode) {
      cv::Mat continuous_roi = scan_roi;
      if (!continuous_roi.isContinuous()) {
        continuous_roi = continuous_roi.clone();
      }

      debug_qr_preprocess_preview_ = continuous_roi.clone();
      debug_qr_preprocess_mode_ = scan_mode;

      zbar::Image zbar_image(
          static_cast<unsigned int>(continuous_roi.cols),
          static_cast<unsigned int>(continuous_roi.rows),
          "Y800",
          continuous_roi.data,
          static_cast<unsigned long>(continuous_roi.total()));

      const int raw_count = scanner.scan(zbar_image);
      VisualCodeScanStats stats{};
      stats.symbol_count = raw_count;

      if (raw_count <= 0) {
        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar no code scan=%s roi=%dx%d at=%d,%d detection_score=%.3f",
            scan_mode,
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y,
            detection.score);
      }

      for (auto symbol = zbar_image.symbol_begin();
           symbol != zbar_image.symbol_end();
           ++symbol)
      {
        const std::string raw_code = symbol->get_data();
        const std::string code = trimAndUppercase(raw_code);
        const std::string symbol_type = symbol->get_type_name();
        const bool is_qr_code = symbol_type.find("QR") != std::string::npos;
        const std::vector<ParsedVisualCode> parsed_codes = parseVisualCodes(code);

        debug_raw_symbol_ = code;
        debug_raw_symbol_type_ = symbol_type;

        if (!is_qr_code || parsed_codes.empty()) {
          continue;
        }

        cv::Point2f code_center(
            static_cast<float>(roi.x) + static_cast<float>(roi.width) * 0.5F,
            static_cast<float>(roi.y) + static_cast<float>(roi.height) * 0.5F);
        const int location_size = symbol->get_location_size();
        if (location_size > 0) {
          code_center = cv::Point2f(0.0F, 0.0F);
          for (int i = 0; i < location_size; ++i) {
            code_center.x += static_cast<float>(roi.x) +
                static_cast<float>(symbol->get_location_x(i)) / coordinate_scale;
            code_center.y += static_cast<float>(roi.y) +
                static_cast<float>(symbol->get_location_y(i)) / coordinate_scale;
          }
          code_center.x /= static_cast<float>(location_size);
          code_center.y /= static_cast<float>(location_size);
        }

        const float dx = code_center.x - image_center.x;
        const float dy = code_center.y - image_center.y;
        for (const ParsedVisualCode &parsed_code : parsed_codes) {
          decoded_codes.push_back({
              parsed_code.code,
              parsed_code.category,
              symbol_type,
              static_cast<double>(dx * dx + dy * dy)});
          ++stats.accepted_count;
        }
      }

      zbar_image.set_data(nullptr, 0U);
      return stats;
    };

    VisualCodeScanStats scan_stats = scan_gray_roi(raw_gray_roi, 1.0F, "raw_gray");
    if (scan_stats.accepted_count > 0 || !qr_preprocess_enabled_) {
      continue;
    }

    try {
      cv::Mat clahe_roi;
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(raw_gray_roi, clahe_roi);
      scan_stats = scan_gray_roi(clahe_roi, 1.0F, "clahe_gray");
      if (scan_stats.accepted_count > 0) {
        continue;
      }

      const int min_side_px = std::min(clahe_roi.cols, clahe_roi.rows);
      int adaptive_block_size_px =
          std::min(kQrAdaptiveThresholdBlockSizePx, min_side_px);
      if (adaptive_block_size_px % 2 == 0) {
        --adaptive_block_size_px;
      }
      if (adaptive_block_size_px >= 3) {
        cv::Mat adaptive_binary_roi;
        cv::adaptiveThreshold(
            clahe_roi,
            adaptive_binary_roi,
            255.0,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY,
            adaptive_block_size_px,
            kQrAdaptiveThresholdOffset);
        (void)scan_gray_roi(adaptive_binary_roi, 1.0F, "adaptive_binary");
      }
    } catch (const cv::Exception &e) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "QR preprocess failed (rknn): %s",
          e.what());
    }
  }

  std::sort(
      decoded_codes.begin(),
      decoded_codes.end(),
      [](const DecodedVisualCode &a, const DecodedVisualCode &b) {
        return a.center_distance_sq < b.center_distance_sq;
      });
  return decoded_codes;
}

void QrVisionNode::drawRknnDetections(
    cv::Mat &display,
    const DepthSampleResult &center_depth) const
{
  // 绿框 + 类名/分数；仅最高分框显示其中心深度
  for (const Detection &detection : last_rknn_detections_) {
    cv::rectangle(display, detection.box, cv::Scalar(0, 255, 0), 2);
    const char *class_name = RknnYoloDetector::className(detection.class_id);
    std::string label;
    if (detection.has_depth) {
      label = cv::format(
          "%s %.2f  depth %.2fm (best)",
          class_name,
          detection.score,
          detection.depth_m);
    } else {
      label = cv::format("%s %.2f", class_name, detection.score);
    }
    const int label_y = std::max(20, detection.box.y - 6);
    cv::putText(
        display,
        label,
        cv::Point(detection.box.x, label_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.52,
        cv::Scalar(255, 0, 255),
        1,
        cv::LINE_AA);
  }
  (void)center_depth;
}

void QrVisionNode::drawRknnProbeHud(
    cv::Mat &display,
    const DepthSampleResult &center_depth) const
{
  // 对齐 package_qr_shelf_rknn_probe / rknn_model_probe HUD
  const cv::Scalar text_color(255, 0, 255);
  double npu_capacity_fps = 0.0;
  std::array<double, kRknnWorkerCount> core_ms{};
  for (std::size_t i = 0; i < kRknnWorkerCount; ++i) {
    core_ms[i] = rknn_core_run_ms_[i].load();
    if (core_ms[i] > 0.0) {
      npu_capacity_fps += 1000.0 / core_ms[i];
    }
  }

  int y = 30;
  const auto put_line = [&](const std::string &line) {
    cv::putText(
        display,
        line,
        cv::Point(12, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.54,
        text_color,
        1,
        cv::LINE_AA);
    y += 26;
  };

  put_line(cv::format(
      "FPS %.1f  NPU cap %.1f  Drop %llu  Stale %llu  mode=%s",
      smoothed_fps_,
      npu_capacity_fps,
      static_cast<unsigned long long>(rknn_dropped_count_.load()),
      static_cast<unsigned long long>(rknn_stale_count_.load()),
      last_input_mode_.c_str()));
  put_line(cv::format(
      "C0 %.2fms  C1 %.2fms  C2 %.2fms  processed %llu",
      core_ms[0],
      core_ms[1],
      core_ms[2],
      static_cast<unsigned long long>(rknn_processed_count_.load())));
  put_line(cv::format(
      "Pre %.2f  In %.2f  Run %.2f  Out %.2f  Post %.2f  Total %.2f ms",
      last_rknn_timing_.preprocess_ms,
      last_rknn_timing_.input_prepare_ms,
      last_rknn_timing_.rknn_run_ms,
      last_rknn_timing_.output_get_ms,
      last_rknn_timing_.postprocess_ms,
      last_rknn_timing_.detector_total_ms));
  put_line(cv::format(
      "RGB %dx%d  detections=%zu  camera_info=%s",
      display.cols,
      display.rows,
      last_rknn_detections_.size(),
      has_camera_info_ ? "ready" : "waiting"));
  put_line(
      center_depth.has_valid_depth
          ? cv::format(
              "Best-box depth %.3fm (max-score det center, soft depth)",
              center_depth.depth_m)
          : std::string("Best-box depth n/a (no det / soft depth unavailable)"));
  put_line(cv::format(
      "3-context RKNN memory: weight %.1f MiB  internal %.1f MiB  DMA %.1f MiB",
      rknn_weight_mib_,
      rknn_internal_mib_,
      rknn_dma_mib_));

  if (!pkg_code_state_.stable_code.empty() ||
      !sku_code_state_.stable_code.empty() ||
      !debug_raw_symbol_.empty())
  {
    put_line(cv::format(
        "QR pkg=%s sku=%s raw=%s",
        pkg_code_state_.stable_code.empty() ? "-" : pkg_code_state_.stable_code.c_str(),
        sku_code_state_.stable_code.empty() ? "-" : sku_code_state_.stable_code.c_str(),
        debug_raw_symbol_.empty() ? "-" : debug_raw_symbol_.c_str()));
  }
}
#endif

#if DRONE_PERCEPTION_HAS_BPU
bool QrVisionNode::encodeFrameCaptureJpeg(
    const cv::Mat &color_image,
    std::vector<uint8_t> &jpeg_data)
{
  jpeg_data.clear();

  if (color_image.empty()) {
    return false;
  }

  try {
    const std::vector<int> encode_params = {
        cv::IMWRITE_JPEG_QUALITY,
        barcode_capture_jpeg_quality_};

    return cv::imencode(".jpg", color_image, jpeg_data, encode_params) &&
        !jpeg_data.empty();
  } catch (const cv::Exception &e) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "frame capture jpeg encode failed: %s",
        e.what());
  }

  return false;
}

void QrVisionNode::resetCaptureCandidateState()
{
  best_package_capture_candidate_ = {};
  package_capture_candidate_count_ = 0;
  hover_capture_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

bool QrVisionNode::selectBestPackageInDeadzone(
    const cv::Mat &color_image,
    CaptureFrameCandidate &candidate) const
{
  candidate = {};

  if (color_image.empty() || last_bpu_detections_.empty()) {
    return false;
  }

  const double image_center_x = static_cast<double>(color_image.cols) * 0.5;
  const double image_center_y = static_cast<double>(color_image.rows) * 0.5;
  const double deadzone_radius =
      static_cast<double>(std::min(color_image.cols, color_image.rows)) *
      package_capture_deadzone_ratio_;
  const double deadzone_radius_sq = deadzone_radius * deadzone_radius;

  for (const BpuYoloDetection &detection : last_bpu_detections_) {
    if (detection.class_id != kYoloClassPackage ||
        static_cast<double>(detection.score) < package_capture_min_score_) {
      continue;
    }

    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        color_image.cols,
        color_image.rows);

    if (image_rect.x_max - image_rect.x_min <= 1.0F ||
        image_rect.y_max - image_rect.y_min <= 1.0F) {
      continue;
    }

    const double package_center_x =
        (static_cast<double>(image_rect.x_min) + static_cast<double>(image_rect.x_max)) * 0.5;
    const double package_center_y =
        (static_cast<double>(image_rect.y_min) + static_cast<double>(image_rect.y_max)) * 0.5;
    const double dx = package_center_x - image_center_x;
    const double dy = package_center_y - image_center_y;
    const double center_distance_sq = dx * dx + dy * dy;

    if (center_distance_sq > deadzone_radius_sq) {
      continue;
    }

    const bool replace_candidate =
        !candidate.valid ||
        detection.score > candidate.package_score ||
        (std::abs(detection.score - candidate.package_score) <= 1.0e-6F &&
         center_distance_sq < candidate.center_distance_sq);

    if (!replace_candidate) {
      continue;
    }

    candidate.package_score = detection.score;
    candidate.center_distance_sq = center_distance_sq;
    candidate.valid = true;
  }

  if (!candidate.valid) {
    return false;
  }

  candidate.image = color_image.clone();
  return !candidate.image.empty();
}

void QrVisionNode::bufferPackageCaptureCandidate(const cv::Mat &color_image)
{
  if (!hover_active_ ||
      hover_capture_start_time_.nanoseconds() <= 0 ||
      color_image.empty()) {
    return;
  }

  const rclcpp::Time now_time = this->now();
  const double elapsed_s = (now_time - hover_capture_start_time_).seconds();

  if (elapsed_s < 0.0 || elapsed_s > capture_collection_duration_s_) {
    return;
  }

  CaptureFrameCandidate candidate;

  try {
    if (!selectBestPackageInDeadzone(color_image, candidate)) {
      return;
    }
  } catch (const cv::Exception &e) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "package capture candidate clone failed: %s",
        e.what());
    return;
  }

  candidate.stamp = now_time;
  ++package_capture_candidate_count_;

  const bool replace_best =
      !best_package_capture_candidate_.valid ||
      candidate.package_score > best_package_capture_candidate_.package_score ||
      (std::abs(candidate.package_score - best_package_capture_candidate_.package_score) <= 1.0e-6F &&
       candidate.center_distance_sq < best_package_capture_candidate_.center_distance_sq);

  if (!replace_best) {
    return;
  }

  best_package_capture_candidate_ = std::move(candidate);

  RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "best package capture candidate updated score=%.3f center_distance_px=%.1f "
      "elapsed_s=%.2f candidates=%d",
      best_package_capture_candidate_.package_score,
      std::sqrt(best_package_capture_candidate_.center_distance_sq),
      elapsed_s,
      package_capture_candidate_count_);
}

bool QrVisionNode::publishBestBufferedCapture()
{
  if (!best_package_capture_candidate_.valid ||
      best_package_capture_candidate_.image.empty()) {
    RCLCPP_INFO(
        get_logger(),
        "no centered package capture candidate collected in first %.2fs",
        capture_collection_duration_s_);
    return false;
  }

  const std::string barcode = composeCaptureBarcodeWithNan(true);

  if (barcode == last_published_package_barcode_) {
    return false;
  }

  const bool published = publishFullFrameCapture(
      barcode,
      best_package_capture_candidate_.image);

  if (!published) {
    return false;
  }

  RCLCPP_INFO(
      get_logger(),
      "published best buffered package capture score=%.3f center_distance_px=%.1f "
      "candidates=%d",
      best_package_capture_candidate_.package_score,
      std::sqrt(best_package_capture_candidate_.center_distance_sq),
      package_capture_candidate_count_);

  return true;
}
#endif

void QrVisionNode::initializeSubscriptions()
{
  hover_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      hover_active_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      std::bind(
          &QrVisionNode::handleHoverActive,
          this,
          std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
          &QrVisionNode::handleCameraInfo,
          this,
          std::placeholders::_1));

  if (use_rgbd_) {
    rgbd_sub_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
        rgbd_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &QrVisionNode::handleRgbdFrame,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Using RGBD input. rgbd=%s camera_info=%s require_camera_info=%s",
        rgbd_topic_.c_str(),
        camera_info_topic_.c_str(),
        require_camera_info_ ? "true" : "false");
    return;
  }

#if DRONE_PERCEPTION_HAS_RKNN
  // RKNN 优先：color 驱动 + depth 软对齐（探针风格），不走硬 ApproximateTime
  if (enable_rknn_) {
    color_only_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        color_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &QrVisionNode::handleColorFrame,
            this,
            std::placeholders::_1));
    rknn_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &QrVisionNode::handleDepthFrameSoft,
            this,
            std::placeholders::_1));
    RCLCPP_INFO(
        get_logger(),
        "Using RKNN 3-worker + soft depth. color=%s depth=%s (soft attach, "
        "match_window_ms=%.0f) camera_info=%s",
        color_topic_.c_str(),
        depth_topic_.c_str(),
        static_cast<double>(kDepthSoftMatchNs) / 1.0e6,
        camera_info_topic_.c_str());
    return;
  }
#endif

  if (require_depth_) {
    color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

    color_depth_sync_ = std::make_shared<message_filters::Synchronizer<ColorDepthSyncPolicy>>(
        ColorDepthSyncPolicy(10),
        color_sub_,
        depth_sub_);
    color_depth_sync_->registerCallback(std::bind(
        &QrVisionNode::handleSyncedFrame,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
        get_logger(),
        "Using synced color+depth. color=%s depth=%s camera_info=%s require_camera_info=%s",
        color_topic_.c_str(),
        depth_topic_.c_str(),
        camera_info_topic_.c_str(),
        require_camera_info_ ? "true" : "false");
    return;
  }

  color_only_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      color_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
          &QrVisionNode::handleColorFrame,
          this,
          std::placeholders::_1));

  RCLCPP_INFO(
      get_logger(),
      "Using color-only input. color=%s camera_info=%s require_camera_info=%s",
      color_topic_.c_str(),
      camera_info_topic_.c_str(),
      require_camera_info_ ? "true" : "false");
}

void QrVisionNode::handleCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg)
{
  (void)camera_info_msg;
  if (!has_camera_info_) {
    RCLCPP_INFO(
        get_logger(),
        "camera_info received: topic=%s size=%ux%u — camera ready for color pipeline",
        camera_info_topic_.c_str(),
        camera_info_msg->width,
        camera_info_msg->height);
  }
  has_camera_info_ = true;
}

void QrVisionNode::updateFps()
{
  const rclcpp::Time current_frame_time = this->now();
  if (last_frame_time_.nanoseconds() > 0)
  {
    const double frame_interval_s = (current_frame_time - last_frame_time_).seconds();
    if (frame_interval_s > 0.0)
    {
      const double current_fps = 1.0 / frame_interval_s;
      smoothed_fps_ = smoothed_fps_ <= 0.0 ? current_fps : smoothed_fps_ * 0.9 + current_fps * 0.1;
    }
  }
  last_frame_time_ = current_frame_time;
}

void QrVisionNode::handleSyncedFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
{
  if (require_camera_info_ && !has_camera_info_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "waiting for camera_info on %s before processing frames",
        camera_info_topic_.c_str());
    return;
  }

  const auto callback_t0 = SteadyClock::now();

  cv_bridge::CvImageConstPtr color_bridge;
  cv_bridge::CvImageConstPtr depth_bridge;

  try
  {
    color_bridge = cv_bridge::toCvShare(color_msg, "bgr8");
    depth_bridge = cv_bridge::toCvShare(depth_msg);
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "cv_bridge synced conversion failed: %s",
        ex.what());
    return;
  }

  processFrame(color_bridge, depth_bridge, callback_t0, "synced");
}

void QrVisionNode::handleColorFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &color_msg)
{
  if (require_camera_info_ && !has_camera_info_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "waiting for camera_info on %s before processing color frames",
        camera_info_topic_.c_str());
    return;
  }

#if DRONE_PERCEPTION_HAS_RKNN
  // 三 worker 路径：回调只入队，不在此线程 infer（对齐探针）
  if (enable_rknn_) {
    enqueueRknnColorFrame(color_msg);
    return;
  }
#endif

  const auto callback_t0 = SteadyClock::now();
  cv_bridge::CvImageConstPtr color_bridge;
  try {
    color_bridge = cv_bridge::toCvShare(color_msg, "bgr8");
  } catch (const cv_bridge::Exception &ex) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "cv_bridge color conversion failed: %s",
        ex.what());
    return;
  }

  processFrame(color_bridge, cv_bridge::CvImageConstPtr{}, callback_t0, "color");
}

void QrVisionNode::handleRgbdFrame(
    const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &rgbd_msg)
{
  const auto callback_t0 = SteadyClock::now();

  cv_bridge::CvImageConstPtr color_bridge;
  cv_bridge::CvImageConstPtr depth_bridge;

  try
  {
    color_bridge = cv_bridge::toCvShare(rgbd_msg->rgb, rgbd_msg, "bgr8");
    depth_bridge = cv_bridge::toCvShare(rgbd_msg->depth, rgbd_msg);
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "cv_bridge RGBD conversion failed: %s",
        ex.what());
    return;
  }

  has_camera_info_ = true;
  processFrame(color_bridge, depth_bridge, callback_t0, "rgbd");
}

void QrVisionNode::processFrame(
    const cv_bridge::CvImageConstPtr &color_bridge,
    const cv_bridge::CvImageConstPtr &depth_bridge,
    const std::chrono::steady_clock::time_point &callback_t0,
    const char *input_mode)
{
  if (!color_bridge || color_bridge->image.empty()) {
    return;
  }

  const bool has_depth = static_cast<bool>(depth_bridge) && !depth_bridge->image.empty();
  if (has_depth &&
      (color_bridge->image.cols != depth_bridge->image.cols ||
       color_bridge->image.rows != depth_bridge->image.rows))
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Color and depth image sizes do not match: color=%dx%d depth=%dx%d",
        color_bridge->image.cols,
        color_bridge->image.rows,
        depth_bridge->image.cols,
        depth_bridge->image.rows);
    return;
  }

  updateFps();

  DepthSampleResult center_depth{};
  if (has_depth) {
    const int center_u = color_bridge->image.cols / 2;
    const int center_v = color_bridge->image.rows / 2;
    center_depth = depth_processor_.sampleAt(
        depth_bridge->image,
        center_u,
        center_v,
        sample_radius_px_);
  }

// RKNN 三 worker 路径不在 processFrame 内推理（见 enqueueRknnColorFrame / consumeRknnResults）

#if DRONE_PERCEPTION_HAS_BPU
  last_bpu_detections_.clear();

  if (!enable_rknn_ && enable_bpu_ && bpu_detector_) {
    const auto preprocess_t0 = SteadyClock::now();
    const bool input_ready = prepareBpuInput(color_bridge->image);
    const auto preprocess_t1 = SteadyClock::now();

    if (input_ready) {
      try {
        const auto infer_t0 = SteadyClock::now();

        const auto detections = bpu_detector_->inferNv12(
            bpu_input_nv12_.data(),
            bpu_input_nv12_.size());
        last_bpu_detections_ = detections;

        const auto infer_t1 = SteadyClock::now();
        int qrcode_count = 0;
        int package_count = 0;
        int shelf_tag_count = 0;

        for (const BpuYoloDetection &detection : detections) {
          if (detection.class_id == kYoloClassQrcode) {
            ++qrcode_count;
          } else if (detection.class_id == kYoloClassPackage) {
            ++package_count;
          } else if (detection.class_id == kYoloClassShelfTag) {
            ++shelf_tag_count;
          }
        }

        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "BPU inference ok. mode=%s preprocess_ms=%.2f infer_ms=%.2f "
            "input_size=%zu detections=%zu qrcode=%d package=%d shelf_tag=%d "
            "letterbox_scale=%.4f letterbox_pad=%d,%d",
            input_mode,
            elapsedMs(preprocess_t0, preprocess_t1),
            elapsedMs(infer_t0, infer_t1),
            bpu_input_nv12_.size(),
            detections.size(),
            qrcode_count,
            package_count,
            shelf_tag_count,
            bpu_letterbox_.scale,
            bpu_letterbox_.pad_x,
            bpu_letterbox_.pad_y);
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "BPU inference failed: %s",
            e.what());
      }
    }
  }

  // 本阶段不接 OCR 文字识别
  last_ocr_regions_.clear();
  updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
#endif

#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_rknn_) {
    updateVisualCodeStability(
        decodeVisualCodesFromDetections(color_bridge->image, last_bpu_detections_));
  }
#endif
#if !DRONE_PERCEPTION_HAS_RKNN && !DRONE_PERCEPTION_HAS_BPU
  updateVisualCodeStability({});
  updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
#endif

  // 非 RKNN 路径的发布；RKNN 三 worker 在 consumeRknnResults 里发布
  if (!enable_rknn_) {
    publishBarcodeCapture(color_bridge->image);
  }

  const auto callback_t1 = SteadyClock::now();
  last_callback_ms_ = elapsedMs(callback_t0, callback_t1);
  last_input_mode_ = input_mode != nullptr ? input_mode : "color";

  if (debug_view_)
  {
    displayDebugFrame(color_bridge->image, center_depth);
  }

  if (center_depth.has_valid_depth)
  {
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "video stream mode=%s fps=%.1f size=%dx%d center_depth=%.3fm camera_info=%s "
        "callback_ms=%.2f debug_view=%s",
        input_mode,
        smoothed_fps_,
        color_bridge->image.cols,
        color_bridge->image.rows,
        center_depth.depth_m,
        has_camera_info_ ? "true" : "false",
        last_callback_ms_,
        debug_view_ ? "true" : "false");
  }
  else
  {
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "video stream mode=%s fps=%.1f size=%dx%d center_depth=invalid camera_info=%s "
        "callback_ms=%.2f debug_view=%s",
        input_mode,
        smoothed_fps_,
        color_bridge->image.cols,
        color_bridge->image.rows,
        has_camera_info_ ? "true" : "false",
        last_callback_ms_,
        debug_view_ ? "true" : "false");
  }
}

void QrVisionNode::displayDebugFrame(
    const cv::Mat &color_image,
    const DepthSampleResult &center_depth)
{
  cv::Mat display = color_image.clone();

#if DRONE_PERCEPTION_HAS_RKNN
  // RKNN 模式：完整搬迁 rknn_model_probe 风格 HUD + 检测框
  if (enable_rknn_) {
    drawRknnDetections(display, center_depth);
    drawRknnProbeHud(display, center_depth);
    cv::imshow(window_name_, display);
    cv::waitKey(1);
    return;
  }
#endif

  cv::Mat overlay = display.clone();
  const int panel_bottom = enable_bpu_ocr_ ? 236 : 164;
  const int panel_right = std::min(display.cols - 12, 620);
  cv::rectangle(
      overlay,
      cv::Point(12, 12),
      cv::Point(panel_right, panel_bottom),
      cv::Scalar(30, 30, 30),
      cv::FILLED);
  cv::addWeighted(overlay, 0.45, display, 0.55, 0.0, display);

  cv::putText(
      display,
      cv::format("FPS: %.1f", smoothed_fps_),
      cv::Point(24, 38),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 255, 255),
      1);
  cv::putText(
      display,
      cv::format("Size: %dx%d", color_image.cols, color_image.rows),
      cv::Point(24, 60),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(255, 255, 255),
      1);
  cv::putText(
      display,
      center_depth.has_valid_depth
        ? std::string(cv::format("Center depth: %.3fm", center_depth.depth_m))
        : std::string("Center depth: invalid"),
      cv::Point(24, 82),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(180, 255, 180),
      1);

  auto fit_status_text = [](std::string text) {
    static constexpr std::size_t kMaxStatusTextLength = 58U;

    if (text.size() > kMaxStatusTextLength) {
      text = text.substr(0U, kMaxStatusTextLength - 3U) + "...";
    }

    return text;
  };

  auto draw_visual_code_line = [&](
      const CodeStabilityState &state,
      const char *category,
      int y) {
    if (state.stable_code.empty()) {
      return y;
    }

    const bool is_ocr_code = state.stable_symbol_type == "OCR";
    const char *kind = is_ocr_code ? "OCR" : "QR";
    const std::string line = fit_status_text(cv::format(
        "%s %s: %s",
        kind,
        category,
        state.stable_code.c_str()));

    cv::putText(
        display,
        line,
        cv::Point(24, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        is_ocr_code ? cv::Scalar(255, 210, 120) : cv::Scalar(120, 220, 255),
        1);

    return y + 22;
  };

  int visual_code_y = 106;
  visual_code_y = draw_visual_code_line(shelf_code_state_, "shelf", visual_code_y);
  visual_code_y = draw_visual_code_line(sku_code_state_, "sku", visual_code_y);
  (void)draw_visual_code_line(pkg_code_state_, "pkg", visual_code_y);

  if (!debug_raw_symbol_.empty()) {
    const std::string raw_line = cv::format(
        "Raw zbar: %s %s",
        debug_raw_symbol_type_.c_str(),
        fit_status_text(debug_raw_symbol_).c_str());
    cv::putText(
        display,
        raw_line,
        cv::Point(24, visual_code_y + 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(180, 180, 180),
        1);
  }

#if DRONE_PERCEPTION_HAS_BPU
  drawBpuDetections(display);
  drawOcrRegions(display);
  drawQrPreprocessPreview(display);

  if (enable_bpu_ocr_ && !last_ocr_regions_.empty()) {
    int ocr_text_y = visual_code_y + 48;

    for (const OcrTextRegion &region : last_ocr_regions_) {
      const std::string ocr_line = cv::format(
          "OCR: %s (%.2f)",
          fit_status_text(region.text).c_str(),
          region.score);
      cv::putText(
          display,
          ocr_line,
          cv::Point(24, ocr_text_y),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          cv::Scalar(255, 210, 120),
          1);
      ocr_text_y += 22;

      if (ocr_text_y > panel_bottom - 8) {
        break;
      }
    }
  }
#endif

  cv::imshow(window_name_, display);
  cv::waitKey(1);
}

void QrVisionNode::initializeCameraControlDefaults()
{
  camera_numeric_controls_[kCameraExposureControlIndex] = {
      "rgb_camera.exposure",
      "exposure",
      1,
      10000,
      999,
      1,
      10000,
      999,
      999,
      998,
      rclcpp::ParameterType::PARAMETER_INTEGER,
      true,
      false};
  camera_numeric_controls_[kCameraGainControlIndex] = {
      "rgb_camera.gain",
      "gain",
      0,
      128,
      64,
      0,
      128,
      64,
      64,
      64,
      rclcpp::ParameterType::PARAMETER_INTEGER,
      true,
      false};
  camera_numeric_controls_[kCameraWhiteBalanceControlIndex] = {
      "rgb_camera.white_balance",
      "white_balance",
      2800,
      6500,
      4600,
      2800,
      6500,
      4600,
      4600,
      1800,
      rclcpp::ParameterType::PARAMETER_DOUBLE,
      true,
      false};

  camera_bool_controls_[kCameraAutoExposureControlIndex] = {
      "rgb_camera.enable_auto_exposure",
      "enable_auto_exposure",
      false,
      false,
      true,
      false};
  camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex] = {
      "rgb_camera.enable_auto_white_balance",
      "enable_auto_white_balance",
      true,
      true,
      true,
      false};
  camera_bool_controls_[kCameraAutoExposurePriorityControlIndex] = {
      "rgb_camera.auto_exposure_priority",
      "auto_exposure_priority",
      false,
      false,
      true,
      false};

  for (CameraNumericControl &control : camera_numeric_controls_) {
    control.restore_value = control.fallback_default;
    control.restore_value_initialized = true;
  }

  for (CameraBoolControl &control : camera_bool_controls_) {
    control.restore_value = control.current_value;
    control.restore_value_initialized = true;
  }

  for (std::size_t i = 0U; i < camera_trackbar_contexts_.size(); ++i) {
    camera_trackbar_contexts_[i].node = this;
    camera_trackbar_contexts_[i].index = i;
  }

  updateCameraControlEnableStates();
}

void QrVisionNode::initializeCameraControls()
{
  initializeCameraControlDefaults();

  if (!camera_controls_enabled_) {
    return;
  }

  if (!debug_view_) {
    camera_controls_enabled_ = false;
    RCLCPP_WARN(
        get_logger(),
        "camera_controls_enabled requires debug_view=true because it uses OpenCV HighGUI");
    return;
  }

  if (camera_param_node_.empty()) {
    camera_param_node_ = "/camera/camera";
  }

  try {
    camera_param_client_ =
        std::make_shared<rclcpp::AsyncParametersClient>(this, camera_param_node_);

    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      CameraNumericControl &control = camera_numeric_controls_[i];
      control.current_value = std::clamp(
          control.current_value,
          control.min_value,
          control.max_value);
      control.last_sent_value = control.current_value;
      control.trackbar_value = control.current_value - control.min_value;

      cv::createTrackbar(
          control.label,
          window_name_,
          nullptr,
          std::max(1, control.max_value - control.min_value),
          &QrVisionNode::handleCameraTrackbarCallback,
          &camera_trackbar_contexts_[i]);
      updateCameraTrackbarPosition(i);
    }

    cv::setMouseCallback(
        window_name_,
        &QrVisionNode::handleCameraControlsMouseCallback,
        this);

    camera_controls_attached_ = true;
    camera_controls_status_text_ = "camera param node offline";

    camera_controls_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(camera_controls_update_period_ms_),
        [this]() {
          updateCameraControlClient();
        });

    RCLCPP_INFO(
        get_logger(),
        "D435i RGB controls embedded in debug_view. window=%s camera_param_node=%s refresh_ms=%d",
        window_name_.c_str(),
        camera_param_node_.c_str(),
        camera_controls_update_period_ms_);
  } catch (const cv::Exception &e) {
    camera_controls_enabled_ = false;
    camera_controls_attached_ = false;

    RCLCPP_WARN(
        get_logger(),
        "Cannot attach D435i controls to debug window: %s",
        e.what());
  }
}

void QrVisionNode::updateCameraControlClient()
{
  if (!camera_controls_enabled_ || !camera_param_client_) {
    return;
  }

  const bool service_ready = camera_param_client_->service_is_ready();

  if (!service_ready) {
    camera_param_node_online_ = false;
    camera_controls_status_text_ = "camera param node offline";
    return;
  }

  if (!camera_param_node_online_) {
    camera_param_node_online_ = true;
    camera_param_describe_requested_ = false;
    camera_param_get_requested_ = false;
    camera_controls_status_text_ = "camera param node online";
  }

  if (!camera_param_describe_requested_ && !camera_param_describe_in_flight_) {
    requestCameraControlDescriptors();
  }

  if (!camera_param_get_requested_ && !camera_param_get_in_flight_) {
    requestCameraControlValues();
  }

  sendPendingCameraControlParameters();
}

void QrVisionNode::requestCameraControlDescriptors()
{
  if (!camera_param_client_) {
    return;
  }

  std::vector<std::string> parameter_names;
  parameter_names.reserve(camera_numeric_controls_.size() + camera_bool_controls_.size());

  for (const CameraNumericControl &control : camera_numeric_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  for (const CameraBoolControl &control : camera_bool_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  camera_param_describe_requested_ = true;
  camera_param_describe_in_flight_ = true;

  try {
    camera_param_client_->describe_parameters(
        parameter_names,
        [this](auto future) {
          camera_param_describe_in_flight_ = false;

          try {
            applyCameraControlDescriptors(future.get());
            camera_controls_status_text_ = "camera descriptors loaded";
          } catch (const std::exception &e) {
            camera_controls_status_text_ = "describe failed";
            RCLCPP_WARN(
                get_logger(),
                "Failed to describe D435i RGB parameters: %s",
                e.what());
          }
        });
  } catch (const std::exception &e) {
    camera_param_describe_in_flight_ = false;
    camera_controls_status_text_ = "describe request failed";

    RCLCPP_WARN(
        get_logger(),
        "Failed to request D435i RGB parameter descriptors: %s",
        e.what());
  }
}

void QrVisionNode::requestCameraControlValues()
{
  if (!camera_param_client_) {
    return;
  }

  std::vector<std::string> parameter_names;
  parameter_names.reserve(camera_numeric_controls_.size() + camera_bool_controls_.size());

  for (const CameraNumericControl &control : camera_numeric_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  for (const CameraBoolControl &control : camera_bool_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  camera_param_get_requested_ = true;
  camera_param_get_in_flight_ = true;

  try {
    camera_param_client_->get_parameters(
        parameter_names,
        [this](auto future) {
          camera_param_get_in_flight_ = false;

          try {
            applyCameraControlValues(future.get());
            camera_controls_status_text_ = "fixed defaults pending";
          } catch (const std::exception &e) {
            camera_controls_status_text_ = "get parameters failed";
            RCLCPP_WARN(
                get_logger(),
                "Failed to read D435i RGB parameters: %s",
                e.what());
          }
        });
  } catch (const std::exception &e) {
    camera_param_get_in_flight_ = false;
    camera_controls_status_text_ = "get request failed";

    RCLCPP_WARN(
        get_logger(),
        "Failed to request D435i RGB parameter values: %s",
        e.what());
  }
}

void QrVisionNode::applyCameraControlDescriptors(
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> &descriptors)
{
  for (const rcl_interfaces::msg::ParameterDescriptor &descriptor : descriptors) {
    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      CameraNumericControl &control = camera_numeric_controls_[i];

      if (descriptor.name != control.parameter_name) {
        continue;
      }

      const auto descriptor_type = static_cast<rclcpp::ParameterType>(descriptor.type);

      if (descriptor_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
          descriptor_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        control.parameter_type = descriptor_type;
      }

      int descriptor_min = control.fallback_min;
      int descriptor_max = control.fallback_max;

      if (!descriptor.integer_range.empty()) {
        descriptor_min = static_cast<int>(descriptor.integer_range.front().from_value);
        descriptor_max = static_cast<int>(descriptor.integer_range.front().to_value);
      } else if (!descriptor.floating_point_range.empty()) {
        descriptor_min = static_cast<int>(std::round(
            descriptor.floating_point_range.front().from_value));
        descriptor_max = static_cast<int>(std::round(
            descriptor.floating_point_range.front().to_value));
      }

      if (descriptor_min > descriptor_max) {
        std::swap(descriptor_min, descriptor_max);
      }

      if (descriptor_min == descriptor_max) {
        descriptor_min = control.fallback_min;
        descriptor_max = control.fallback_max;
      }

      control.min_value = descriptor_min;
      control.max_value = descriptor_max;
      control.current_value = std::clamp(
          control.current_value,
          control.min_value,
          control.max_value);
      control.last_sent_value = std::clamp(
          control.last_sent_value,
          control.min_value,
          control.max_value);

      if (camera_controls_attached_) {
        cv::setTrackbarMax(
            control.label,
            window_name_,
            std::max(1, control.max_value - control.min_value));
        updateCameraTrackbarPosition(i);
      }
    }
  }
}

void QrVisionNode::applyCameraControlValues(
    const std::vector<rclcpp::Parameter> &parameters)
{
  for (const rclcpp::Parameter &parameter : parameters) {
    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      CameraNumericControl &control = camera_numeric_controls_[i];

      if (parameter.get_name() != control.parameter_name) {
        continue;
      }

      int value = control.current_value;

      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<int>(parameter.as_int());
        control.parameter_type = rclcpp::ParameterType::PARAMETER_INTEGER;
      } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = static_cast<int>(std::round(parameter.as_double()));
        control.parameter_type = rclcpp::ParameterType::PARAMETER_DOUBLE;
      } else {
        continue;
      }

      control.current_value = std::clamp(value, control.min_value, control.max_value);
      control.last_sent_value = control.current_value;
      control.send_pending = false;
      updateCameraTrackbarPosition(i);
    }

    for (CameraBoolControl &control : camera_bool_controls_) {
      if (parameter.get_name() != control.parameter_name ||
          parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        continue;
      }

      control.current_value = parameter.as_bool();
      control.last_sent_value = control.current_value;
      control.send_pending = false;
    }
  }

  updateCameraControlEnableStates();
  resetCameraControlsToDefaults();
}

void QrVisionNode::updateCameraControlEnableStates()
{
  const bool auto_exposure =
      camera_bool_controls_[kCameraAutoExposureControlIndex].current_value;
  const bool auto_white_balance =
      camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex].current_value;

  camera_bool_controls_[kCameraAutoExposureControlIndex].enabled = true;
  camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex].enabled = true;
  camera_bool_controls_[kCameraAutoExposurePriorityControlIndex].enabled = auto_exposure;

  camera_numeric_controls_[kCameraExposureControlIndex].enabled = !auto_exposure;
  camera_numeric_controls_[kCameraGainControlIndex].enabled = !auto_exposure;
  camera_numeric_controls_[kCameraWhiteBalanceControlIndex].enabled = !auto_white_balance;
}

void QrVisionNode::resetCameraControlsToDefaults()
{
  for (CameraBoolControl &control : camera_bool_controls_) {
    const bool restore_value = control.restore_value_initialized
        ? control.restore_value
        : control.current_value;

    control.current_value = restore_value;
    control.send_pending = control.current_value != control.last_sent_value;
  }

  updateCameraControlEnableStates();

  for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
    CameraNumericControl &control = camera_numeric_controls_[i];
    const int restore_value = control.restore_value_initialized
        ? control.restore_value
        : control.fallback_default;

    control.current_value = std::clamp(
        restore_value,
        control.min_value,
        control.max_value);
    control.send_pending = control.enabled && control.current_value != control.last_sent_value;
    updateCameraTrackbarPosition(i);
  }

  camera_controls_status_text_ = "reset defaults pending";
}

void QrVisionNode::sendPendingCameraControlParameters()
{
  if (!camera_controls_enabled_ ||
      !camera_param_node_online_ ||
      !camera_param_client_ ||
      camera_param_set_in_flight_) {
    return;
  }

  const rclcpp::Time now_time = this->now();

  if (last_camera_param_send_time_.nanoseconds() > 0) {
    const int64_t elapsed_ms =
        (now_time - last_camera_param_send_time_).nanoseconds() / 1000000;

    if (elapsed_ms < kCameraControlsSendDebounceMs) {
      return;
    }
  }

  std::vector<rclcpp::Parameter> parameters;

  for (const CameraBoolControl &control : camera_bool_controls_) {
    if (!control.enabled ||
        !control.send_pending ||
        control.current_value == control.last_sent_value) {
      continue;
    }

    parameters.emplace_back(control.parameter_name, control.current_value);
  }

  if (parameters.empty()) {
    for (const CameraNumericControl &control : camera_numeric_controls_) {
      if (!control.enabled ||
          !control.send_pending ||
          control.current_value == control.last_sent_value) {
        continue;
      }

      if (control.parameter_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        parameters.emplace_back(
            control.parameter_name,
            static_cast<double>(control.current_value));
      } else {
        parameters.emplace_back(
            control.parameter_name,
            static_cast<int64_t>(control.current_value));
      }
    }
  }

  if (parameters.empty()) {
    return;
  }

  camera_param_set_in_flight_ = true;
  last_camera_param_send_time_ = now_time;

  try {
    camera_param_client_->set_parameters(
        parameters,
        [this, parameters](auto future) {
          camera_param_set_in_flight_ = false;

          try {
            handleCameraParameterSetResults(parameters, future.get());
          } catch (const std::exception &e) {
            camera_controls_status_text_ = "set parameters failed";
            RCLCPP_WARN(
                get_logger(),
                "Failed to set D435i RGB parameters: %s",
                e.what());
          }
        });
  } catch (const std::exception &e) {
    camera_param_set_in_flight_ = false;
    camera_controls_status_text_ = "set request failed";

    RCLCPP_WARN(
        get_logger(),
        "Failed to send D435i RGB parameter request: %s",
        e.what());
  }
}

void QrVisionNode::handleCameraParameterSetResults(
    const std::vector<rclcpp::Parameter> &parameters,
    const std::vector<rcl_interfaces::msg::SetParametersResult> &results)
{
  const std::size_t result_count = std::min(parameters.size(), results.size());
  bool all_success = result_count == parameters.size();

  for (std::size_t i = 0U; i < result_count; ++i) {
    const rclcpp::Parameter &parameter = parameters[i];
    const rcl_interfaces::msg::SetParametersResult &result = results[i];

    if (!result.successful) {
      all_success = false;
      RCLCPP_WARN(
          get_logger(),
          "D435i parameter rejected. name=%s reason=%s",
          parameter.get_name().c_str(),
          result.reason.c_str());
    }

    for (CameraNumericControl &control : camera_numeric_controls_) {
      if (parameter.get_name() != control.parameter_name) {
        continue;
      }

      if (result.successful) {
        control.last_sent_value = control.current_value;
      }

      control.send_pending = false;
    }

    for (CameraBoolControl &control : camera_bool_controls_) {
      if (parameter.get_name() != control.parameter_name) {
        continue;
      }

      if (result.successful) {
        control.last_sent_value = control.current_value;
      }

      control.send_pending = false;
    }
  }

  camera_controls_status_text_ = all_success
      ? "camera parameters applied"
      : "camera parameter rejected";
}

void QrVisionNode::handleCameraTrackbarChange(std::size_t index, int position)
{
  if (camera_controls_updating_trackbar_ || index >= camera_numeric_controls_.size()) {
    return;
  }

  CameraNumericControl &control = camera_numeric_controls_[index];
  const int value = std::clamp(
      control.min_value + position,
      control.min_value,
      control.max_value);

  control.current_value = value;
  control.trackbar_value = control.current_value - control.min_value;

  if (control.enabled) {
    control.send_pending = true;
  }
}

void QrVisionNode::handleCameraControlsClick(int x, int y)
{
  const cv::Point window_point(x, y);

  if (!camera_controls_panel_rect_.contains(window_point)) {
    return;
  }

  const cv::Point point(
      window_point.x - camera_controls_panel_rect_.x,
      window_point.y - camera_controls_panel_rect_.y);

  if (camera_reset_button_rect_.contains(point)) {
    resetCameraControlsToDefaults();
    return;
  }

  for (std::size_t i = 0U; i < camera_bool_controls_.size(); ++i) {
    const CameraBoolControl &control = camera_bool_controls_[i];

    if (!control.enabled) {
      continue;
    }

    if (control.true_rect.contains(point)) {
      setCameraBoolControl(i, true);
      return;
    }

    if (control.false_rect.contains(point)) {
      setCameraBoolControl(i, false);
      return;
    }
  }
}

void QrVisionNode::setCameraBoolControl(std::size_t index, bool value)
{
  if (index >= camera_bool_controls_.size() ||
      !camera_bool_controls_[index].enabled ||
      camera_bool_controls_[index].current_value == value) {
    return;
  }

  CameraBoolControl &control = camera_bool_controls_[index];
  control.current_value = value;
  control.send_pending = true;
  updateCameraControlEnableStates();

  if (index == kCameraAutoExposureControlIndex && !value) {
    camera_numeric_controls_[kCameraExposureControlIndex].send_pending = true;
    camera_numeric_controls_[kCameraGainControlIndex].send_pending = true;
  }

  if (index == kCameraAutoWhiteBalanceControlIndex && !value) {
    camera_numeric_controls_[kCameraWhiteBalanceControlIndex].send_pending = true;
  }
}

void QrVisionNode::updateCameraTrackbarPosition(std::size_t index)
{
  if (index >= camera_numeric_controls_.size() || !camera_controls_attached_) {
    return;
  }

  CameraNumericControl &control = camera_numeric_controls_[index];
  const int position = std::clamp(
      control.current_value - control.min_value,
      0,
      std::max(1, control.max_value - control.min_value));

  camera_controls_updating_trackbar_ = true;
  cv::setTrackbarPos(
      control.label,
      window_name_,
      position);
  camera_controls_updating_trackbar_ = false;
  control.trackbar_value = position;
}

void QrVisionNode::drawCameraBoolControl(
    cv::Mat &panel,
    CameraBoolControl &control,
    int y)
{
  const cv::Scalar text_color = control.enabled
      ? cv::Scalar(230, 235, 235)
      : cv::Scalar(115, 120, 125);
  const cv::Scalar active_color(90, 220, 140);
  const cv::Scalar inactive_color = control.enabled
      ? cv::Scalar(90, 95, 105)
      : cv::Scalar(70, 74, 78);
  const cv::Scalar border_color = control.enabled
      ? cv::Scalar(185, 195, 205)
      : cv::Scalar(90, 94, 98);
  const int true_x = 360;
  const int false_x = 510;
  const int radius = 9;

  cv::putText(
      panel,
      control.label,
      cv::Point(28, y + 6),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      text_color,
      1);

  auto draw_option = [&](
      bool option_value,
      int center_x,
      const char *label,
      cv::Rect &rect) {
    const cv::Point center(center_x, y);
    rect = cv::Rect(center_x - 22, y - 18, 90, 36);

    cv::circle(
        panel,
        center,
        radius,
        option_value == control.current_value ? active_color : inactive_color,
        cv::FILLED);
    cv::circle(panel, center, radius, border_color, 1);

    cv::putText(
        panel,
        label,
        cv::Point(center_x + 18, y + 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        text_color,
        1);
  };

  draw_option(true, true_x, "true", control.true_rect);
  draw_option(false, false_x, "false", control.false_rect);

  if (!control.enabled) {
    cv::putText(
        panel,
        "(requires auto exposure=true)",
        cv::Point(590, y + 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(105, 110, 116),
        1);
  }
}

void QrVisionNode::drawCameraResetButton(cv::Mat &panel)
{
  camera_reset_button_rect_ = cv::Rect(
      std::max(24, panel.cols - 198),
      16,
      174,
      30);

  const cv::Scalar fill_color = camera_param_node_online_
      ? cv::Scalar(65, 95, 115)
      : cv::Scalar(62, 68, 74);
  const cv::Scalar border_color = camera_param_node_online_
      ? cv::Scalar(130, 205, 235)
      : cv::Scalar(105, 112, 120);
  const cv::Scalar text_color = camera_param_node_online_
      ? cv::Scalar(230, 245, 255)
      : cv::Scalar(185, 190, 195);

  cv::rectangle(
      panel,
      camera_reset_button_rect_,
      fill_color,
      cv::FILLED);
  cv::rectangle(
      panel,
      camera_reset_button_rect_,
      border_color,
      1);
  cv::putText(
      panel,
      "Reset Defaults",
      cv::Point(camera_reset_button_rect_.x + 18, camera_reset_button_rect_.y + 21),
      cv::FONT_HERSHEY_SIMPLEX,
      0.52,
      text_color,
      1);
}

void QrVisionNode::drawCameraControlsPanel(cv::Mat &display)
{
  camera_controls_panel_rect_ = cv::Rect();

  if (!camera_controls_enabled_ || !camera_controls_attached_ || display.empty()) {
    return;
  }

  const int video_width = display.cols;
  const int video_height = display.rows;
  const int panel_width = std::max(kCameraControlsPanelWidthPx, video_width);
  const int panel_height = kCameraControlsPanelHeightPx;

  cv::Mat canvas(
      video_height + panel_height,
      panel_width,
      display.type(),
      cv::Scalar(12, 14, 18));
  display.copyTo(canvas(cv::Rect(0, 0, video_width, video_height)));
  camera_controls_panel_rect_ = cv::Rect(
      0,
      video_height,
      panel_width,
      panel_height);

  try {
    cv::Mat panel = canvas(camera_controls_panel_rect_);
    panel.setTo(cv::Scalar(24, 28, 32));

    cv::rectangle(
        panel,
        cv::Point(0, 0),
        cv::Point(panel.cols, 58),
        cv::Scalar(38, 45, 51),
        cv::FILLED);

    cv::putText(
        panel,
        "D435i RGB Controls",
        cv::Point(24, 34),
        cv::FONT_HERSHEY_SIMPLEX,
        0.72,
        cv::Scalar(230, 245, 255),
        2);
    drawCameraResetButton(panel);

    const std::string status = camera_controls_status_text_.empty()
        ? std::string("camera param node offline")
        : camera_controls_status_text_;
    cv::putText(
        panel,
        cv::format("node: %s", camera_param_node_.c_str()),
        cv::Point(28, 82),
        cv::FONT_HERSHEY_SIMPLEX,
        0.48,
        cv::Scalar(190, 200, 205),
        1);
    cv::putText(
        panel,
        cv::format("status: %s", status.c_str()),
        cv::Point(28, 104),
        cv::FONT_HERSHEY_SIMPLEX,
        0.48,
        camera_param_node_online_ ? cv::Scalar(115, 225, 155) : cv::Scalar(100, 165, 255),
        1);

    cv::putText(
        panel,
        "Numeric sliders are attached to this debug window.",
        cv::Point(28, 134),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        cv::Scalar(160, 168, 174),
        1);

    const std::array<int, kCameraNumericControlCount> numeric_y = {162, 187, 212};

    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      const CameraNumericControl &control = camera_numeric_controls_[i];
      const cv::Scalar text_color = control.enabled
          ? cv::Scalar(225, 232, 235)
          : cv::Scalar(115, 120, 125);
      const std::string suffix = control.enabled
          ? std::string("")
          : std::string("  disabled by auto mode");

      cv::putText(
          panel,
          cv::format(
              "%s = %d  range [%d..%d]%s",
              control.parameter_name.c_str(),
              control.current_value,
              control.min_value,
              control.max_value,
              suffix.c_str()),
          cv::Point(28, numeric_y[i]),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          text_color,
          1);
    }

    cv::line(
        panel,
        cv::Point(24, 232),
        cv::Point(panel.cols - 24, 232),
        cv::Scalar(60, 68, 75),
        1);

    drawCameraBoolControl(
        panel,
        camera_bool_controls_[kCameraAutoExposureControlIndex],
        262);
    drawCameraBoolControl(
        panel,
        camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex],
        296);
    drawCameraBoolControl(
        panel,
        camera_bool_controls_[kCameraAutoExposurePriorityControlIndex],
        330);

    display = canvas;
  } catch (const cv::Exception &e) {
    camera_controls_panel_rect_ = cv::Rect();
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Failed to draw D435i controls in debug view: %s",
        e.what());
  }
}

void QrVisionNode::handleCameraTrackbarCallback(int position, void *userdata)
{
  CameraTrackbarContext *context = static_cast<CameraTrackbarContext *>(userdata);

  if (context == nullptr || context->node == nullptr) {
    return;
  }

  context->node->handleCameraTrackbarChange(context->index, position);
}

void QrVisionNode::handleCameraControlsMouseCallback(
    int event,
    int x,
    int y,
    int flags,
    void *userdata)
{
  (void)flags;

  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }

  QrVisionNode *node = static_cast<QrVisionNode *>(userdata);

  if (node == nullptr) {
    return;
  }

  node->handleCameraControlsClick(x, y);
}

#if DRONE_PERCEPTION_HAS_BPU
QrVisionNode::BpuImageRect QrVisionNode::mapBpuDetectionToImageRect(
    const BpuYoloDetection &detection,
    int image_width,
    int image_height) const
{
  if (bpu_letterbox_.scale <= 0.0F || image_width <= 0 || image_height <= 0) {
    return {};
  }

  const float image_width_f = static_cast<float>(image_width);
  const float image_height_f = static_cast<float>(image_height);
  const float pad_x = static_cast<float>(bpu_letterbox_.pad_x);
  const float pad_y = static_cast<float>(bpu_letterbox_.pad_y);

  const float x_min = (detection.x_min_px - pad_x) / bpu_letterbox_.scale;
  const float y_min = (detection.y_min_px - pad_y) / bpu_letterbox_.scale;
  const float x_max = (detection.x_max_px - pad_x) / bpu_letterbox_.scale;
  const float y_max = (detection.y_max_px - pad_y) / bpu_letterbox_.scale;

  return {
      std::clamp(std::min(x_min, x_max), 0.0F, image_width_f),
      std::clamp(std::min(y_min, y_max), 0.0F, image_height_f),
      std::clamp(std::max(x_min, x_max), 0.0F, image_width_f),
      std::clamp(std::max(y_min, y_max), 0.0F, image_height_f)};
}

QrVisionNode::BpuImageRect QrVisionNode::expandImageRect(
    const BpuImageRect &image_rect,
    float padding_ratio,
    int image_width,
    int image_height) const
{
  const float width_px = image_rect.x_max - image_rect.x_min;
  const float height_px = image_rect.y_max - image_rect.y_min;

  if (width_px <= 1.0F || height_px <= 1.0F || image_width <= 0 || image_height <= 0) {
    return {};
  }

  const float pad_x_px = width_px * padding_ratio;
  const float pad_y_px = height_px * padding_ratio;
  const float image_width_f = static_cast<float>(image_width);
  const float image_height_f = static_cast<float>(image_height);

  return {
      std::clamp(image_rect.x_min - pad_x_px, 0.0F, image_width_f),
      std::clamp(image_rect.y_min - pad_y_px, 0.0F, image_height_f),
      std::clamp(image_rect.x_max + pad_x_px, 0.0F, image_width_f),
      std::clamp(image_rect.y_max + pad_y_px, 0.0F, image_height_f)};
}

void QrVisionNode::recognizeShelfTagFromDetections(
    const cv::Mat &color_image,
    const std::vector<BpuYoloDetection> &detections,
    const char *input_mode)
{
  last_ocr_regions_.clear();

  if (!bpu_ocr_pipeline_ || color_image.empty()) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const BpuYoloDetection *best_shelf_tag = nullptr;

  for (const BpuYoloDetection &detection : detections) {
    if (detection.class_id != kYoloClassShelfTag) {
      continue;
    }

    if (best_shelf_tag == nullptr || detection.score > best_shelf_tag->score) {
      best_shelf_tag = &detection;
    }
  }

  if (best_shelf_tag == nullptr) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const BpuImageRect image_rect = mapBpuDetectionToImageRect(
      *best_shelf_tag,
      color_image.cols,
      color_image.rows);
  const BpuImageRect padded_rect = expandImageRect(
      image_rect,
      ocr_yolo_padding_ratio_,
      color_image.cols,
      color_image.rows);

  if (padded_rect.x_max - padded_rect.x_min <= 1.0F ||
      padded_rect.y_max - padded_rect.y_min <= 1.0F) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const std::array<cv::Point2f, 4> shelf_tag_box = {
      cv::Point2f(padded_rect.x_min, padded_rect.y_min),
      cv::Point2f(padded_rect.x_max, padded_rect.y_min),
      cv::Point2f(padded_rect.x_max, padded_rect.y_max),
      cv::Point2f(padded_rect.x_min, padded_rect.y_max)};

  last_ocr_regions_ = bpu_ocr_pipeline_->recognizeTextBoxes(color_image, {shelf_tag_box});

  std::vector<DecodedVisualCode> ocr_codes;
  std::string ocr_text = "none";
  float ocr_score = 0.0F;

  if (!last_ocr_regions_.empty()) {
    const OcrTextRegion &region = last_ocr_regions_.front();
    ocr_text = region.text;
    ocr_score = region.score;
    ocr_codes.push_back({region.text, "shelf", "OCR", 0.0});
  }

  updateVisualCodeCategoryStability(ocr_codes, "shelf", shelf_code_state_, "ocr");

  const OcrTimingStats &ocr_timing = bpu_ocr_pipeline_->lastTiming();
  RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "BPU OCR shelf_tag mode=%s shelf_tag_score=%.3f roi=[%.0f,%.0f,%.0f,%.0f] "
      "text=%s text_score=%.3f rec[crop=%.2f pre=%.2f infer=%.2f decode=%.2f total=%.2f]",
      input_mode,
      best_shelf_tag->score,
      padded_rect.x_min,
      padded_rect.y_min,
      padded_rect.x_max,
      padded_rect.y_max,
      ocr_text.c_str(),
      ocr_score,
      ocr_timing.rec_crop_ms,
      ocr_timing.rec_preprocess_ms,
      ocr_timing.rec_infer_ms,
      ocr_timing.rec_decode_ms,
      ocr_timing.rec_total_ms);
}

void QrVisionNode::drawBpuDetections(cv::Mat &display) const
{
  for (const BpuYoloDetection &detection : last_bpu_detections_) {
    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        display.cols,
        display.rows);

    if (image_rect.x_max - image_rect.x_min <= 1.0F ||
        image_rect.y_max - image_rect.y_min <= 1.0F) {
      continue;
    }

    const cv::Point top_left(
        static_cast<int>(image_rect.x_min),
        static_cast<int>(image_rect.y_min));
    const cv::Point bottom_right(
        static_cast<int>(image_rect.x_max),
        static_cast<int>(image_rect.y_max));

    cv::Scalar box_color(0, 255, 0);

    if (detection.class_id == kYoloClassQrcode) {
      box_color = cv::Scalar(120, 220, 255);
    } else if (detection.class_id == kYoloClassPackage) {
      box_color = cv::Scalar(120, 255, 160);
    } else if (detection.class_id == kYoloClassShelfTag) {
      box_color = cv::Scalar(255, 210, 120);
    }

    cv::rectangle(
        display,
        top_left,
        bottom_right,
        box_color,
        2);

    const std::string label = cv::format(
        "%s %.2f",
        BpuYoloDetector::className(detection.class_id),
        detection.score);
    const int label_y = top_left.y > 18 ? top_left.y - 6 : top_left.y + 18;

    cv::putText(
        display,
        label,
        cv::Point(top_left.x, label_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        box_color,
        1);
  }
}

void QrVisionNode::drawOcrRegions(cv::Mat &display) const
{
  for (const OcrTextRegion &region : last_ocr_regions_) {
    std::vector<cv::Point> polygon;
    polygon.reserve(region.box.size());

    for (const cv::Point2f &point : region.box) {
      polygon.emplace_back(
          static_cast<int>(std::round(point.x)),
          static_cast<int>(std::round(point.y)));
    }

    const cv::Point *points = polygon.data();
    const int point_count = static_cast<int>(polygon.size());

    cv::polylines(
        display,
        &points,
        &point_count,
        1,
        true,
        cv::Scalar(255, 180, 80),
        2);

    if (polygon.empty()) {
      continue;
    }

    cv::putText(
        display,
        cv::format("%s %.2f", region.text.c_str(), region.score),
        cv::Point(polygon.front().x, std::max(18, polygon.front().y - 6)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 200, 120),
      1);
  }
}

void QrVisionNode::drawQrPreprocessPreview(cv::Mat &display) const
{
  if (display.empty() || debug_qr_preprocess_preview_.empty()) {
    return;
  }

  cv::Mat preview_bgr;

  if (debug_qr_preprocess_preview_.channels() == 1) {
    cv::cvtColor(debug_qr_preprocess_preview_, preview_bgr, cv::COLOR_GRAY2BGR);
  } else {
    preview_bgr = debug_qr_preprocess_preview_.clone();
  }

  if (preview_bgr.empty()) {
    return;
  }

  const double scale = std::min(
      static_cast<double>(kQrPreprocessPreviewMaxSizePx) / static_cast<double>(preview_bgr.cols),
      static_cast<double>(kQrPreprocessPreviewMaxSizePx) / static_cast<double>(preview_bgr.rows));
  const int preview_width = std::max(1, static_cast<int>(std::round(static_cast<double>(preview_bgr.cols) * scale)));
  const int preview_height = std::max(1, static_cast<int>(std::round(static_cast<double>(preview_bgr.rows) * scale)));

  cv::Mat resized_preview;
  cv::resize(
      preview_bgr,
      resized_preview,
      cv::Size(preview_width, preview_height),
      0.0,
      0.0,
      cv::INTER_NEAREST);

  const int margin_px = 16;
  const int label_height_px = 24;
  const int panel_width = resized_preview.cols + margin_px;
  const int panel_height = resized_preview.rows + label_height_px + margin_px;
  const int panel_x = std::max(0, display.cols - panel_width - margin_px);
  const int panel_y = std::max(0, display.rows - panel_height - margin_px);
  const cv::Rect panel_rect(
      panel_x,
      panel_y,
      std::min(panel_width, display.cols - panel_x),
      std::min(panel_height, display.rows - panel_y));

  if (panel_rect.width <= margin_px || panel_rect.height <= label_height_px) {
    return;
  }

  cv::Mat overlay = display.clone();
  cv::rectangle(
      overlay,
      panel_rect,
      cv::Scalar(20, 20, 20),
      cv::FILLED);
  cv::addWeighted(overlay, 0.58, display, 0.42, 0.0, display);

  const std::string label = debug_qr_preprocess_mode_.empty()
      ? std::string("QR preprocess")
      : cv::format("QR preprocess: %s", debug_qr_preprocess_mode_.c_str());
  cv::putText(
      display,
      label,
      cv::Point(panel_x + 8, panel_y + 17),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(120, 220, 255),
      1);

  const cv::Rect image_rect(
      panel_x + 8,
      panel_y + label_height_px,
      std::min(resized_preview.cols, display.cols - panel_x - 8),
      std::min(resized_preview.rows, display.rows - panel_y - label_height_px - 8));

  if (image_rect.width <= 0 || image_rect.height <= 0) {
    return;
  }

  resized_preview(cv::Rect(0, 0, image_rect.width, image_rect.height))
      .copyTo(display(image_rect));
  cv::rectangle(
      display,
      image_rect,
      cv::Scalar(120, 220, 255),
      1);
}
#endif

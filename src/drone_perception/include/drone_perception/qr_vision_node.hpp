#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/types.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "drone_msgs/msg/barcode_capture.hpp"
#include "drone_perception/depth_processor.hpp"

#ifndef DRONE_PERCEPTION_HAS_BPU
#define DRONE_PERCEPTION_HAS_BPU 0
#endif

#ifndef DRONE_PERCEPTION_HAS_RKNN
#define DRONE_PERCEPTION_HAS_RKNN 0
#endif

#if DRONE_PERCEPTION_HAS_BPU
#include "drone_perception/bpu_ocr_pipeline.hpp"
#include "drone_perception/bpu_yolo_detector.hpp"
#endif

#if DRONE_PERCEPTION_HAS_RKNN
#include "drone_perception/detection.hpp"
#include "drone_perception/rknn_yolo_detector.hpp"
#endif

class QrVisionNode : public rclcpp::Node
{
public:
  QrVisionNode();

  ~QrVisionNode() override;

private:
  struct DecodedVisualCode
  {
    std::string code;
    std::string category;
    std::string symbol_type;
    double center_distance_sq{0.0};
  };

  struct CodeStabilityState
  {
    std::string candidate_code;
    std::string candidate_symbol_type;
    std::string stable_code;
    std::string stable_symbol_type;
    int candidate_count{0};
    int lost_count{0};
  };

  struct BpuImageRect
  {
    float x_min{0.0F};
    float y_min{0.0F};
    float x_max{0.0F};
    float y_max{0.0F};
  };

  struct BpuLetterboxState
  {
    float scale{1.0F};
    int pad_x{0};
    int pad_y{0};
  };

  struct CaptureFrameCandidate
  {
    cv::Mat image;
    rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
    float package_score{0.0F};
    double center_distance_sq{0.0};
    bool valid{false};
  };

  struct CameraNumericControl
  {
    std::string parameter_name;
    std::string label;
    int fallback_min{0};
    int fallback_max{100};
    int fallback_default{0};
    int min_value{0};
    int max_value{100};
    int current_value{0};
    int last_sent_value{0};
    int trackbar_value{0};
    rclcpp::ParameterType parameter_type{rclcpp::ParameterType::PARAMETER_INTEGER};
    bool enabled{true};
    bool send_pending{false};
    int restore_value{0};
    bool restore_value_initialized{false};
  };

  struct CameraBoolControl
  {
    std::string parameter_name;
    std::string label;
    bool current_value{false};
    bool last_sent_value{false};
    bool enabled{true};
    bool send_pending{false};
    bool restore_value{false};
    bool restore_value_initialized{false};
    cv::Rect true_rect{};
    cv::Rect false_rect{};
  };

  struct CameraTrackbarContext
  {
    QrVisionNode *node{nullptr};
    std::size_t index{0U};
  };

  static constexpr std::size_t kCameraNumericControlCount = 3U;
  static constexpr std::size_t kCameraBoolControlCount = 3U;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
      ColorDepthSyncPolicy;

  void declareParameters();

  void initializeSubscriptions();

  void initializeBpuDetector();

  void initializeBpuOcrPipeline();

  void initializeRknnDetector();

  bool prepareBpuInput(const cv::Mat &color_image);

  void updateVisualCodeStability(const std::vector<DecodedVisualCode> &decoded_codes);

  std::string composeCaptureBarcode(bool require_complete) const;

  std::string composeCaptureBarcodeWithNan(bool allow_all_nan) const;

  void handleHoverActive(const std_msgs::msg::Bool::SharedPtr msg);

  void resetHoverCaptureState();

  void publishBarcodeCapture(const cv::Mat &color_image);

  void publishTextOnlyCapture(const std::string &barcode);

  bool publishFullFrameCapture(
      const std::string &barcode,
      const cv::Mat &color_image);

  void updateVisualCodeCategoryStability(
      const std::vector<DecodedVisualCode> &decoded_codes,
      const std::string &category,
      CodeStabilityState &state,
      const char *source_name);

  void handleSyncedFrame(
      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg);

  void handleColorFrame(
      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg);

  void handleRgbdFrame(
      const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &rgbd_msg);

  void processFrame(
      const cv_bridge::CvImageConstPtr &color_bridge,
      const cv_bridge::CvImageConstPtr &depth_bridge,
      const std::chrono::steady_clock::time_point &callback_t0,
      const char *input_mode);

  void handleCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg);

  void displayDebugFrame(
      const cv::Mat &color_image,
      const DepthSampleResult &center_depth);

  void initializeCameraControls();

  void initializeCameraControlDefaults();

  void drawCameraControlsPanel(cv::Mat &display);

  void updateCameraControlClient();

  void requestCameraControlDescriptors();

  void requestCameraControlValues();

  void applyCameraControlDescriptors(
      const std::vector<rcl_interfaces::msg::ParameterDescriptor> &descriptors);

  void applyCameraControlValues(
      const std::vector<rclcpp::Parameter> &parameters);

  void updateCameraControlEnableStates();

  void resetCameraControlsToDefaults();

  void sendPendingCameraControlParameters();

  void handleCameraParameterSetResults(
      const std::vector<rclcpp::Parameter> &parameters,
      const std::vector<rcl_interfaces::msg::SetParametersResult> &results);

  void handleCameraTrackbarChange(std::size_t index, int position);

  void handleCameraControlsClick(int x, int y);

  void setCameraBoolControl(std::size_t index, bool value);

  void updateCameraTrackbarPosition(std::size_t index);

  void drawCameraBoolControl(
      cv::Mat &panel,
      CameraBoolControl &control,
      int y);

  void drawCameraResetButton(cv::Mat &panel);

  static void handleCameraTrackbarCallback(int position, void *userdata);

  static void handleCameraControlsMouseCallback(
      int event,
      int x,
      int y,
      int flags,
      void *userdata);

#if DRONE_PERCEPTION_HAS_BPU
  void drawBpuDetections(cv::Mat &display) const;

  void resetCaptureCandidateState();

  void bufferPackageCaptureCandidate(const cv::Mat &color_image);

  bool publishBestBufferedCapture();

  bool selectBestPackageInDeadzone(
      const cv::Mat &color_image,
      CaptureFrameCandidate &candidate) const;

  BpuImageRect mapBpuDetectionToImageRect(
      const BpuYoloDetection &detection,
      int image_width,
      int image_height) const;

  BpuImageRect expandImageRect(
      const BpuImageRect &image_rect,
      float padding_ratio,
      int image_width,
      int image_height) const;

  void recognizeShelfTagFromDetections(
      const cv::Mat &color_image,
      const std::vector<BpuYoloDetection> &detections,
      const char *input_mode);

  std::vector<DecodedVisualCode> decodeVisualCodesFromDetections(
      const cv::Mat &color_image,
      const std::vector<BpuYoloDetection> &detections);

  bool encodeFrameCaptureJpeg(
      const cv::Mat &color_image,
      std::vector<uint8_t> &jpeg_data);

  void drawOcrRegions(cv::Mat &display) const;
#endif

  // BPU / RKNN 共用：右下角显示当前 ZBar 预处理阶段预览
  void drawQrPreprocessPreview(cv::Mat &display) const;

#if DRONE_PERCEPTION_HAS_RKNN
  // 对齐 package_qr_shelf_rknn_probe：3 worker × 3 NPU + 队列丢旧 + stale 丢弃
  // 深度：软对齐（color 驱动 + 最新 depth，时间窗匹配），不是硬同步、也不是永久丢弃深度
  static constexpr std::size_t kRknnWorkerCount = 3;
  static constexpr std::size_t kRknnTaskQueueCapacity = 3;
  static constexpr std::int64_t kDepthSoftMatchNs = 50'000'000;  // 50ms

  struct RknnFrameTask
  {
    std::uint64_t frame_id = 0;
    sensor_msgs::msg::Image::ConstSharedPtr color;
    sensor_msgs::msg::Image::ConstSharedPtr depth;  // 软附着的最新 depth
  };

  struct RknnInferResult
  {
    std::uint64_t frame_id = 0;
    std::size_t worker_index = 0;
    std::vector<Detection> detections;
    RknnYoloDetector::InferenceTimingStats timing;
    DepthSampleResult center_depth;
    cv::Mat bgr_image;
  };

  void startRknnWorkers();
  void stopRknnWorkers();
  void rknnWorkerLoop(std::size_t worker_index);
  void enqueueRknnColorFrame(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg);
  void handleDepthFrameSoft(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg);
  void consumeRknnResults();
  static bool depthSoftMatchesColor(
      const sensor_msgs::msg::Image &color,
      const sensor_msgs::msg::Image &depth);

  std::vector<DecodedVisualCode> decodeVisualCodesFromRknnDetections(
      const cv::Mat &color_image,
      const std::vector<Detection> &detections);

  void drawRknnDetections(
      cv::Mat &display,
      const DepthSampleResult &center_depth) const;

  void drawRknnProbeHud(
      cv::Mat &display,
      const DepthSampleResult &center_depth) const;
#endif

  void updateFps();

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string rgbd_topic_;
  std::string window_name_;
  std::string camera_param_node_;
  std::string barcode_capture_topic_;
  std::string hover_active_topic_;

  std::string bpu_model_path_;
  std::string ocr_rec_model_path_;
  std::string rknn_model_path_;

  std::vector<uint8_t> bpu_input_nv12_;
  BpuLetterboxState bpu_letterbox_;

  bool debug_view_ = true;
  bool enable_bpu_ = false;
  bool enable_bpu_ocr_ = false;
  bool enable_rknn_ = false;
  bool require_depth_ = false;
  bool require_camera_info_ = true;
  bool use_rgbd_ = false;
  bool qr_preprocess_enabled_ = true;
  bool hover_active_ = false;
  bool full_capture_sent_in_hover_ = false;
  bool camera_controls_enabled_ = false;
  mutable bool debug_window_created_ = false;
  bool camera_controls_attached_ = false;
  bool camera_controls_updating_trackbar_ = false;
  bool camera_param_node_online_ = false;
  bool camera_param_describe_requested_ = false;
  bool camera_param_describe_in_flight_ = false;
  bool camera_param_get_requested_ = false;
  bool camera_param_get_in_flight_ = false;
  bool camera_param_set_in_flight_ = false;
  int log_throttle_ms_ = 500;
  int camera_controls_update_period_ms_ = 200;
  int sample_radius_px_ = 10;
  int shelf_code_stable_frames_ = 3;
  int shelf_code_lost_tolerance_frames_ = 2;
  int barcode_capture_jpeg_quality_ = 90;
  float ocr_yolo_padding_ratio_ = 0.15F;
  float package_capture_padding_ratio_ = 0.05F;
  double capture_collection_duration_s_ = 2.0;
  double package_capture_deadzone_ratio_ = 0.08;
  double package_capture_min_score_ = 0.50;

  rclcpp::Time last_frame_time_;
  double smoothed_fps_ = 0.0;
  double last_callback_ms_ = 0.0;
  std::string last_input_mode_{"color"};
  std::string last_published_package_barcode_;

  CodeStabilityState shelf_code_state_;
  CodeStabilityState sku_code_state_;
  CodeStabilityState pkg_code_state_;

  DepthProcessor depth_processor_;

  rclcpp::AsyncParametersClient::SharedPtr camera_param_client_;
  rclcpp::TimerBase::SharedPtr camera_controls_timer_;
  std::array<CameraNumericControl, kCameraNumericControlCount> camera_numeric_controls_{};
  std::array<CameraBoolControl, kCameraBoolControlCount> camera_bool_controls_{};
  std::array<CameraTrackbarContext, kCameraNumericControlCount> camera_trackbar_contexts_{};
  cv::Rect camera_controls_panel_rect_{};
  cv::Rect camera_reset_button_rect_{};
  rclcpp::Time last_camera_param_send_time_;
  std::string camera_controls_status_text_;

#if DRONE_PERCEPTION_HAS_BPU
  std::unique_ptr<BpuYoloDetector> bpu_detector_;
  std::unique_ptr<BpuOcrPipeline> bpu_ocr_pipeline_;
  std::vector<BpuYoloDetection> last_bpu_detections_;
  std::vector<OcrTextRegion> last_ocr_regions_;
  rclcpp::Time hover_capture_start_time_;
  CaptureFrameCandidate best_package_capture_candidate_;
  int package_capture_candidate_count_{0};
#endif

#if DRONE_PERCEPTION_HAS_RKNN
  std::array<std::unique_ptr<RknnYoloDetector>, kRknnWorkerCount> rknn_detectors_;
  std::array<std::thread, kRknnWorkerCount> rknn_workers_;
  std::array<std::atomic<double>, kRknnWorkerCount> rknn_core_run_ms_{};
  std::atomic<bool> rknn_running_{false};
  std::mutex rknn_task_mutex_;
  std::condition_variable rknn_task_cv_;
  std::deque<RknnFrameTask> rknn_task_queue_;
  std::mutex rknn_result_mutex_;
  std::optional<RknnInferResult> rknn_latest_result_;
  std::uint64_t rknn_next_frame_id_ = 1;
  std::uint64_t rknn_consumed_frame_id_ = 0;
  std::atomic<std::uint64_t> rknn_dropped_count_{0};
  std::atomic<std::uint64_t> rknn_stale_count_{0};
  std::atomic<std::uint64_t> rknn_processed_count_{0};
  sensor_msgs::msg::Image::ConstSharedPtr rknn_latest_depth_;
  std::mutex rknn_depth_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rknn_depth_sub_;
  rclcpp::TimerBase::SharedPtr rknn_consume_timer_;
  std::vector<Detection> last_rknn_detections_;
  RknnYoloDetector::InferenceTimingStats last_rknn_timing_{};
  DepthSampleResult last_rknn_center_depth_{};
  double rknn_weight_mib_ = 0.0;
  double rknn_internal_mib_ = 0.0;
  double rknn_dma_mib_ = 0.0;
#endif

  // ZBar 调试字段（BPU / RKNN 共用）
  std::string debug_raw_symbol_;
  std::string debug_raw_symbol_type_;
  cv::Mat debug_qr_preprocess_preview_;
  std::string debug_qr_preprocess_mode_;

  bool has_camera_info_ = false;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ColorDepthSyncPolicy>> color_depth_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_only_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hover_active_sub_;
  rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr barcode_capture_pub_;
};

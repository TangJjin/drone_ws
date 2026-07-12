#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sched.h>

#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <librealsense2/rsutil.h>

#include "rknn_api.h"
#include "drone_perception/depth_processor.hpp"
#include "drone_perception/rknn_yolo_detector.hpp"

namespace
{

std::vector<std::uint8_t> readBinaryFile(const std::string &path)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("model path is not a regular file: " + path);
  }

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("failed to open model: " + path);
  }

  const std::streamsize size = file.tellg();
  if (size <= 0) {
    throw std::runtime_error("model is empty: " + path);
  }

  std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
  file.seekg(0, std::ios::beg);
  if (!file.read(reinterpret_cast<char *>(data.data()), size)) {
    throw std::runtime_error("failed to read model: " + path);
  }
  return data;
}

void checkRknn(int result, const std::string &operation)
{
  if (result != RKNN_SUCC) {
    throw std::runtime_error(operation + " failed, ret=" + std::to_string(result));
  }
}

void bindToPerformanceCpus()
{
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  for (int cpu = 4; cpu <= 7; ++cpu) {
    CPU_SET(cpu, &cpu_set);
  }
  if (sched_setaffinity(0, sizeof(cpu_set), &cpu_set) != 0) {
    throw std::runtime_error(
      "failed to bind CPU affinity to cores 4-7: " +
      std::string(std::strerror(errno)));
  }
  std::cout << "CPU affinity : cores 4-7\n";
}

void printTensor(const char *kind, const rknn_tensor_attr &attr)
{
  std::cout << kind << '[' << attr.index << "]\n"
            << "  name             : " << attr.name << '\n'
            << "  dimensions       : [";

  for (std::uint32_t i = 0; i < attr.n_dims; ++i) {
    std::cout << attr.dims[i] << (i + 1U < attr.n_dims ? ", " : "");
  }

  std::cout << "]\n"
            << "  element count    : " << attr.n_elems << '\n'
            << "  byte size        : " << attr.size << '\n'
            << "  size with stride : " << attr.size_with_stride << '\n'
            << "  width stride     : " << attr.w_stride << '\n'
            << "  format           : " << get_format_string(attr.fmt)
            << " (" << static_cast<int>(attr.fmt) << ")\n"
            << "  type             : " << get_type_string(attr.type)
            << " (" << static_cast<int>(attr.type) << ")\n"
            << "  quantization     : " << get_qnt_type_string(attr.qnt_type)
            << " (" << static_cast<int>(attr.qnt_type) << ")\n"
            << "  zero point       : " << attr.zp << '\n'
            << "  scale            : " << attr.scale << '\n'
            << "  fractional length: " << static_cast<int>(attr.fl) << "\n\n";
}

class RknnContext
{
public:
  RknnContext() = default;
  RknnContext(const RknnContext &) = delete;
  RknnContext &operator=(const RknnContext &) = delete;

  ~RknnContext()
  {
    if (context_ != 0) {
      rknn_destroy(context_);
    }
  }

  rknn_context *address() { return &context_; }
  rknn_context get() const { return context_; }

private:
  rknn_context context_ = 0;
};

class FirstColorFrameProbe : public rclcpp::Node
{
public:
  FirstColorFrameProbe()
  : Node("rknn_d435i_color_probe")
  {
    const std::string topic = declare_parameter<std::string>(
      "color_topic", "/camera/camera/color/image_raw");
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr message) {
        handleFrame(message, topic);
      });

    RCLCPP_INFO(get_logger(), "Waiting for the first D435i color frame on %s", topic.c_str());
  }

private:
  void handleFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &message,
    const std::string &topic)
  {
    try {
      const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(message, "bgr8");
      RCLCPP_INFO(
        get_logger(),
        "D435i color frame received: topic=%s source_encoding=%s converted_encoding=bgr8 "
        "width=%u height=%u source_step=%u cv_step=%zu channels=%d",
        topic.c_str(), message->encoding.c_str(), message->width, message->height,
        message->step, static_cast<std::size_t>(image->image.step), image->image.channels());
      rclcpp::shutdown();
    } catch (const cv_bridge::Exception &error) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion to bgr8 failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

class D435iRknnStream : public rclcpp::Node
{
public:
  explicit D435iRknnStream(const std::string &model_path)
  : Node("rknn_d435i_stream"),
    started_at_(Clock::now())
  {
    constexpr std::array<rknn_core_mask, kWorkerCount> core_masks{
      RKNN_NPU_CORE_0, RKNN_NPU_CORE_1, RKNN_NPU_CORE_2};
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      detectors_[index] = std::make_unique<RknnYoloDetector>(model_path, core_masks[index]);
      const rknn_mem_size memory = detectors_[index]->memorySize();
      weight_mib_ += static_cast<double>(memory.total_weight_size) / (1024.0 * 1024.0);
      internal_mib_ += static_cast<double>(memory.total_internal_size) / (1024.0 * 1024.0);
      dma_mib_ += static_cast<double>(memory.total_dma_allocated_size) / (1024.0 * 1024.0);
    }

    const std::string color_topic = declare_parameter<std::string>(
      "color_topic", "/camera/camera/color/image_raw");
    const std::string depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    const std::string color_info_topic = declare_parameter<std::string>(
      "color_info_topic", "/camera/camera/color/camera_info");
    const std::string depth_info_topic = declare_parameter<std::string>(
      "depth_info_topic", "/camera/camera/depth/camera_info");
    const std::string depth_to_color_topic = declare_parameter<std::string>(
      "depth_to_color_topic", "/camera/camera/extrinsics/depth_to_color");

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) { receiveColor(message); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) { receiveDepth(message); });
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) { receiveColorInfo(message); });
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) { receiveDepthInfo(message); });
    const auto extrinsics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    extrinsics_sub_ = create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
      depth_to_color_topic, extrinsics_qos,
      [this](const realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr message) {
        receiveExtrinsics(message);
      });
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      workers_[index] = std::thread(&D435iRknnStream::workerLoop, this, index);
    }
    ui_thread_ = std::thread(&D435iRknnStream::uiLoop, this);
    RCLCPP_INFO(get_logger(),
      "Streaming 3-context raw D435i RKNN inference started: color=%s depth=%s model=%s",
      color_topic.c_str(), depth_topic.c_str(), model_path.c_str());
  }

  ~D435iRknnStream() override
  {
    running_.store(false);
    task_ready_.notify_all();
    result_ready_.notify_all();
    for (std::thread &worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    if (ui_thread_.joinable()) {
      ui_thread_.join();
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  static constexpr std::size_t kWorkerCount = 3;
  static constexpr std::size_t kTaskQueueCapacity = 3;

  struct FrameBundle
  {
    std::uint64_t frame_id = 0;
    Image::ConstSharedPtr color;
    Image::ConstSharedPtr depth;
    CameraInfo::ConstSharedPtr color_info;
    CameraInfo::ConstSharedPtr depth_info;
    realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr depth_to_color;
  };

  struct InferenceResult
  {
    std::uint64_t frame_id = 0;
    std::size_t worker_index = 0;
    cv::Mat display;
    std::size_t detection_count = 0;
    bool depth_ready = false;
    bool center_depth_valid = false;
    float center_depth_m = 0.0F;
    int depth_width = 0;
    int depth_height = 0;
    std::string depth_encoding;
    RknnYoloDetector::InferenceTimingStats timing;
    double api_run_ms = 0.0;
  };

  void receiveColor(const Image::ConstSharedPtr message)
  {
    const std::int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (last_received_stamp_ns_ > 0 && stamp_ns > last_received_stamp_ns_) {
      const double fps = 1.0e9 / static_cast<double>(stamp_ns - last_received_stamp_ns_);
      const double previous_fps = input_fps_.load();
      input_fps_.store(previous_fps <= 0.0 ? fps : 0.9 * previous_fps + 0.1 * fps);
    }
    last_received_stamp_ns_ = stamp_ns;
    received_count_.fetch_add(1);

    FrameBundle frame;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      frame.frame_id = next_frame_id_++;
      frame.color = message;
      frame.depth = latest_depth_;
      frame.color_info = latest_color_info_;
      frame.depth_info = latest_depth_info_;
      frame.depth_to_color = latest_depth_to_color_;
    }
    {
      std::lock_guard<std::mutex> lock(task_mutex_);
      if (task_queue_.size() >= kTaskQueueCapacity) {
        task_queue_.pop_front();
        dropped_count_.fetch_add(1);
      }
      task_queue_.push_back(std::move(frame));
    }
    task_ready_.notify_one();
  }

  void receiveDepth(const Image::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = message;
  }

  void receiveColorInfo(const CameraInfo::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_color_info_ = message;
  }

  void receiveDepthInfo(const CameraInfo::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_info_ = message;
  }

  void receiveExtrinsics(const realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_to_color_ = message;
  }

  void workerLoop(std::size_t worker_index)
  {
    while (running_.load() && rclcpp::ok()) {
      FrameBundle frame;
      {
        std::unique_lock<std::mutex> lock(task_mutex_);
        task_ready_.wait(lock, [this] {
          return !task_queue_.empty() || !running_.load() || !rclcpp::ok();
        });
        if (!running_.load() || !rclcpp::ok()) {
          break;
        }
        frame = std::move(task_queue_.front());
        task_queue_.pop_front();
      }
      processFrame(frame, worker_index);
    }
  }

  static rs2_distortion toRs2Distortion(const std::string &model)
  {
    if (model == "plumb_bob") {
      return RS2_DISTORTION_BROWN_CONRADY;
    }
    if (model == "equidistant" || model == "kannala_brandt4") {
      return RS2_DISTORTION_KANNALA_BRANDT4;
    }
    return RS2_DISTORTION_NONE;
  }

  static rs2_intrinsics toRs2Intrinsics(const CameraInfo &info)
  {
    rs2_intrinsics intrinsics{};
    intrinsics.width = static_cast<int>(info.width);
    intrinsics.height = static_cast<int>(info.height);
    intrinsics.ppx = static_cast<float>(info.k[2]);
    intrinsics.ppy = static_cast<float>(info.k[5]);
    intrinsics.fx = static_cast<float>(info.k[0]);
    intrinsics.fy = static_cast<float>(info.k[4]);
    intrinsics.model = toRs2Distortion(info.distortion_model);
    for (std::size_t i = 0; i < std::min<std::size_t>(5, info.d.size()); ++i) {
      intrinsics.coeffs[i] = static_cast<float>(info.d[i]);
    }
    return intrinsics;
  }

  static rs2_extrinsics inverseExtrinsics(const rs2_extrinsics &forward)
  {
    rs2_extrinsics inverse{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        // RealSense stores its rotation matrix in column-major order.
        inverse.rotation[column * 3 + row] = forward.rotation[row * 3 + column];
      }
    }
    for (int row = 0; row < 3; ++row) {
      inverse.translation[row] = 0.0F;
      for (int column = 0; column < 3; ++column) {
        inverse.translation[row] -=
          inverse.rotation[column * 3 + row] * forward.translation[column];
      }
    }
    return inverse;
  }

  DepthSampleResult sampleRawDepth(
    DepthProcessor &depth_processor, const cv::Mat &depth, const FrameBundle &frame,
    int color_u, int color_v) const
  {
    if (!frame.depth || !frame.color_info || !frame.depth_info || !frame.depth_to_color ||
      depth.type() != CV_16UC1)
    {
      return {};
    }
    const auto &extrinsics = *frame.depth_to_color;
    rs2_extrinsics depth_to_color{};
    std::copy(extrinsics.rotation.begin(), extrinsics.rotation.end(), depth_to_color.rotation);
    std::copy(extrinsics.translation.begin(), extrinsics.translation.end(), depth_to_color.translation);
    const rs2_extrinsics color_to_depth = inverseExtrinsics(depth_to_color);
    const rs2_intrinsics depth_intrinsics = toRs2Intrinsics(*frame.depth_info);
    const rs2_intrinsics color_intrinsics = toRs2Intrinsics(*frame.color_info);
    const float color_pixel[2] = {static_cast<float>(color_u), static_cast<float>(color_v)};
    float depth_pixel[2] = {};
    rs2_project_color_pixel_to_depth_pixel(
      depth_pixel, reinterpret_cast<const uint16_t *>(depth.data), 0.001F, 0.1F, 10.0F,
      &depth_intrinsics, &color_intrinsics, &color_to_depth, &depth_to_color, color_pixel);
    const int projected_u = static_cast<int>(std::lround(depth_pixel[0]));
    const int projected_v = static_cast<int>(std::lround(depth_pixel[1]));
    const DepthSampleResult projected = depth_processor.sampleAt(
      depth, projected_u, projected_v, sample_radius_px_);
    if (projected.has_valid_depth) {
      return projected;
    }

    // Some realsense2_camera releases publish extrinsics with a matrix layout
    // that differs from rsutil's C representation. Use calibrated intrinsics
    // as a bounded fallback rather than returning a stale or arbitrary depth.
    const float depth_u = (static_cast<float>(color_u) - color_intrinsics.ppx) /
      color_intrinsics.fx * depth_intrinsics.fx + depth_intrinsics.ppx;
    const float depth_v = (static_cast<float>(color_v) - color_intrinsics.ppy) /
      color_intrinsics.fy * depth_intrinsics.fy + depth_intrinsics.ppy;
    return depth_processor.sampleAt(
      depth, static_cast<int>(std::lround(depth_u)), static_cast<int>(std::lround(depth_v)), 20);
  }

  static bool depthMatchesColor(const FrameBundle &frame)
  {
    if (!frame.color || !frame.depth) {
      return false;
    }
    const std::int64_t color_stamp = rclcpp::Time(frame.color->header.stamp).nanoseconds();
    const std::int64_t depth_stamp = rclcpp::Time(frame.depth->header.stamp).nanoseconds();
    constexpr std::int64_t kMaxDepthOffsetNs = 50'000'000;
    return std::llabs(color_stamp - depth_stamp) <= kMaxDepthOffsetNs;
  }

  void processFrame(const FrameBundle &frame, std::size_t worker_index)
  {
    try {
      RknnYoloDetector &detector = *detectors_[worker_index];
      DepthProcessor &depth_processor = depth_processors_[worker_index];
      const auto image = cv_bridge::toCvShare(frame.color, "bgr8");
      const bool depth_ready = frame.depth && frame.color_info && frame.depth_info &&
        frame.depth_to_color && depthMatchesColor(frame);
      if (!depth_ready) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Raw depth unavailable, stale, or missing calibration; continuing with 2D detection");
      }
      const cv_bridge::CvImageConstPtr depth = depth_ready
        ? cv_bridge::toCvShare(frame.depth)
        : cv_bridge::CvImageConstPtr{};
      const std::vector<Detection> detections = detector.infer(image->image);

      cv::Mat display = image->image.clone();

      for (Detection detection : detections) {
        const DepthSampleResult sample = depth_ready
          ? sampleRawDepth(
            depth_processor, depth->image, frame, detection.center.x, detection.center.y)
          : DepthSampleResult{};
        detection.has_depth = sample.has_valid_depth;
        detection.depth_m = sample.depth_m;
        if (sample.has_valid_depth) {
          detection.point_3d = depth_processor.projectTo3D(
            detection.center.x, detection.center.y, sample.depth_m, *frame.color_info);
        }
        cv::rectangle(display, detection.box, cv::Scalar(0, 255, 0), 2);
        const std::string label = sample.has_valid_depth
          ? cv::format("class %d %.2f  %.2fm XYZ(%.2f,%.2f,%.2f)",
            detection.class_id, detection.score, sample.depth_m, detection.point_3d.x,
            detection.point_3d.y, detection.point_3d.z)
          : cv::format("class %d %.2f  depth n/a", detection.class_id, detection.score);
        const int label_y = std::max(20, detection.box.y - 6);
        cv::putText(display, label, cv::Point(detection.box.x, label_y),
          cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
      }

      const DepthSampleResult center_depth = depth_ready
        ? sampleRawDepth(
          depth_processor, depth->image, frame, display.cols / 2, display.rows / 2)
        : DepthSampleResult{};
      InferenceResult result;
      result.frame_id = frame.frame_id;
      result.worker_index = worker_index;
      result.display = std::move(display);
      result.detection_count = detections.size();
      result.depth_ready = depth_ready;
      result.center_depth_valid = center_depth.has_valid_depth;
      result.center_depth_m = center_depth.depth_m;
      if (depth_ready) {
        result.depth_width = depth->image.cols;
        result.depth_height = depth->image.rows;
        result.depth_encoding = frame.depth->encoding;
      }
      result.timing = detector.lastTiming();
      result.api_run_ms = detector.lastRknnRunMs();
      core_run_ms_[worker_index].store(result.api_run_ms);
      processed_count_.fetch_add(1);

      {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result.frame_id <= displayed_frame_id_) {
          stale_result_count_.fetch_add(1);
          return;
        }
        if (latest_result_ && result.frame_id <= latest_result_->frame_id) {
          stale_result_count_.fetch_add(1);
          return;
        }
        latest_result_ = std::move(result);
      }
      result_ready_.notify_one();
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "stream inference failed: %s", error.what());
    }
  }

  void uiLoop()
  {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    while (running_.load() && rclcpp::ok()) {
      std::optional<InferenceResult> result;
      {
        std::unique_lock<std::mutex> lock(result_mutex_);
        result_ready_.wait_for(lock, std::chrono::milliseconds(20), [this] {
          return latest_result_.has_value() || !running_.load() || !rclcpp::ok();
        });
        if (latest_result_) {
          result = std::move(latest_result_);
          latest_result_.reset();
          if (result->frame_id <= displayed_frame_id_) {
            stale_result_count_.fetch_add(1);
            result.reset();
          } else {
            displayed_frame_id_ = result->frame_id;
          }
        }
      }

      if (result) {
        displayResult(*result);
      }
      const int key = cv::waitKey(1) & 0xff;
      if (key == 'q' || key == 27) {
        rclcpp::shutdown();
        break;
      }
    }
    cv::destroyWindow(window_name_);
  }

  void displayResult(InferenceResult &result)
  {
    const auto now = Clock::now();
    if (last_display_at_ != Clock::time_point{}) {
      const double interval = std::chrono::duration<double>(now - last_display_at_).count();
      if (interval > 0.0) {
        const double current_fps = 1.0 / interval;
        display_fps_ = display_fps_ <= 0.0 ? current_fps :
          0.9 * display_fps_ + 0.1 * current_fps;
      }
    }
    last_display_at_ = now;

    double npu_capacity_fps = 0.0;
    std::array<double, kWorkerCount> run_ms{};
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      run_ms[index] = core_run_ms_[index].load();
      if (run_ms[index] > 0.0) {
        npu_capacity_fps += 1000.0 / run_ms[index];
      }
    }

    const cv::Scalar text_color(255, 0, 255);
    const std::string status = cv::format(
      "Input %.1f  Display %.1f  NPU cap %.1f FPS  Drop %llu  Stale %llu",
      input_fps_.load(), display_fps_, npu_capacity_fps,
      static_cast<unsigned long long>(dropped_count_.load()),
      static_cast<unsigned long long>(stale_result_count_.load()));
    cv::putText(result.display, status, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string core_status = cv::format(
      "C0 %.2fms  C1 %.2fms  C2 %.2fms  latest frame %llu via C%zu",
      run_ms[0], run_ms[1], run_ms[2],
      static_cast<unsigned long long>(result.frame_id), result.worker_index);
    cv::putText(result.display, core_status, cv::Point(12, 56), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string stream_status = result.depth_ready
      ? cv::format("RGB %dx%d  raw Depth %dx%d %s", result.display.cols, result.display.rows,
        result.depth_width, result.depth_height, result.depth_encoding.c_str())
      : cv::format("RGB %dx%d  raw Depth unavailable/stale",
        result.display.cols, result.display.rows);
    cv::putText(result.display, stream_status, cv::Point(12, 82), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string depth_status = result.center_depth_valid
      ? cv::format("Center depth %.3fm  raw-depth registered in node", result.center_depth_m)
      : std::string("Center depth n/a  raw-depth registration");
    cv::putText(result.display, depth_status, cv::Point(12, 108), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string memory_status = cv::format(
      "3-context RKNN memory: weight %.1f MiB  internal %.1f MiB  DMA %.1f MiB",
      weight_mib_, internal_mib_, dma_mib_);
    cv::putText(result.display, memory_status, cv::Point(12, 134), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    cv::imshow(window_name_, result.display);

    const double report_seconds = std::chrono::duration<double>(now - last_report_at_).count();
    if (report_seconds >= 1.0) {
      const std::uint64_t processed = processed_count_.load();
      const double process_fps = static_cast<double>(processed - last_report_processed_) /
        report_seconds;
      RCLCPP_INFO(get_logger(),
        "parallel frames=%llu received=%llu queue_drop=%llu stale_result=%llu "
        "input_fps=%.2f process_fps=%.2f display_fps=%.2f npu_capacity_fps=%.2f "
        "core_ms=[%.2f,%.2f,%.2f] latest_frame=%llu worker=%zu detections=%zu",
        static_cast<unsigned long long>(processed),
        static_cast<unsigned long long>(received_count_.load()),
        static_cast<unsigned long long>(dropped_count_.load()),
        static_cast<unsigned long long>(stale_result_count_.load()),
        input_fps_.load(), process_fps, display_fps_, npu_capacity_fps,
        run_ms[0], run_ms[1], run_ms[2],
        static_cast<unsigned long long>(result.frame_id), result.worker_index,
        result.detection_count);
      last_report_processed_ = processed;
      last_report_at_ = now;
    }
  }

  std::array<std::unique_ptr<RknnYoloDetector>, kWorkerCount> detectors_;
  std::array<DepthProcessor, kWorkerCount> depth_processors_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_sub_;
  std::mutex data_mutex_;
  Image::ConstSharedPtr latest_depth_;
  CameraInfo::ConstSharedPtr latest_color_info_;
  CameraInfo::ConstSharedPtr latest_depth_info_;
  realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr latest_depth_to_color_;
  std::mutex task_mutex_;
  std::condition_variable task_ready_;
  std::deque<FrameBundle> task_queue_;
  std::mutex result_mutex_;
  std::condition_variable result_ready_;
  std::optional<InferenceResult> latest_result_;
  std::array<std::thread, kWorkerCount> workers_;
  std::thread ui_thread_;
  std::atomic<bool> running_{true};
  std::atomic<std::uint64_t> received_count_{0};
  std::atomic<std::uint64_t> dropped_count_{0};
  std::atomic<std::uint64_t> stale_result_count_{0};
  std::atomic<std::uint64_t> processed_count_{0};
  std::array<std::atomic<double>, kWorkerCount> core_run_ms_{};
  std::uint64_t next_frame_id_ = 1;
  std::uint64_t displayed_frame_id_ = 0;
  int sample_radius_px_ = 10;
  Clock::time_point started_at_;
  Clock::time_point last_report_at_ = started_at_;
  Clock::time_point last_display_at_{};
  std::uint64_t last_report_processed_ = 0;
  double display_fps_ = 0.0;
  std::atomic<double> input_fps_{0.0};
  std::int64_t last_received_stamp_ns_ = 0;
  double weight_mib_ = 0.0;
  double internal_mib_ = 0.0;
  double dma_mib_ = 0.0;
  const std::string window_name_ = "D435i RKNN Detection";
};

}  // namespace

int main(int argc, char **argv)
{
  try {
    bindToPerformanceCpus();
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  if (argc == 1) {
    const std::string model_path =
      ament_index_cpp::get_package_share_directory("drone_perception") +
      "/models/qr_rk3588_hybrid_bbox_fp16.rknn";
    rclcpp::init(argc, argv);
    try {
      rclcpp::spin(std::make_shared<D435iRknnStream>(model_path));
    } catch (const std::exception &error) {
      std::cerr << "Error: " << error.what() << '\n';
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::shutdown();
    return 0;
  }

  if (argc >= 3 && std::string(argv[1]) == "--infer") {
    const std::string model_path = argv[2];
    rclcpp::init(argc, argv);
    try {
      rclcpp::spin(std::make_shared<D435iRknnStream>(model_path));
    } catch (const std::exception &error) {
      std::cerr << "Error: " << error.what() << '\n';
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::shutdown();
    return 0;
  }

  if (argc >= 2 && std::string(argv[1]) == "--camera") {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FirstColorFrameProbe>());
    return 0;
  }

  if (argc != 2) {
    std::cerr << "Usage:\n"
              << "  " << argv[0] << " <model.rknn>\n"
              << "  " << argv[0] << " --infer <model.rknn> [--ros-args -p color_topic:=<topic>]\n"
              << "  " << argv[0] << " --camera [--ros-args -p color_topic:=<topic>]\n";
    return 2;
  }

  try {
    const std::string model_path = argv[1];
    std::vector<std::uint8_t> model = readBinaryFile(model_path);

    RknnContext context;
    checkRknn(
      rknn_init(
        context.address(), model.data(), static_cast<std::uint32_t>(model.size()), 0, nullptr),
      "rknn_init");

    rknn_sdk_version sdk_version{};
    checkRknn(
      rknn_query(
        context.get(), RKNN_QUERY_SDK_VERSION, &sdk_version, sizeof(sdk_version)),
      "RKNN_QUERY_SDK_VERSION");

    rknn_input_output_num io_count{};
    checkRknn(
      rknn_query(context.get(), RKNN_QUERY_IN_OUT_NUM, &io_count, sizeof(io_count)),
      "RKNN_QUERY_IN_OUT_NUM");

    std::cout << "Model        : " << model_path << '\n'
              << "Model bytes  : " << model.size() << '\n'
              << "RKNN API     : " << sdk_version.api_version << '\n'
              << "RKNN driver  : " << sdk_version.drv_version << '\n'
              << "Input count  : " << io_count.n_input << '\n'
              << "Output count : " << io_count.n_output << "\n\n";

    for (std::uint32_t i = 0; i < io_count.n_input; ++i) {
      rknn_tensor_attr attr{};
      attr.index = i;
      checkRknn(
        rknn_query(context.get(), RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr)),
        "RKNN_QUERY_INPUT_ATTR[" + std::to_string(i) + "]");
      printTensor("input", attr);
    }

    for (std::uint32_t i = 0; i < io_count.n_output; ++i) {
      rknn_tensor_attr attr{};
      attr.index = i;
      checkRknn(
        rknn_query(context.get(), RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)),
        "RKNN_QUERY_OUTPUT_ATTR[" + std::to_string(i) + "]");
      printTensor("output", attr);
    }
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}

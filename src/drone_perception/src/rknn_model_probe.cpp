#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/msg/image.hpp>

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
    detector_(model_path, RKNN_NPU_CORE_0),
    started_at_(Clock::now())
  {
    const rknn_mem_size memory = detector_.memorySize();
    weight_mib_ = static_cast<double>(memory.total_weight_size) / (1024.0 * 1024.0);
    internal_mib_ = static_cast<double>(memory.total_internal_size) / (1024.0 * 1024.0);
    dma_mib_ = static_cast<double>(memory.total_dma_allocated_size) / (1024.0 * 1024.0);
    const std::string rgbd_topic = declare_parameter<std::string>(
      "rgbd_topic", "/camera/camera/rgbd");
    rgbd_sub_ = create_subscription<realsense2_camera_msgs::msg::RGBD>(
      rgbd_topic, rclcpp::SensorDataQoS(),
      [this](const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr message) {
        receiveFrame(message);
      });
    worker_ = std::thread(&D435iRknnStream::workerLoop, this);
    RCLCPP_INFO(get_logger(),
      "Streaming RGBD RKNN inference started: rgbd=%s model=%s",
      rgbd_topic.c_str(), model_path.c_str());
  }

  ~D435iRknnStream() override
  {
    running_.store(false);
    frame_ready_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  using Clock = std::chrono::steady_clock;

  void receiveFrame(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr message)
  {
    const std::int64_t stamp_ns = rclcpp::Time(message->rgb.header.stamp).nanoseconds();
    if (last_received_stamp_ns_ > 0 && stamp_ns > last_received_stamp_ns_) {
      const double fps = 1.0e9 / static_cast<double>(stamp_ns - last_received_stamp_ns_);
      const double previous_fps = input_fps_.load();
      input_fps_.store(previous_fps <= 0.0 ? fps : 0.9 * previous_fps + 0.1 * fps);
    }
    last_received_stamp_ns_ = stamp_ns;
    received_count_.fetch_add(1);

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_) {
        dropped_count_.fetch_add(1);
      }
      latest_frame_ = message;
    }
    frame_ready_.notify_one();
  }

  void workerLoop()
  {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    while (running_.load() && rclcpp::ok()) {
      realsense2_camera_msgs::msg::RGBD::ConstSharedPtr message;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_ready_.wait(lock, [this] {
          return latest_frame_ != nullptr || !running_.load() || !rclcpp::ok();
        });
        if (!running_.load() || !rclcpp::ok()) {
          break;
        }
        message = std::move(latest_frame_);
      }
      inferFrame(message);
    }
    cv::destroyWindow(window_name_);
  }

  void inferFrame(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &message)
  {
    try {
      const auto image = cv_bridge::toCvShare(message->rgb, message, "bgr8");
      const auto depth = cv_bridge::toCvShare(message->depth, message);
      if (image->image.size() != depth->image.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "RGBD size mismatch: color=%dx%d depth=%dx%d", image->image.cols, image->image.rows,
          depth->image.cols, depth->image.rows);
        return;
      }
      const std::vector<Detection> detections = detector_.infer(image->image);
      ++frame_count_;

      const auto now = Clock::now();
      if (last_frame_at_ != Clock::time_point{}) {
        const double interval = std::chrono::duration<double>(now - last_frame_at_).count();
        if (interval > 0.0) {
          const double current_fps = 1.0 / interval;
          display_fps_ = display_fps_ <= 0.0 ? current_fps : 0.9 * display_fps_ + 0.1 * current_fps;
        }
      }
      last_frame_at_ = now;

      cv::Mat display = image->image.clone();

      for (Detection detection : detections) {
        const DepthSampleResult sample = depth_processor_.sampleAt(
          depth->image, detection.center.x, detection.center.y, sample_radius_px_);
        detection.has_depth = sample.has_valid_depth;
        detection.depth_m = sample.depth_m;
        if (sample.has_valid_depth) {
          detection.point_3d = depth_processor_.projectTo3D(
            detection.center.x, detection.center.y, sample.depth_m, message->rgb_camera_info);
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
        RCLCPP_INFO(get_logger(),
          "detection class=%d score=%.3f box=(%d,%d,%d,%d) center=(%d,%d) depth=%.3fm",
          detection.class_id, detection.score, detection.box.x, detection.box.y,
          detection.box.width, detection.box.height, detection.center.x, detection.center.y,
          sample.has_valid_depth ? sample.depth_m : -1.0F);
      }

      const DepthSampleResult center_depth = depth_processor_.sampleAt(
        depth->image, display.cols / 2, display.rows / 2, sample_radius_px_);
      const auto &timing = detector_.lastTiming();
      const double api_run_ms = detector_.lastRknnRunMs();
      const double npu_fps = api_run_ms > 0.0 ? 1000.0 / api_run_ms : 0.0;
      const cv::Scalar text_color(255, 0, 255);
      const std::string status = cv::format(
        "Input %.1f  Process %.1f  NPU %.1f FPS  CORE_0  Drop %llu",
        input_fps_.load(), display_fps_, npu_fps,
        static_cast<unsigned long long>(dropped_count_.load()));
      cv::putText(display, status, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX,
        0.54, text_color, 1, cv::LINE_AA);
      const std::string stream_status = cv::format("RGB %dx%d  Depth %dx%d %s",
        display.cols, display.rows, depth->image.cols, depth->image.rows,
        message->depth.encoding.c_str());
      cv::putText(display, stream_status, cv::Point(12, 56), cv::FONT_HERSHEY_SIMPLEX,
        0.54, text_color, 1, cv::LINE_AA);
      const std::string depth_status = center_depth.has_valid_depth
        ? cv::format("Center depth %.3fm  RGBD aligned + intrinsics", center_depth.depth_m)
        : std::string("Center depth n/a  RGBD aligned + intrinsics");
      cv::putText(display, depth_status, cv::Point(12, 82), cv::FONT_HERSHEY_SIMPLEX,
        0.54, text_color, 1, cv::LINE_AA);
      const std::string memory_status = cv::format(
        "RKNN memory: weight %.1f MiB  internal %.1f MiB  DMA %.1f MiB",
        weight_mib_, internal_mib_, dma_mib_);
      cv::putText(display, memory_status, cv::Point(12, 108), cv::FONT_HERSHEY_SIMPLEX,
        0.54, text_color, 1, cv::LINE_AA);
      cv::imshow(window_name_, display);
      const int key = cv::waitKey(1) & 0xff;
      if (key == 'q' || key == 27) {
        rclcpp::shutdown();
        return;
      }

      const double report_seconds = std::chrono::duration<double>(now - last_report_at_).count();
      if (report_seconds >= 1.0) {
        const double total_seconds = std::chrono::duration<double>(now - started_at_).count();
        RCLCPP_INFO(get_logger(),
          "stream frames=%llu received=%llu dropped=%llu input_fps=%.2f process_fps=%.2f "
          "npu_fps=%.2f api_run=%.2fms core=0 "
          "detections=%zu preprocess=%.2fms input=%.2fms rknn_run=%.2fms "
          "output=%.2fms postprocess=%.2fms total=%.2fms",
          static_cast<unsigned long long>(frame_count_),
          static_cast<unsigned long long>(received_count_.load()),
          static_cast<unsigned long long>(dropped_count_.load()), input_fps_.load(),
          frame_count_ / total_seconds, npu_fps, api_run_ms,
          detections.size(), timing.preprocess_ms, timing.input_set_ms, timing.rknn_run_ms,
          timing.output_get_ms, timing.postprocess_ms, timing.detector_total_ms);
        last_report_at_ = now;
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "stream inference failed: %s", error.what());
    }
  }

  RknnYoloDetector detector_;
  DepthProcessor depth_processor_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
  std::mutex frame_mutex_;
  std::condition_variable frame_ready_;
  realsense2_camera_msgs::msg::RGBD::ConstSharedPtr latest_frame_;
  std::thread worker_;
  std::atomic<bool> running_{true};
  std::atomic<std::uint64_t> received_count_{0};
  std::atomic<std::uint64_t> dropped_count_{0};
  int sample_radius_px_ = 10;
  Clock::time_point started_at_;
  Clock::time_point last_report_at_ = started_at_;
  Clock::time_point last_frame_at_{};
  std::uint64_t frame_count_ = 0;
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

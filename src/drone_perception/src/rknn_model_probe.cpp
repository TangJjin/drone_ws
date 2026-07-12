#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rknn_api.h"
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
  : Node("rknn_d435i_stream"), detector_(model_path), started_at_(Clock::now())
  {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    const std::string topic = declare_parameter<std::string>(
      "color_topic", "/camera/camera/color/image_raw");
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) { inferFrame(message); });
    RCLCPP_INFO(get_logger(), "Streaming RKNN inference started: topic=%s model=%s",
      topic.c_str(), model_path.c_str());
  }

  ~D435iRknnStream() override
  {
    cv::destroyWindow(window_name_);
  }

private:
  using Clock = std::chrono::steady_clock;

  void inferFrame(const sensor_msgs::msg::Image::ConstSharedPtr &message)
  {
    try {
      const auto image = cv_bridge::toCvShare(message, "bgr8");
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

      for (const Detection &detection : detections) {
        cv::rectangle(display, detection.box, cv::Scalar(0, 255, 0), 2);
        const std::string label = cv::format(
          "class %d %.2f", detection.class_id, detection.score);
        const int label_y = std::max(20, detection.box.y - 6);
        cv::putText(display, label, cv::Point(detection.box.x, label_y),
          cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        RCLCPP_INFO(get_logger(),
          "detection class=%d score=%.3f box=(%d,%d,%d,%d) center=(%d,%d)",
          detection.class_id, detection.score, detection.box.x, detection.box.y,
          detection.box.width, detection.box.height, detection.center.x, detection.center.y);
      }

      const std::string status = cv::format(
        "FPS %.1f  %dx%d", display_fps_, display.cols, display.rows);
      cv::putText(display, status, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX,
        0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
      cv::imshow(window_name_, display);
      const int key = cv::waitKey(1) & 0xff;
      if (key == 'q' || key == 27) {
        rclcpp::shutdown();
        return;
      }

      const double report_seconds = std::chrono::duration<double>(now - last_report_at_).count();
      if (report_seconds >= 1.0) {
        const double total_seconds = std::chrono::duration<double>(now - started_at_).count();
        const auto &timing = detector_.lastTiming();
        RCLCPP_INFO(get_logger(),
          "stream frames=%llu fps=%.2f detections=%zu preprocess=%.2fms input=%.2fms "
          "rknn_run=%.2fms output=%.2fms postprocess=%.2fms total=%.2fms",
          static_cast<unsigned long long>(frame_count_), frame_count_ / total_seconds,
          detections.size(), timing.preprocess_ms, timing.input_set_ms, timing.rknn_run_ms,
          timing.output_get_ms, timing.postprocess_ms, timing.detector_total_ms);
        last_report_at_ = now;
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "stream inference failed: %s", error.what());
    }
  }

  RknnYoloDetector detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  Clock::time_point started_at_;
  Clock::time_point last_report_at_ = started_at_;
  Clock::time_point last_frame_at_{};
  std::uint64_t frame_count_ = 0;
  double display_fps_ = 0.0;
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

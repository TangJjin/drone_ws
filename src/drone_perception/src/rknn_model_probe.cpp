#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rknn_api.h"

namespace
{

std::vector<std::uint8_t> readBinaryFile(const std::string &path)
{
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
        message->step, image->image.step, image->image.channels());
      rclcpp::shutdown();
    } catch (const cv_bridge::Exception &error) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion to bgr8 failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char **argv)
{
  if (argc >= 2 && std::string(argv[1]) == "--camera") {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FirstColorFrameProbe>());
    return 0;
  }

  if (argc != 2) {
    std::cerr << "Usage:\n"
              << "  " << argv[0] << " <model.rknn>\n"
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

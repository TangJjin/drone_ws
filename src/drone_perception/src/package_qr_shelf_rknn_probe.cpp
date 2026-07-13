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
#include "drone_perception/detection.hpp"

#include <limits>
#include <map>
#include <sstream>
#include <utility>

#include <opencv2/dnn.hpp>

namespace
{
using SteadyClock = std::chrono::steady_clock;

double elapsedMs(
  const SteadyClock::time_point &start,
  const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 路径 B：9 输出 YOLO11（检测器 + DFL 后处理合在本文件）
// 类别 0=qrcode, 1=package, 2=shelf_tag；置信度阈值 0.5
class RknnYolo11PathbDetector
{
public:
  static constexpr int kClassCount = 3;
  static constexpr float kConfThresh = 0.5F;
  static constexpr float kNmsThresh = 0.45F;
  static constexpr int kDflLen = 16;
  static constexpr int kBoxChannels = 4 * kDflLen;

  enum class Layout { NCHW = 0, NHWC = 1 };

  struct InferenceTimingStats
  {
    double preprocess_ms = 0.0;
    double input_prepare_ms = 0.0;
    double rknn_run_ms = 0.0;
    double output_get_ms = 0.0;
    double postprocess_ms = 0.0;
    double detector_total_ms = 0.0;
  };

  explicit RknnYolo11PathbDetector(
    const std::string &model_path,
    rknn_core_mask core_mask = RKNN_NPU_CORE_0_1_2)
  {
    loadModel(model_path, core_mask);
  }

  ~RknnYolo11PathbDetector()
  {
    // 与 rknn_yolo_detector / rknn_model_probe 一致：先释放 zero-copy 输入再 destroy context
    if (input_mem_ != nullptr && context_ != 0) {
      rknn_destroy_mem(context_, input_mem_);
      input_mem_ = nullptr;
    }
    if (context_ != 0) {
      rknn_destroy(context_);
      context_ = 0;
    }
  }

  RknnYolo11PathbDetector(const RknnYolo11PathbDetector &) = delete;
  RknnYolo11PathbDetector &operator=(const RknnYolo11PathbDetector &) = delete;

  static const char *className(int class_id)
  {
    static constexpr const char *kNames[kClassCount] = {
      "qrcode", "package", "shelf_tag"};
    if (class_id < 0 || class_id >= kClassCount) {
      return "unknown";
    }
    return kNames[class_id];
  }

  const InferenceTimingStats &lastTiming() const { return last_timing_; }

  rknn_mem_size memorySize() const
  {
    rknn_mem_size memory_size;
    std::memset(&memory_size, 0, sizeof(memory_size));
    const int ret = rknn_query(
      context_, RKNN_QUERY_MEM_SIZE, &memory_size, sizeof(memory_size));
    if (ret != RKNN_SUCC) {
      throw std::runtime_error(
        "RKNN_QUERY_MEM_SIZE failed, ret=" + std::to_string(ret));
    }
    return memory_size;
  }

  double lastRknnRunMs() const
  {
    rknn_perf_run perf_run;
    std::memset(&perf_run, 0, sizeof(perf_run));
    const int ret = rknn_query(
      context_, RKNN_QUERY_PERF_RUN, &perf_run, sizeof(perf_run));
    if (ret != RKNN_SUCC) {
      throw std::runtime_error(
        "RKNN_QUERY_PERF_RUN failed, ret=" + std::to_string(ret));
    }
    return static_cast<double>(perf_run.run_duration) / 1000.0;
  }

  std::vector<Detection> infer(const cv::Mat &rgb_image)
  {
    InferenceTimingStats timing;
    const auto detector_t0 = SteadyClock::now();

    const auto preprocess_t0 = SteadyClock::now();
    const LetterboxResult letterbox = makeLetterbox(rgb_image);
    const auto preprocess_t1 = SteadyClock::now();
    timing.preprocess_ms = elapsedMs(preprocess_t0, preprocess_t1);

    // 输入准备：优先 native FP16 zero-copy（与 rknn_model_probe / RknnYoloDetector 相同）
    const auto input_prepare_t0 = SteadyClock::now();
    int ret = RKNN_SUCC;
    if (zero_copy_input_) {
      letterbox.image.convertTo(input_fp16_view_, CV_16FC3, 1.0 / 255.0);
      ret = rknn_mem_sync(context_, input_mem_, RKNN_MEMORY_SYNC_TO_DEVICE);
      if (ret != RKNN_SUCC) {
        throw std::runtime_error(
          "rknn_mem_sync input failed, ret=" + std::to_string(ret));
      }
    } else {
      rknn_input input;
      std::memset(&input, 0, sizeof(input));
      input.index = 0;
      input.type = RKNN_TENSOR_UINT8;
      input.size = static_cast<unsigned int>(input_width_ * input_height_ * 3);
      input.fmt = RKNN_TENSOR_NHWC;
      input.buf = letterbox.image.data;
      ret = rknn_inputs_set(context_, 1, &input);
      if (ret != RKNN_SUCC) {
        throw std::runtime_error(
          "rknn_inputs_set failed, ret=" + std::to_string(ret));
      }
    }
    const auto input_prepare_t1 = SteadyClock::now();
    timing.input_prepare_ms = elapsedMs(input_prepare_t0, input_prepare_t1);

    const auto rknn_run_t0 = SteadyClock::now();
    ret = rknn_run(context_, nullptr);
    const auto rknn_run_t1 = SteadyClock::now();
    timing.rknn_run_ms = elapsedMs(rknn_run_t0, rknn_run_t1);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("rknn_run failed, ret=" + std::to_string(ret));
    }

    std::vector<rknn_output> outputs(output_count_);
    for (uint32_t i = 0; i < output_count_; ++i) {
      std::memset(&outputs[i], 0, sizeof(outputs[i]));
      outputs[i].index = i;
      outputs[i].want_float = 1;
    }

    const auto output_get_t0 = SteadyClock::now();
    ret = rknn_outputs_get(context_, output_count_, outputs.data(), nullptr);
    const auto output_get_t1 = SteadyClock::now();
    timing.output_get_ms = elapsedMs(output_get_t0, output_get_t1);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error(
        "rknn_outputs_get failed, ret=" + std::to_string(ret));
    }

    struct OutputGuard
    {
      rknn_context ctx = 0;
      uint32_t n = 0;
      rknn_output *outs = nullptr;
      ~OutputGuard()
      {
        if (outs != nullptr && n > 0) {
          rknn_outputs_release(ctx, n, outs);
        }
      }
    } guard{context_, output_count_, outputs.data()};

    const auto postprocess_t0 = SteadyClock::now();
    std::vector<ScaleBranch> branches;
    bindScaleBranches(outputs, branches);
    std::vector<Detection> detections = parseBranches(
      branches, rgb_image.size(), letterbox.scale, letterbox.pad_x, letterbox.pad_y);
    const auto postprocess_t1 = SteadyClock::now();
    timing.postprocess_ms = elapsedMs(postprocess_t0, postprocess_t1);
    timing.detector_total_ms = elapsedMs(detector_t0, postprocess_t1);
    last_timing_ = timing;
    return detections;
  }

private:
  struct LetterboxResult
  {
    cv::Mat image;
    float scale = 1.0F;
    int pad_x = 0;
    int pad_y = 0;
  };

  struct OutputSlot
  {
    uint32_t index = 0;
    int channels = 0;
    int height = 0;
    int width = 0;
    Layout layout = Layout::NCHW;
  };

  struct BranchTensor
  {
    const float *data = nullptr;
    int channels = 0;
    int height = 0;
    int width = 0;
    Layout layout = Layout::NCHW;
  };

  struct ScaleBranch
  {
    BranchTensor box;
    BranchTensor cls;
  };

  static std::vector<unsigned char> readFile(const std::string &path)
  {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("failed to open model file: " + path);
    }
    const std::streamsize file_size = file.tellg();
    if (file_size <= 0) {
      throw std::runtime_error("model file is empty: " + path);
    }
    std::vector<unsigned char> data(static_cast<std::size_t>(file_size));
    file.seekg(0, std::ios::beg);
    if (!file.read(reinterpret_cast<char *>(data.data()), file_size)) {
      throw std::runtime_error("failed to read model file: " + path);
    }
    return data;
  }

  static void printTensorAttr(const char *prefix, const rknn_tensor_attr &attr)
  {
    std::cout << "[RKNN-PATHB] " << prefix << "[" << attr.index << "]"
              << " name=" << attr.name << " n_dims=" << attr.n_dims << " dims=(";
    for (uint32_t i = 0; i < attr.n_dims; ++i) {
      std::cout << attr.dims[i] << (i + 1 < attr.n_dims ? "," : "");
    }
    std::cout << ") type=" << attr.type << " fmt=" << attr.fmt
              << " n_elems=" << attr.n_elems << std::endl;
  }

  static bool parseSpatialChannels(
    const rknn_tensor_attr &attr, int &channels, int &height, int &width, Layout &layout)
  {
    if (attr.n_dims != 4) {
      return false;
    }

    // 优先信任 Runtime 的 fmt。
    // 错误启发式会把 NCHW (1,64,40,40) 误判成 NHWC(H=64,W=40,C=40)。
    if (attr.fmt == RKNN_TENSOR_NHWC) {
      layout = Layout::NHWC;
      height = static_cast<int>(attr.dims[1]);
      width = static_cast<int>(attr.dims[2]);
      channels = static_cast<int>(attr.dims[3]);
      return height > 0 && width > 0 && channels > 0;
    }

    if (attr.fmt == RKNN_TENSOR_NCHW) {
      layout = Layout::NCHW;
      channels = static_cast<int>(attr.dims[1]);
      height = static_cast<int>(attr.dims[2]);
      width = static_cast<int>(attr.dims[3]);
      return channels > 0 && height > 0 && width > 0;
    }

    // fmt 未知时：通道维通常是 1/3/64，用这个区分 NCHW/NHWC
    const int d1 = static_cast<int>(attr.dims[1]);
    const int d2 = static_cast<int>(attr.dims[2]);
    const int d3 = static_cast<int>(attr.dims[3]);
    const auto is_channel = [](int v) {
      return v == 1 || v == 3 || v == 64;
    };
    if (is_channel(d1) && d2 >= 8 && d3 >= 8) {
      layout = Layout::NCHW;
      channels = d1;
      height = d2;
      width = d3;
      return true;
    }
    if (is_channel(d3) && d1 >= 8 && d2 >= 8) {
      layout = Layout::NHWC;
      height = d1;
      width = d2;
      channels = d3;
      return true;
    }

    // 最后回退 NCHW
    layout = Layout::NCHW;
    channels = d1;
    height = d2;
    width = d3;
    return channels > 0 && height > 0 && width > 0;
  }

  static float readValue(const BranchTensor &tensor, int channel, int y, int x)
  {
    if (tensor.data == nullptr || channel < 0 || channel >= tensor.channels ||
      y < 0 || y >= tensor.height || x < 0 || x >= tensor.width)
    {
      return 0.0F;
    }
    if (tensor.layout == Layout::NHWC) {
      const std::size_t index =
        (static_cast<std::size_t>(y) * static_cast<std::size_t>(tensor.width) +
          static_cast<std::size_t>(x)) *
          static_cast<std::size_t>(tensor.channels) +
        static_cast<std::size_t>(channel);
      return tensor.data[index];
    }
    const std::size_t plane =
      static_cast<std::size_t>(tensor.height) * static_cast<std::size_t>(tensor.width);
    const std::size_t index =
      static_cast<std::size_t>(channel) * plane +
      static_cast<std::size_t>(y) * static_cast<std::size_t>(tensor.width) +
      static_cast<std::size_t>(x);
    return tensor.data[index];
  }

  static void dflDecodeBox(
    const BranchTensor &box, int y, int x, float stride_x, float stride_y, float *xyxy)
  {
    float distances[4] = {};
    for (int side = 0; side < 4; ++side) {
      float max_logit = -std::numeric_limits<float>::infinity();
      float logits[kDflLen];
      for (int i = 0; i < kDflLen; ++i) {
        logits[i] = readValue(box, side * kDflLen + i, y, x);
        max_logit = std::max(max_logit, logits[i]);
      }
      float sum_exp = 0.0F;
      float expected = 0.0F;
      for (int i = 0; i < kDflLen; ++i) {
        const float p = std::exp(logits[i] - max_logit);
        sum_exp += p;
        expected += p * static_cast<float>(i);
      }
      distances[side] = expected / std::max(sum_exp, 1e-12F);
    }
    const float gx = static_cast<float>(x);
    const float gy = static_cast<float>(y);
    xyxy[0] = (gx + 0.5F - distances[0]) * stride_x;
    xyxy[1] = (gy + 0.5F - distances[1]) * stride_y;
    xyxy[2] = (gx + 0.5F + distances[2]) * stride_x;
    xyxy[3] = (gy + 0.5F + distances[3]) * stride_y;
  }

  std::vector<Detection> parseBranches(
    const std::vector<ScaleBranch> &branches,
    const cv::Size &original_size,
    float scale,
    int pad_x,
    int pad_y) const
  {
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    if (branches.empty() || scale <= 0.0F ||
      original_size.width <= 0 || original_size.height <= 0)
    {
      return {};
    }

    for (const ScaleBranch &branch : branches) {
      if (branch.box.data == nullptr || branch.cls.data == nullptr ||
        branch.box.channels != kBoxChannels ||
        branch.cls.channels != kClassCount ||
        branch.box.height <= 0 || branch.box.width <= 0 ||
        branch.cls.height != branch.box.height ||
        branch.cls.width != branch.box.width)
      {
        continue;
      }
      const float stride_x =
        static_cast<float>(input_width_) / static_cast<float>(branch.box.width);
      const float stride_y =
        static_cast<float>(input_height_) / static_cast<float>(branch.box.height);
      for (int y = 0; y < branch.box.height; ++y) {
        for (int x = 0; x < branch.box.width; ++x) {
          int best_class = -1;
          float best_score = 0.0F;
          for (int class_id = 0; class_id < kClassCount; ++class_id) {
            const float score = readValue(branch.cls, class_id, y, x);
            if (score > best_score) {
              best_score = score;
              best_class = class_id;
            }
          }
          if (best_class < 0 || best_score < kConfThresh) {
            continue;
          }
          float xyxy[4] = {};
          dflDecodeBox(branch.box, y, x, stride_x, stride_y, xyxy);
          float x1 = (xyxy[0] - static_cast<float>(pad_x)) / scale;
          float y1 = (xyxy[1] - static_cast<float>(pad_y)) / scale;
          float x2 = (xyxy[2] - static_cast<float>(pad_x)) / scale;
          float y2 = (xyxy[3] - static_cast<float>(pad_y)) / scale;
          x1 = std::clamp(x1, 0.0F, static_cast<float>(original_size.width - 1));
          y1 = std::clamp(y1, 0.0F, static_cast<float>(original_size.height - 1));
          x2 = std::clamp(x2, 0.0F, static_cast<float>(original_size.width - 1));
          y2 = std::clamp(y2, 0.0F, static_cast<float>(original_size.height - 1));
          const int left = static_cast<int>(std::round(x1));
          const int top = static_cast<int>(std::round(y1));
          const int right = static_cast<int>(std::round(x2));
          const int bottom = static_cast<int>(std::round(y2));
          const int width = right - left;
          const int height = bottom - top;
          if (width <= 1 || height <= 1) {
            continue;
          }
          boxes.emplace_back(left, top, width, height);
          scores.push_back(best_score);
          class_ids.push_back(best_class);
        }
      }
    }

    if (boxes.empty()) {
      return {};
    }

    std::vector<Detection> detections;
    std::vector<int> unique_classes = class_ids;
    std::sort(unique_classes.begin(), unique_classes.end());
    unique_classes.erase(
      std::unique(unique_classes.begin(), unique_classes.end()), unique_classes.end());
    for (int class_id : unique_classes) {
      std::vector<cv::Rect> class_boxes;
      std::vector<float> class_scores;
      std::vector<int> local_indices;
      for (std::size_t i = 0; i < class_ids.size(); ++i) {
        if (class_ids[i] != class_id) {
          continue;
        }
        class_boxes.push_back(boxes[i]);
        class_scores.push_back(scores[i]);
        local_indices.push_back(static_cast<int>(i));
      }
      std::vector<int> keep;
      cv::dnn::NMSBoxes(class_boxes, class_scores, kConfThresh, kNmsThresh, keep);
      for (int keep_index : keep) {
        const int source = local_indices[static_cast<std::size_t>(keep_index)];
        Detection detection;
        detection.class_id = class_ids[static_cast<std::size_t>(source)];
        detection.score = scores[static_cast<std::size_t>(source)];
        detection.box = boxes[static_cast<std::size_t>(source)];
        detection.center = cv::Point(
          detection.box.x + detection.box.width / 2,
          detection.box.y + detection.box.height / 2);
        detections.push_back(detection);
      }
    }
    return detections;
  }

  LetterboxResult makeLetterbox(const cv::Mat &rgb_image)
  {
    if (rgb_image.empty()) {
      throw std::runtime_error("input image is empty");
    }
    const float scale = std::min(
      static_cast<float>(input_width_) / static_cast<float>(rgb_image.cols),
      static_cast<float>(input_height_) / static_cast<float>(rgb_image.rows));
    const int resized_width =
      static_cast<int>(static_cast<float>(rgb_image.cols) * scale);
    const int resized_height =
      static_cast<int>(static_cast<float>(rgb_image.rows) * scale);
    const int pad_x = (input_width_ - resized_width) / 2;
    const int pad_y = (input_height_ - resized_height) / 2;
    letterbox_buffer_.create(input_height_, input_width_, rgb_image.type());
    letterbox_buffer_.setTo(cv::Scalar(0, 0, 0));
    cv::Mat destination = letterbox_buffer_(
      cv::Rect(pad_x, pad_y, resized_width, resized_height));
    if (resized_width == rgb_image.cols && resized_height == rgb_image.rows) {
      rgb_image.copyTo(destination);
    } else {
      resized_buffer_.create(resized_height, resized_width, rgb_image.type());
      cv::resize(rgb_image, resized_buffer_, cv::Size(resized_width, resized_height));
      resized_buffer_.copyTo(destination);
    }
    return LetterboxResult{letterbox_buffer_, scale, pad_x, pad_y};
  }

  void loadModel(const std::string &model_path, rknn_core_mask core_mask)
  {
    model_data_ = readFile(model_path);
    const int ret = rknn_init(
      &context_, model_data_.data(),
      static_cast<unsigned int>(model_data_.size()), 0, nullptr);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("rknn_init failed, ret=" + std::to_string(ret));
    }
    const int core_ret = rknn_set_core_mask(context_, core_mask);
    if (core_ret != RKNN_SUCC) {
      throw std::runtime_error(
        "rknn_set_core_mask failed, ret=" + std::to_string(core_ret));
    }
    std::cout << "[RKNN-PATHB] NPU core mask: " << static_cast<int>(core_mask) << std::endl;

    rknn_input_output_num io_num;
    std::memset(&io_num, 0, sizeof(io_num));
    int query_ret = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (query_ret != RKNN_SUCC) {
      throw std::runtime_error(
        "RKNN_QUERY_IN_OUT_NUM failed, ret=" + std::to_string(query_ret));
    }
    std::cout << "[RKNN-PATHB] input num: " << io_num.n_input
              << ", output num: " << io_num.n_output << std::endl;
    if (io_num.n_input != 1 || io_num.n_output != 9) {
      throw std::runtime_error(
        "unsupported path-B IO counts, expected input=1 output=9");
    }
    output_count_ = io_num.n_output;
    output_slots_.clear();

    for (uint32_t i = 0; i < io_num.n_input; ++i) {
      rknn_tensor_attr attr;
      std::memset(&attr, 0, sizeof(attr));
      attr.index = i;
      query_ret = rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
      if (query_ret != RKNN_SUCC) {
        throw std::runtime_error("RKNN_QUERY_INPUT_ATTR failed");
      }
      printTensorAttr("input", attr);
      if (attr.n_dims == 4) {
        if (attr.fmt == RKNN_TENSOR_NHWC) {
          input_height_ = static_cast<int>(attr.dims[1]);
          input_width_ = static_cast<int>(attr.dims[2]);
        } else {
          input_height_ = static_cast<int>(attr.dims[2]);
          input_width_ = static_cast<int>(attr.dims[3]);
        }
      }
    }

    int box_count = 0, cls_count = 0, sum_count = 0;
    for (uint32_t i = 0; i < io_num.n_output; ++i) {
      rknn_tensor_attr attr;
      std::memset(&attr, 0, sizeof(attr));
      attr.index = i;
      query_ret = rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
      if (query_ret != RKNN_SUCC) {
        throw std::runtime_error("RKNN_QUERY_OUTPUT_ATTR failed");
      }
      printTensorAttr("output", attr);
      OutputSlot slot;
      slot.index = i;
      if (!parseSpatialChannels(attr, slot.channels, slot.height, slot.width, slot.layout)) {
        throw std::runtime_error("failed to parse path-B output dims");
      }
      if (attr.fmt == RKNN_TENSOR_NHWC) {
        slot.layout = Layout::NHWC;
      }
      if (slot.channels == 64) {
        ++box_count;
      } else if (slot.channels == kClassCount) {
        ++cls_count;
      } else if (slot.channels == 1) {
        ++sum_count;
      } else {
        std::ostringstream message;
        message << "unexpected path-B output channels=" << slot.channels
                << " at index=" << i
                << " parsed as " << slot.height << "x" << slot.width
                << " layout=" << static_cast<int>(slot.layout)
                << " fmt=" << static_cast<int>(attr.fmt);
        throw std::runtime_error(message.str());
      }
      output_slots_.push_back(slot);
    }
    if (box_count != 3 || cls_count != 3 || sum_count != 3) {
      throw std::runtime_error("path-B output role count mismatch");
    }

    // 与 rknn_model_probe 使用的 RknnYoloDetector 相同：尝试 native FP16 zero-copy
    configureZeroCopyInput();

    std::cout << "[RKNN-PATHB] model ready: " << input_width_ << "x" << input_height_
              << ", classes=3 (qrcode/package/shelf_tag), conf=" << kConfThresh
              << " nms=" << kNmsThresh
              << ", zero_copy=" << (zero_copy_input_ ? "on" : "off") << std::endl;
  }

  // 对齐 rknn_yolo_detector.cpp::configureZeroCopyInput
  void configureZeroCopyInput()
  {
    std::memset(&native_input_attr_, 0, sizeof(native_input_attr_));
    native_input_attr_.index = 0;
    const int query_ret = rknn_query(
      context_,
      RKNN_QUERY_NATIVE_INPUT_ATTR,
      &native_input_attr_,
      sizeof(native_input_attr_));
    if (query_ret != RKNN_SUCC ||
      native_input_attr_.n_dims != 4 ||
      native_input_attr_.fmt != RKNN_TENSOR_NHWC ||
      native_input_attr_.type != RKNN_TENSOR_FLOAT16 ||
      native_input_attr_.dims[3] != 3)
    {
      std::cout << "[RKNN-PATHB] native FP16 zero-copy input unavailable; "
                << "using rknn_inputs_set fallback" << std::endl;
      return;
    }

    native_input_attr_.pass_through = 1;
    input_mem_ = rknn_create_mem(context_, native_input_attr_.size_with_stride);
    if (input_mem_ == nullptr) {
      throw std::runtime_error("rknn_create_mem for zero-copy input failed");
    }
    const int set_ret = rknn_set_io_mem(context_, input_mem_, &native_input_attr_);
    if (set_ret != RKNN_SUCC) {
      rknn_destroy_mem(context_, input_mem_);
      input_mem_ = nullptr;
      throw std::runtime_error(
        "rknn_set_io_mem for zero-copy input failed, ret=" + std::to_string(set_ret));
    }

    const int height = static_cast<int>(native_input_attr_.dims[1]);
    const int width = static_cast<int>(native_input_attr_.dims[2]);
    const int width_stride = native_input_attr_.w_stride == 0 ?
      width : static_cast<int>(native_input_attr_.w_stride);
    input_fp16_view_ = cv::Mat(
      height,
      width,
      CV_16FC3,
      input_mem_->virt_addr,
      static_cast<std::size_t>(width_stride) * 3U * sizeof(std::uint16_t));
    zero_copy_input_ = true;
    std::cout << "[RKNN-PATHB] native FP16 zero-copy input enabled: "
              << width << "x" << height
              << " stride=" << width_stride
              << " bytes=" << native_input_attr_.size_with_stride << std::endl;
  }

  void bindScaleBranches(
    const std::vector<rknn_output> &outputs,
    std::vector<ScaleBranch> &branches) const
  {
    struct ScaleGroup
    {
      const OutputSlot *box = nullptr;
      const OutputSlot *cls = nullptr;
    };
    std::map<int, ScaleGroup> groups;
    for (const OutputSlot &slot : output_slots_) {
      ScaleGroup &group = groups[slot.height * 10000 + slot.width];
      if (slot.channels == 64) {
        group.box = &slot;
      } else if (slot.channels == kClassCount) {
        group.cls = &slot;
      }
    }
    std::vector<std::pair<int, ScaleGroup>> ordered(groups.begin(), groups.end());
    std::sort(ordered.begin(), ordered.end(),
      [](const auto &a, const auto &b) { return a.first > b.first; });
    branches.clear();
    for (const auto &item : ordered) {
      if (item.second.box == nullptr || item.second.cls == nullptr) {
        throw std::runtime_error("incomplete path-B scale group");
      }
      ScaleBranch branch;
      const OutputSlot *box = item.second.box;
      const OutputSlot *cls = item.second.cls;
      branch.box = {static_cast<const float *>(outputs[box->index].buf),
        box->channels, box->height, box->width, box->layout};
      branch.cls = {static_cast<const float *>(outputs[cls->index].buf),
        cls->channels, cls->height, cls->width, cls->layout};
      branches.push_back(branch);
    }
    if (branches.size() != 3) {
      throw std::runtime_error("expected 3 path-B scales");
    }
  }

  int input_width_ = 640;
  int input_height_ = 640;
  uint32_t output_count_ = 0;
  std::vector<OutputSlot> output_slots_;
  rknn_context context_ = 0;
  rknn_tensor_mem *input_mem_ = nullptr;
  rknn_tensor_attr native_input_attr_{};
  cv::Mat input_fp16_view_;
  bool zero_copy_input_ = false;
  std::vector<unsigned char> model_data_;
  InferenceTimingStats last_timing_;
  cv::Mat resized_buffer_;
  cv::Mat letterbox_buffer_;
};


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
  : Node("package_qr_shelf_color_probe")
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
  : Node("package_qr_shelf_rknn_stream"),
    started_at_(Clock::now())
  {
    constexpr std::array<rknn_core_mask, kWorkerCount> core_masks{
      RKNN_NPU_CORE_0, RKNN_NPU_CORE_1, RKNN_NPU_CORE_2};
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      detectors_[index] = std::make_unique<RknnYolo11PathbDetector>(model_path, core_masks[index]);
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
      "Streaming 3-context path-B package/qr/shelf RKNN inference started: color=%s depth=%s model=%s",
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
    RknnYolo11PathbDetector::InferenceTimingStats timing;
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
      RknnYolo11PathbDetector &detector = *detectors_[worker_index];
      DepthProcessor &depth_processor = depth_processors_[worker_index];
      const auto image = cv_bridge::toCvShare(frame.color, "rgb8");
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

      cv::Mat display;
      cv::cvtColor(image->image, display, cv::COLOR_RGB2BGR);

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
        const char *class_name = RknnYolo11PathbDetector::className(detection.class_id);
        const std::string label = sample.has_valid_depth
          ? cv::format("%s %.2f  %.2fm XYZ(%.2f,%.2f,%.2f)",
            class_name, detection.score, sample.depth_m, detection.point_3d.x,
            detection.point_3d.y, detection.point_3d.z)
          : cv::format("%s %.2f  depth n/a", class_name, detection.score);
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
    const std::string timing_status = cv::format(
      "Pre %.2f  In %.2f  Run %.2f  Out %.2f  Post %.2f  Total %.2f ms",
      result.timing.preprocess_ms, result.timing.input_prepare_ms,
      result.timing.rknn_run_ms, result.timing.output_get_ms,
      result.timing.postprocess_ms, result.timing.detector_total_ms);
    cv::putText(result.display, timing_status, cv::Point(12, 82), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string stream_status = result.depth_ready
      ? cv::format("RGB %dx%d  raw Depth %dx%d %s", result.display.cols, result.display.rows,
        result.depth_width, result.depth_height, result.depth_encoding.c_str())
      : cv::format("RGB %dx%d  raw Depth unavailable/stale",
        result.display.cols, result.display.rows);
    cv::putText(result.display, stream_status, cv::Point(12, 108), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string depth_status = result.center_depth_valid
      ? cv::format("Center depth %.3fm  raw-depth registered in node", result.center_depth_m)
      : std::string("Center depth n/a  raw-depth registration");
    cv::putText(result.display, depth_status, cv::Point(12, 134), cv::FONT_HERSHEY_SIMPLEX,
      0.54, text_color, 1, cv::LINE_AA);
    const std::string memory_status = cv::format(
      "3-context path-B RKNN memory: weight %.1f MiB  internal %.1f MiB  DMA %.1f MiB",
      weight_mib_, internal_mib_, dma_mib_);
    cv::putText(result.display, memory_status, cv::Point(12, 160), cv::FONT_HERSHEY_SIMPLEX,
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
        "core_ms=[%.2f,%.2f,%.2f] preprocess_ms=%.2f input_prepare_ms=%.2f "
        "rknn_run_ms=%.2f output_get_ms=%.2f postprocess_ms=%.2f detector_total_ms=%.2f "
        "latest_frame=%llu worker=%zu detections=%zu",
        static_cast<unsigned long long>(processed),
        static_cast<unsigned long long>(received_count_.load()),
        static_cast<unsigned long long>(dropped_count_.load()),
        static_cast<unsigned long long>(stale_result_count_.load()),
        input_fps_.load(), process_fps, display_fps_, npu_capacity_fps,
        run_ms[0], run_ms[1], run_ms[2],
        result.timing.preprocess_ms, result.timing.input_prepare_ms,
        result.timing.rknn_run_ms, result.timing.output_get_ms,
        result.timing.postprocess_ms, result.timing.detector_total_ms,
        static_cast<unsigned long long>(result.frame_id), result.worker_index,
        result.detection_count);
      last_report_processed_ = processed;
      last_report_at_ = now;
    }
  }

  std::array<std::unique_ptr<RknnYolo11PathbDetector>, kWorkerCount> detectors_;
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
  const std::string window_name_ = "Package/QR/Shelf RKNN PathB";
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
      "/models/package_qrcode_shelf_tag_fp16.rknn";
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
    std::cerr << "Usage (path-B 9-output YOLO11: qrcode/package/shelf_tag):\n"
              << "  " << argv[0] << " <model.rknn>\n"
              << "  " << argv[0] << " --infer <model.rknn> [--ros-args -p color_topic:=<topic>]\n"
              << "  " << argv[0] << " --camera [--ros-args -p color_topic:=<topic>]\n"
              << "Default model: share/drone_perception/models/package_qrcode_shelf_tag_fp16.rknn\n";
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

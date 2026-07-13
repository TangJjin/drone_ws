#include "drone_perception/rknn_yolo_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
using SteadyClock = std::chrono::steady_clock;

double elapsedMs(
  const SteadyClock::time_point &start,
  const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void printTensorAttr(const char *prefix, const rknn_tensor_attr &attr)
{
  std::cout << "[RKNN-PATHB] " << prefix << "[" << attr.index << "]"
            << " name=" << attr.name << " n_dims=" << attr.n_dims << " dims=(";
  for (uint32_t i = 0; i < attr.n_dims; ++i) {
    std::cout << attr.dims[i] << (i + 1 < attr.n_dims ? "," : "");
  }
  std::cout << ") type=" << attr.type << " fmt=" << attr.fmt
            << " n_elems=" << attr.n_elems << std::endl;
}

struct RknnOutputGuard
{
  rknn_context context = 0;
  uint32_t output_count = 0;
  rknn_output *outputs = nullptr;

  ~RknnOutputGuard()
  {
    if (outputs != nullptr && output_count > 0) {
      rknn_outputs_release(context, output_count, outputs);
    }
  }

  RknnOutputGuard(const RknnOutputGuard &) = delete;
  RknnOutputGuard &operator=(const RknnOutputGuard &) = delete;
};
}  // namespace

RknnYoloDetector::RknnYoloDetector(
  const std::string &model_path,
  rknn_core_mask core_mask)
{
  loadModel(model_path, core_mask);
}

RknnYoloDetector::~RknnYoloDetector()
{
  if (input_mem_ != nullptr && context_ != 0) {
    rknn_destroy_mem(context_, input_mem_);
    input_mem_ = nullptr;
  }
  if (context_ != 0) {
    rknn_destroy(context_);
    context_ = 0;
  }
}

const RknnYoloDetector::InferenceTimingStats &RknnYoloDetector::lastTiming() const
{
  return last_timing_;
}

rknn_mem_size RknnYoloDetector::memorySize() const
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

double RknnYoloDetector::lastRknnRunMs() const
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

const char *RknnYoloDetector::className(int class_id)
{
  static constexpr const char *kNames[kClassCount] = {
    "qrcode", "package", "shelf_tag"};
  if (class_id < 0 || class_id >= kClassCount) {
    return "unknown";
  }
  return kNames[class_id];
}

std::vector<unsigned char> RknnYoloDetector::readFile(const std::string &path)
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

bool RknnYoloDetector::parseSpatialChannels(
  const rknn_tensor_attr &attr,
  int &channels,
  int &height,
  int &width,
  Layout &layout)
{
  if (attr.n_dims != 4) {
    return false;
  }
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
  const int d1 = static_cast<int>(attr.dims[1]);
  const int d2 = static_cast<int>(attr.dims[2]);
  const int d3 = static_cast<int>(attr.dims[3]);
  const auto is_channel = [](int v) { return v == 1 || v == 3 || v == 64; };
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
  layout = Layout::NCHW;
  channels = d1;
  height = d2;
  width = d3;
  return channels > 0 && height > 0 && width > 0;
}

float RknnYoloDetector::readValue(
  const BranchTensor &tensor, int channel, int y, int x)
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

void RknnYoloDetector::dflDecodeBox(
  const BranchTensor &box,
  int y,
  int x,
  float stride_x,
  float stride_y,
  float *xyxy)
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

std::vector<Detection> RknnYoloDetector::parseBranches(
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

RknnYoloDetector::LetterboxResult RknnYoloDetector::makeLetterbox(
  const cv::Mat &rgb_image)
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

void RknnYoloDetector::configureZeroCopyInput()
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

void RknnYoloDetector::loadModel(
  const std::string &model_path,
  rknn_core_mask core_mask)
{
  model_data_ = readFile(model_path);
  const int ret = rknn_init(
    &context_, model_data_.data(),
    static_cast<unsigned int>(model_data_.size()),
    RKNN_FLAG_PRIOR_HIGH,
    nullptr);
  if (ret != RKNN_SUCC) {
    throw std::runtime_error("rknn_init failed, ret=" + std::to_string(ret));
  }
  const int core_ret = rknn_set_core_mask(context_, core_mask);
  if (core_ret != RKNN_SUCC) {
    throw std::runtime_error(
      "rknn_set_core_mask failed, ret=" + std::to_string(core_ret));
  }
  std::cout << "[RKNN-PATHB] NPU core mask: " << static_cast<int>(core_mask)
            << ", init_flag=PRIOR_HIGH" << std::endl;

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

  configureZeroCopyInput();

  std::cout << "[RKNN-PATHB] model ready: " << input_width_ << "x" << input_height_
            << ", classes=3 (qrcode/package/shelf_tag), conf=" << kConfThresh
            << " nms=" << kNmsThresh
            << ", zero_copy=" << (zero_copy_input_ ? "on" : "off") << std::endl;
}

void RknnYoloDetector::bindScaleBranches(
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

std::vector<Detection> RknnYoloDetector::infer(const cv::Mat &rgb_image)
{
  InferenceTimingStats timing;
  const auto detector_t0 = SteadyClock::now();

  const auto preprocess_t0 = SteadyClock::now();
  const LetterboxResult letterbox = makeLetterbox(rgb_image);
  const auto preprocess_t1 = SteadyClock::now();
  timing.preprocess_ms = elapsedMs(preprocess_t0, preprocess_t1);

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

  RknnOutputGuard output_guard{context_, output_count_, outputs.data()};

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

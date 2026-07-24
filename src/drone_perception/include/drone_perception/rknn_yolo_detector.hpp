#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "rknn_api.h"

#include "drone_perception/detection.hpp"

// 路径 B：YOLO11 9 输出（box/cls/sum × 3 尺度）+ DFL 后处理。
// 类别：0=qrcode, 1=package, 2=shelf_tag
// 默认 conf=0.5；按模型 native 输入类型自动选择 FP16/INT8 zero-copy。
class RknnYoloDetector
{
public:
  static constexpr int kClassCount = 3;
  static constexpr float kConfThresh = 0.5F;
  static constexpr float kNmsThresh = 0.45F;

  enum class ZeroCopyMode
  {
    Off = 0,
    Fp16 = 1,
    Int8 = 2,
    Uint8 = 3,
  };

  struct InferenceTimingStats
  {
    double preprocess_ms = 0.0;
    double input_prepare_ms = 0.0;
    double rknn_run_ms = 0.0;
    double output_get_ms = 0.0;
    double postprocess_ms = 0.0;
    double detector_total_ms = 0.0;
  };

  explicit RknnYoloDetector(
    const std::string &model_path,
    rknn_core_mask core_mask = RKNN_NPU_CORE_0_1_2,
    bool enable_zero_copy = true,
    std::vector<std::string> class_names = {"qrcode", "package", "shelf_tag"},
    float confidence_threshold = kConfThresh,
    float nms_threshold = kNmsThresh,
    int letterbox_value = 0);
  ~RknnYoloDetector();

  RknnYoloDetector(const RknnYoloDetector &) = delete;
  RknnYoloDetector &operator=(const RknnYoloDetector &) = delete;

  std::vector<Detection> infer(const cv::Mat &rgb_image);

  const InferenceTimingStats &lastTiming() const;
  rknn_mem_size memorySize() const;
  double lastRknnRunMs() const;
  ZeroCopyMode zeroCopyMode() const;
  const char *zeroCopyModeName() const;
  rknn_tensor_type nativeInputType() const;
  const std::string &classLabel(int class_id) const;
  std::size_t classCount() const;
  float confidenceThreshold() const;
  float nmsThreshold() const;
  int inputWidth() const;
  int inputHeight() const;
  uint32_t outputCount() const;
  const std::string &apiVersion() const;
  const std::string &driverVersion() const;

  static const char *className(int class_id);
  static const char *zeroCopyModeToString(ZeroCopyMode mode);

private:
  enum class Layout
  {
    NCHW = 0,
    NHWC = 1,
  };

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

  static constexpr int kDflLen = 16;
  static constexpr int kBoxChannels = 4 * kDflLen;

  LetterboxResult makeLetterbox(const cv::Mat &rgb_image);
  void loadModel(
    const std::string &model_path,
    rknn_core_mask core_mask,
    bool enable_zero_copy);
  void configureZeroCopyInput(bool enable_zero_copy);
  void fillInt8ZeroCopyInput(const cv::Mat &rgb_u8_letterbox);
  void fillUint8ZeroCopyInput(const cv::Mat &rgb_u8_letterbox);
  static std::vector<unsigned char> readFile(const std::string &path);
  static bool parseSpatialChannels(
    const rknn_tensor_attr &attr,
    int &channels,
    int &height,
    int &width,
    Layout &layout);
  static float readValue(const BranchTensor &tensor, int channel, int y, int x);
  static void dflDecodeBox(
    const BranchTensor &box,
    int y,
    int x,
    float stride_x,
    float stride_y,
    float *xyxy);
  std::vector<Detection> parseBranches(
    const std::vector<ScaleBranch> &branches,
    const cv::Size &original_size,
    float scale,
    int pad_x,
    int pad_y) const;
  void bindScaleBranches(
    const std::vector<rknn_output> &outputs,
    std::vector<ScaleBranch> &branches) const;

  int input_width_ = 640;
  int input_height_ = 640;
  uint32_t output_count_ = 0;
  std::vector<OutputSlot> output_slots_;
  std::vector<std::string> class_names_;
  float confidence_threshold_ = kConfThresh;
  float nms_threshold_ = kNmsThresh;
  int letterbox_value_ = 0;
  std::string api_version_;
  std::string driver_version_;

  rknn_context context_ = 0;
  rknn_tensor_mem *input_mem_ = nullptr;
  rknn_tensor_attr native_input_attr_{};
  cv::Mat input_fp16_view_;
  ZeroCopyMode zero_copy_mode_ = ZeroCopyMode::Off;
  rknn_tensor_type native_input_type_ = RKNN_TENSOR_FLOAT16;
  float input_qnt_scale_ = 1.0F;
  int32_t input_qnt_zp_ = 0;
  int input_width_stride_ = 640;
  std::vector<unsigned char> model_data_;
  InferenceTimingStats last_timing_;
  cv::Mat resized_buffer_;
  cv::Mat letterbox_buffer_;
};

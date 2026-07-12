#pragma once

#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "rknn_api.h"

#include "drone_perception/detection.hpp"
#include "drone_perception/yolo_postprocessor.hpp"

class RknnYoloDetector
{
public:
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
        rknn_core_mask core_mask = RKNN_NPU_CORE_0_1_2);
    ~RknnYoloDetector();

    std::vector<Detection> infer(const cv::Mat &rgb_image);

    const InferenceTimingStats &lastTiming() const;

    rknn_mem_size memorySize() const;

    double lastRknnRunMs() const;

private:
    struct LetterboxResult
    {
        cv::Mat image;
        float scale = 1.0F;
        int pad_x = 0;
        int pad_y = 0;
    };

    LetterboxResult makeLetterbox(const cv::Mat &rgb_image);

    void loadModel(const std::string &model_path, rknn_core_mask core_mask);

    void configureZeroCopyInput();

    static std::vector<unsigned char> readFile(const std::string &path);

    // 这些参数和当前 QR YOLO RKNN 模型绑定：
    // 输入 640x640，类别数 2，候选数 8400。
    // 支持单输出 [1,6,8400] 和双输出 [1,4,8400] + [1,2,8400]。
    int input_width_ = 640;
    int input_height_ = 640;
    int candidate_count_ = 8400;
    uint32_t output_count_ = 0;
    uint32_t bbox_output_index_ = 0;
    uint32_t class_output_index_ = 0;
    bool bbox_output_found_ = false;
    bool class_output_found_ = false;
    YoloPostprocessor postprocessor_{2, 0.75F, 0.35F};

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

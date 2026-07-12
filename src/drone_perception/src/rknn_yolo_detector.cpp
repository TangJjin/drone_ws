#include "drone_perception/rknn_yolo_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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
    std::cout << "[RKNN] " << prefix << "[" << attr.index << "]"
              << " name=" << attr.name
              << " n_dims=" << attr.n_dims
              << " dims=(";

    for (uint32_t i = 0; i < attr.n_dims; ++i)
    {
        std::cout << attr.dims[i];
        if (i + 1 < attr.n_dims)
        {
            std::cout << ",";
        }
    }

    std::cout << ")"
              << " type=" << attr.type
              << " fmt=" << attr.fmt
              << " qnt_type=" << attr.qnt_type
              << " zp=" << attr.zp
              << " scale=" << attr.scale
              << std::endl;
}
struct RknnOutputGuard
{
    rknn_context context;
    uint32_t output_count;
    rknn_output *outputs;
    bool active = true;

    ~RknnOutputGuard()
    {
        if(active && outputs != nullptr && output_count > 0)
        {
            rknn_outputs_release(context, output_count, outputs);
        }
    }
    RknnOutputGuard(const RknnOutputGuard &) = delete;
    RknnOutputGuard &operator=(const RknnOutputGuard&) = delete;

};

std::string outputModeName(uint32_t output_count)
{
    if (output_count == 1)
    {
        return "single-output YOLO [1,6,8400]";
    }
    if (output_count == 2)
    {
        return "split-output YOLO [1,4,8400] + [1,2,8400]";
    }
    return "unsupported";
}


}  // namespace

RknnYoloDetector::RknnYoloDetector(
    const std::string &model_path,
    rknn_core_mask core_mask)
{
    loadModel(model_path, core_mask);
}

RknnYoloDetector::~RknnYoloDetector()
{
    if (input_mem_ != nullptr)
    {
        rknn_destroy_mem(context_, input_mem_);
        input_mem_ = nullptr;
    }
    if (context_ != 0)
    {
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
    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("RKNN_QUERY_MEM_SIZE failed, ret=" + std::to_string(ret));
    }
    return memory_size;
}

double RknnYoloDetector::lastRknnRunMs() const
{
    rknn_perf_run perf_run;
    std::memset(&perf_run, 0, sizeof(perf_run));
    const int ret = rknn_query(
        context_, RKNN_QUERY_PERF_RUN, &perf_run, sizeof(perf_run));
    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("RKNN_QUERY_PERF_RUN failed, ret=" + std::to_string(ret));
    }
    return static_cast<double>(perf_run.run_duration) / 1000.0;
}

std::vector<unsigned char> RknnYoloDetector::readFile(const std::string &path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);

    if (!file)
    {
        throw std::runtime_error("failed to open model file: " + path);
    }

    const std::streamsize file_size = file.tellg();

    if (file_size <= 0)
    {
        throw std::runtime_error("model file is empty: " + path);
    }

    std::vector<unsigned char> data(static_cast<std::size_t>(file_size));
    file.seekg(0, std::ios::beg);

    if (!file.read(reinterpret_cast<char *>(data.data()), file_size))
    {
        throw std::runtime_error("failed to read model file: " + path);
    }

    return data;
}

RknnYoloDetector::LetterboxResult RknnYoloDetector::makeLetterbox(
    const cv::Mat &rgb_image)
{
    if (rgb_image.empty())
    {
        throw std::runtime_error("input image is empty");
    }

    // 保持原图比例并补边到 640x640，避免二维码/条码被拉伸变形。
    const float scale = std::min(
        static_cast<float>(input_width_) / static_cast<float>(rgb_image.cols),
        static_cast<float>(input_height_) / static_cast<float>(rgb_image.rows));

    const int resized_width = static_cast<int>(static_cast<float>(rgb_image.cols) * scale);
    const int resized_height = static_cast<int>(static_cast<float>(rgb_image.rows) * scale);

    const int pad_x = (input_width_ - resized_width) / 2;
    const int pad_y = (input_height_ - resized_height) / 2;

    letterbox_buffer_.create(input_height_, input_width_, rgb_image.type());
    letterbox_buffer_.setTo(cv::Scalar(114, 114, 114));
    cv::Mat destination = letterbox_buffer_(
        cv::Rect(pad_x, pad_y, resized_width, resized_height));
    if (resized_width == rgb_image.cols && resized_height == rgb_image.rows)
    {
        rgb_image.copyTo(destination);
    }
    else
    {
        resized_buffer_.create(resized_height, resized_width, rgb_image.type());
        cv::resize(
            rgb_image,
            resized_buffer_,
            cv::Size(resized_width, resized_height));
        resized_buffer_.copyTo(destination);
    }

    LetterboxResult result;
    result.image = letterbox_buffer_;
    result.scale = scale;
    result.pad_x = pad_x;
    result.pad_y = pad_y;

    return result;
}

void RknnYoloDetector::loadModel(
    const std::string &model_path,
    rknn_core_mask core_mask)
{
    model_data_ = readFile(model_path);

    // rknn_init 会把 .rknn 模型加载进 RKNN Runtime，必须在 RK3588/OrangePi 上运行。
    const int ret = rknn_init(
        &context_,
        model_data_.data(),
        static_cast<unsigned int>(model_data_.size()),
        0,
        nullptr);

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_init failed, ret=" + std::to_string(ret));
    }

    const int core_ret = rknn_set_core_mask(context_, core_mask);
    if (core_ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_set_core_mask failed, ret=" + std::to_string(core_ret));
    }
    std::cout << "[RKNN] NPU core mask: " << static_cast<int>(core_mask) << std::endl;

    rknn_input_output_num io_num;
    std::memset(&io_num, 0, sizeof(io_num));

    int query_ret = rknn_query(
        context_,
        RKNN_QUERY_IN_OUT_NUM,
        &io_num,
        sizeof(io_num));

    if (query_ret != RKNN_SUCC)
    {
        throw std::runtime_error("RKNN_QUERY_IN_OUT_NUM failed, ret=" + std::to_string(query_ret));
    }

    std::cout << "[RKNN] input num: " << io_num.n_input
              << ", output num: " << io_num.n_output << std::endl;

    if (io_num.n_output != 1 && io_num.n_output != 2)
    {
        std::ostringstream message;
        message << "unsupported RKNN output count: " << io_num.n_output
                << ", expected 1 for FP16 single-output model or 2 for INT8 split-output model";
        throw std::runtime_error(message.str());
    }

    output_count_ = io_num.n_output;
    std::cout << "[RKNN] output mode: " << outputModeName(output_count_) << std::endl;

    for (uint32_t i = 0; i < io_num.n_input; ++i)
    {
        rknn_tensor_attr attr;
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;

        query_ret = rknn_query(
            context_,
            RKNN_QUERY_INPUT_ATTR,
            &attr,
            sizeof(attr));

        if (query_ret != RKNN_SUCC)
        {
            throw std::runtime_error("RKNN_QUERY_INPUT_ATTR failed, ret=" + std::to_string(query_ret));
        }

        printTensorAttr("input", attr);
    }

    for (uint32_t i = 0; i < io_num.n_output; ++i)
    {
        rknn_tensor_attr attr;
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;

        query_ret = rknn_query(
            context_,
            RKNN_QUERY_OUTPUT_ATTR,
            &attr,
            sizeof(attr));

        if (query_ret != RKNN_SUCC)
        {
            throw std::runtime_error("RKNN_QUERY_OUTPUT_ATTR failed, ret=" + std::to_string(query_ret));
        }

        printTensorAttr("output", attr);

        if (output_count_ == 2)
        {
            const uint32_t bbox_elems = static_cast<uint32_t>(4 * candidate_count_);
            const uint32_t class_elems = static_cast<uint32_t>(2 * candidate_count_);
            if (attr.n_elems == bbox_elems)
            {
                bbox_output_index_ = i;
                bbox_output_found_ = true;
            }
            else if (attr.n_elems == class_elems)
            {
                class_output_index_ = i;
                class_output_found_ = true;
            }
        }
    }

    if (output_count_ == 2)
    {
        if (!bbox_output_found_ || !class_output_found_ ||
            bbox_output_index_ == class_output_index_)
        {
            std::ostringstream message;
            message << "failed to identify split RKNN outputs, bbox_index="
                    << bbox_output_index_
                    << ", class_index=" << class_output_index_
                    << ", expected output element counts "
                    << 4 * candidate_count_ << " and "
                    << 2 * candidate_count_;
            throw std::runtime_error(message.str());
        }

        std::cout << "[RKNN] split output role: bbox=output["
                  << bbox_output_index_ << "], class=output["
                  << class_output_index_ << "]" << std::endl;
    }

    configureZeroCopyInput();
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
        std::cout << "[RKNN] native FP16 zero-copy input unavailable; "
                  << "using rknn_inputs_set fallback" << std::endl;
        return;
    }

    native_input_attr_.pass_through = 1;
    input_mem_ = rknn_create_mem(context_, native_input_attr_.size_with_stride);
    if (input_mem_ == nullptr)
    {
        throw std::runtime_error("rknn_create_mem for zero-copy input failed");
    }
    const int set_ret = rknn_set_io_mem(context_, input_mem_, &native_input_attr_);
    if (set_ret != RKNN_SUCC)
    {
        rknn_destroy_mem(context_, input_mem_);
        input_mem_ = nullptr;
        throw std::runtime_error(
            "rknn_set_io_mem for zero-copy input failed, ret=" +
            std::to_string(set_ret));
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
    std::cout << "[RKNN] native FP16 zero-copy input enabled: "
              << width << "x" << height
              << " stride=" << width_stride
              << " bytes=" << native_input_attr_.size_with_stride << std::endl;
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
    if (zero_copy_input_)
    {
        letterbox.image.convertTo(input_fp16_view_, CV_16FC3, 1.0 / 255.0);
        ret = rknn_mem_sync(context_, input_mem_, RKNN_MEMORY_SYNC_TO_DEVICE);
        if (ret != RKNN_SUCC)
        {
            throw std::runtime_error(
                "rknn_mem_sync input failed, ret=" + std::to_string(ret));
        }
    }
    else
    {
        rknn_input input;
        std::memset(&input, 0, sizeof(input));
        input.index = 0;
        input.type = RKNN_TENSOR_UINT8;
        input.size = static_cast<unsigned int>(input_width_ * input_height_ * 3);
        input.fmt = RKNN_TENSOR_NHWC;
        input.buf = letterbox.image.data;
        ret = rknn_inputs_set(context_, 1, &input);
        if (ret != RKNN_SUCC)
        {
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

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_run failed, ret=" + std::to_string(ret));
    }

    if (output_count_ != 1 && output_count_ != 2)
    {
        throw std::runtime_error("RKNN output mode is not initialized");
    }

    std::vector<rknn_output> outputs(output_count_);
    for (uint32_t i = 0; i < output_count_; ++i)
    {
        std::memset(&outputs[i], 0, sizeof(outputs[i]));
        outputs[i].index = i;
        // 让 Runtime 把输出反量化成 float，FP16/INT8 后处理共用 float 路径。
        outputs[i].want_float = 1;
    }

    const auto output_get_t0 = SteadyClock::now();
    ret = rknn_outputs_get(context_, output_count_, outputs.data(), nullptr);
    const auto output_get_t1 = SteadyClock::now();
    timing.output_get_ms = elapsedMs(output_get_t0, output_get_t1);

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_outputs_get failed, ret=" + std::to_string(ret));
    }

    RknnOutputGuard output_guard{context_, output_count_, outputs.data()};

    const auto postprocess_t0 = SteadyClock::now();
    std::vector<Detection> detections;
    if (output_count_ == 1)
    {
        const float *output_data = static_cast<const float *>(outputs[0].buf);
        detections = postprocessor_.parseOutput(
            output_data,
            candidate_count_,
            rgb_image.size(),
            letterbox.scale,
            letterbox.pad_x,
            letterbox.pad_y);
    }
    else
    {
        const float *bbox_data = static_cast<const float *>(outputs[bbox_output_index_].buf);
        const float *class_data = static_cast<const float *>(outputs[class_output_index_].buf);
        detections = postprocessor_.parseSplitOutput(
            bbox_data,
            class_data,
            candidate_count_,
            rgb_image.size(),
            letterbox.scale,
            letterbox.pad_x,
            letterbox.pad_y);
    }

    const auto postprocess_t1 = SteadyClock::now();
    timing.postprocess_ms = elapsedMs(postprocess_t0, postprocess_t1);
    timing.detector_total_ms = elapsedMs(detector_t0, postprocess_t1);
    last_timing_ = timing;

    return detections;
}

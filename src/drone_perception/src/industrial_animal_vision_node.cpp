#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <linux/videodev2.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drone_perception/industrial_animal_vision_node.hpp"
#include "drone_perception/rknn_yolo_detector.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::size_t kWorkerCount = 3;
constexpr std::size_t kTaskQueueCapacity = 3;

struct GstSampleDeleter
{
  void operator()(GstSample *sample) const
  {
    if (sample != nullptr) {
      gst_sample_unref(sample);
    }
  }
};

using SamplePtr = std::shared_ptr<GstSample>;

std::string shellQuote(const std::string &value)
{
  gchar *quoted = g_shell_quote(value.c_str());
  const std::string result = quoted == nullptr ? value : quoted;
  g_free(quoted);
  return result;
}

double elapsedSeconds(const Clock::time_point &start, const Clock::time_point &end)
{
  return std::chrono::duration<double>(end - start).count();
}

class IndustrialAnimalVisionNode : public rclcpp::Node
{
public:
  IndustrialAnimalVisionNode()
  : Node("industrial_animal_vision"), started_at_(Clock::now()), last_report_at_(started_at_)
  {
    declareParameters();
    readParameters();
    validateParameters();
    configureProcessAffinity();
    configureCameraControls();
    startPipeline();
    initializeDetectors();

    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      workers_[index] = std::thread(&IndustrialAnimalVisionNode::workerLoop, this, index);
    }
    capture_thread_ = std::thread(&IndustrialAnimalVisionNode::captureLoop, this);
    if (display_enabled_) {
      ui_thread_ = std::thread(&IndustrialAnimalVisionNode::uiLoop, this);
    }

    RCLCPP_INFO(
      get_logger(),
      "Industrial animal vision ready: profile=%s model=%s camera=%s "
      "request=MJPEG %dx%d@%d display=%s",
      camera_profile_.c_str(), model_path_.c_str(), camera_device_.c_str(),
      camera_width_, camera_height_, camera_fps_, display_enabled_ ? "on" : "off");
  }

  ~IndustrialAnimalVisionNode() override
  {
    running_.store(false);
    task_ready_.notify_all();
    result_ready_.notify_all();

    if (pipeline_ != nullptr) {
      gst_element_send_event(pipeline_, gst_event_new_eos());
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    for (std::thread &worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    if (ui_thread_.joinable()) {
      ui_thread_.join();
    }
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (app_sink_ != nullptr) {
      gst_object_unref(app_sink_);
      app_sink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

private:
  struct FrameTask
  {
    std::uint64_t frame_id = 0;
    SamplePtr sample;
  };

  struct InferenceResult
  {
    std::uint64_t frame_id = 0;
    std::size_t worker_index = 0;
    SamplePtr sample;
    std::vector<Detection> detections;
    RknnYoloDetector::InferenceTimingStats timing;
    double api_run_ms = 0.0;
  };

  void declareParameters()
  {
    declare_parameter<std::string>("camera_profile", "performance_230");
    declare_parameter<std::string>("camera_device", "/dev/video1");
    declare_parameter<std::string>("model_path", "");
    declare_parameter<int>("camera_width", 1280);
    declare_parameter<int>("camera_height", 720);
    declare_parameter<int>("camera_fps", 230);
    declare_parameter<int>("decode_width", 640);
    declare_parameter<int>("decode_height", 360);
    declare_parameter<int>("exposure_auto", 1);
    declare_parameter<int>("exposure_absolute", 40);
    declare_parameter<int>("exposure_auto_priority", 0);
    declare_parameter<int>("gain", 190);
    declare_parameter<int>("brightness", 128);
    declare_parameter<int>("contrast", 65);
    declare_parameter<int>("saturation", 90);
    declare_parameter<int>("gamma", 130);
    declare_parameter<int>("sharpness", 128);
    declare_parameter<int>("backlight_compensation", 16);
    declare_parameter<int>("white_balance_auto", 1);
    declare_parameter<int>("white_balance_temperature", 4650);
    declare_parameter<int>("power_line_frequency", 1);
    declare_parameter<int>("focus_auto", 1);
    declare_parameter<int>("focus_absolute", 0);
    declare_parameter<int>("zoom_absolute", 120);
    declare_parameter<bool>("display_enabled", true);
    declare_parameter<double>("display_fps_limit", 60.0);
    declare_parameter<double>("confidence_threshold", 0.5);
    declare_parameter<double>("nms_threshold", 0.45);
    declare_parameter<bool>("enable_zero_copy", true);
    declare_parameter<bool>("cpu_affinity_enabled", true);
  }

  void readParameters()
  {
    camera_profile_ = get_parameter("camera_profile").as_string();
    camera_device_ = get_parameter("camera_device").as_string();
    model_path_ = get_parameter("model_path").as_string();
    camera_width_ = static_cast<int>(get_parameter("camera_width").as_int());
    camera_height_ = static_cast<int>(get_parameter("camera_height").as_int());
    camera_fps_ = static_cast<int>(get_parameter("camera_fps").as_int());
    decode_width_ = static_cast<int>(get_parameter("decode_width").as_int());
    decode_height_ = static_cast<int>(get_parameter("decode_height").as_int());
    exposure_auto_ = static_cast<int>(get_parameter("exposure_auto").as_int());
    exposure_absolute_ = static_cast<int>(get_parameter("exposure_absolute").as_int());
    exposure_auto_priority_ = static_cast<int>(get_parameter("exposure_auto_priority").as_int());
    gain_ = static_cast<int>(get_parameter("gain").as_int());
    brightness_ = static_cast<int>(get_parameter("brightness").as_int());
    contrast_ = static_cast<int>(get_parameter("contrast").as_int());
    saturation_ = static_cast<int>(get_parameter("saturation").as_int());
    gamma_ = static_cast<int>(get_parameter("gamma").as_int());
    sharpness_ = static_cast<int>(get_parameter("sharpness").as_int());
    backlight_compensation_ = static_cast<int>(get_parameter("backlight_compensation").as_int());
    white_balance_auto_ = static_cast<int>(get_parameter("white_balance_auto").as_int());
    white_balance_temperature_ = static_cast<int>(
      get_parameter("white_balance_temperature").as_int());
    power_line_frequency_ = static_cast<int>(get_parameter("power_line_frequency").as_int());
    focus_auto_ = static_cast<int>(get_parameter("focus_auto").as_int());
    focus_absolute_ = static_cast<int>(get_parameter("focus_absolute").as_int());
    zoom_absolute_ = static_cast<int>(get_parameter("zoom_absolute").as_int());
    display_enabled_ = get_parameter("display_enabled").as_bool();
    display_fps_limit_ = get_parameter("display_fps_limit").as_double();
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    nms_threshold_ = static_cast<float>(get_parameter("nms_threshold").as_double());
    enable_zero_copy_ = get_parameter("enable_zero_copy").as_bool();
    cpu_affinity_enabled_ = get_parameter("cpu_affinity_enabled").as_bool();
  }

  void validateParameters() const
  {
    if (camera_device_.empty() || model_path_.empty()) {
      throw std::invalid_argument("camera_device and model_path must not be empty");
    }
    if (camera_width_ <= 0 || camera_height_ <= 0 || camera_fps_ <= 0 ||
      decode_width_ <= 0 || decode_height_ <= 0)
    {
      throw std::invalid_argument("camera/decode dimensions and camera_fps must be positive");
    }
    if (display_fps_limit_ < 0.0) {
      throw std::invalid_argument("display_fps_limit must be >= 0");
    }
  }

  void configureProcessAffinity()
  {
    if (!cpu_affinity_enabled_) {
      return;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int cpu = 4; cpu <= 7; ++cpu) {
      CPU_SET(cpu, &set);
    }
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
      RCLCPP_WARN(get_logger(), "Unable to set process CPU affinity to CPU4-7: %s", strerror(errno));
    } else {
      RCLCPP_INFO(get_logger(), "Process CPU affinity set to CPU4-7");
    }
  }

  void setCameraControl(int fd, std::uint32_t id, int value, const char *name)
  {
    v4l2_queryctrl query{};
    query.id = id;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &query) != 0 || (query.flags & V4L2_CTRL_FLAG_DISABLED) != 0) {
      RCLCPP_WARN(get_logger(), "V4L2 control unavailable: %s", name);
      return;
    }
    const int clamped = std::clamp(value, query.minimum, query.maximum);
    v4l2_control control{};
    control.id = id;
    control.value = clamped;
    if (ioctl(fd, VIDIOC_S_CTRL, &control) != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set V4L2 %s=%d: %s", name, clamped, strerror(errno));
      return;
    }
    RCLCPP_INFO(get_logger(), "V4L2 %s=%d%s", name, clamped, clamped == value ? "" : " (clamped)");
  }

  void configureCameraControls()
  {
    const int fd = open(camera_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
      throw std::runtime_error("failed to open camera controls on " + camera_device_ +
        ": " + strerror(errno));
    }
    setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO, exposure_auto_, "exposure_auto");
    setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY, exposure_auto_priority_,
      "exposure_auto_priority");
    if (exposure_auto_ == V4L2_EXPOSURE_MANUAL) {
      setCameraControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute_,
        "exposure_absolute");
    }
    setCameraControl(fd, V4L2_CID_GAIN, gain_, "gain");
    setCameraControl(fd, V4L2_CID_BRIGHTNESS, brightness_, "brightness");
    setCameraControl(fd, V4L2_CID_CONTRAST, contrast_, "contrast");
    setCameraControl(fd, V4L2_CID_SATURATION, saturation_, "saturation");
    setCameraControl(fd, V4L2_CID_GAMMA, gamma_, "gamma");
    setCameraControl(fd, V4L2_CID_SHARPNESS, sharpness_, "sharpness");
    setCameraControl(fd, V4L2_CID_BACKLIGHT_COMPENSATION, backlight_compensation_,
      "backlight_compensation");
    setCameraControl(fd, V4L2_CID_AUTO_WHITE_BALANCE, white_balance_auto_,
      "white_balance_auto");
    if (white_balance_auto_ == 0) {
      setCameraControl(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE, white_balance_temperature_,
        "white_balance_temperature");
    }
    setCameraControl(fd, V4L2_CID_POWER_LINE_FREQUENCY, power_line_frequency_,
      "power_line_frequency");
    setCameraControl(fd, V4L2_CID_FOCUS_AUTO, focus_auto_, "focus_auto");
    if (focus_auto_ == 0) {
      setCameraControl(fd, V4L2_CID_FOCUS_ABSOLUTE, focus_absolute_, "focus_absolute");
    }
    setCameraControl(fd, V4L2_CID_ZOOM_ABSOLUTE, zoom_absolute_, "zoom_absolute");
    close(fd);
  }

  void initializeDetectors()
  {
    constexpr std::array<rknn_core_mask, kWorkerCount> masks{
      RKNN_NPU_CORE_0, RKNN_NPU_CORE_1, RKNN_NPU_CORE_2};
    const std::vector<std::string> class_names{
      "elephant", "tiger", "wolf", "peacock", "monkey"};
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      detectors_[index] = std::make_unique<RknnYoloDetector>(
        model_path_, masks[index], enable_zero_copy_, class_names,
        confidence_threshold_, nms_threshold_, 114);
      const rknn_mem_size memory = detectors_[index]->memorySize();
      weight_mib_ += static_cast<double>(memory.total_weight_size) / (1024.0 * 1024.0);
      internal_mib_ += static_cast<double>(memory.total_internal_size) / (1024.0 * 1024.0);
      dma_mib_ += static_cast<double>(memory.total_dma_allocated_size) / (1024.0 * 1024.0);
    }
    api_version_ = detectors_[0]->apiVersion();
    driver_version_ = detectors_[0]->driverVersion();
    zero_copy_mode_ = detectors_[0]->zeroCopyModeName();
    model_input_width_ = detectors_[0]->inputWidth();
    model_input_height_ = detectors_[0]->inputHeight();
    model_output_count_ = detectors_[0]->outputCount();
  }

  std::string buildPipelineDescription() const
  {
    std::ostringstream pipeline;
    pipeline << "v4l2src device=" << shellQuote(camera_device_)
             << " io-mode=2 do-timestamp=true ! "
             << "image/jpeg,width=" << camera_width_
             << ",height=" << camera_height_
             << ",framerate=" << camera_fps_ << "/1 ! "
             << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
             << "jpegparse ! mppjpegdec name=jpeg_decoder format=RGB dma-feature=false ! "
             << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
             << "appsink name=camera_sink emit-signals=false sync=false max-buffers=1 drop=true";
    return pipeline.str();
  }

  void startPipeline()
  {
    int argc = 0;
    char **argv = nullptr;
    gst_init(&argc, &argv);
    pipeline_description_ = buildPipelineDescription();
    RCLCPP_INFO(get_logger(), "Starting internal GStreamer/MPP pipeline: %s",
      pipeline_description_.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
    if (pipeline_ == nullptr || error != nullptr) {
      const std::string message = error == nullptr ? "unknown parse error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("GStreamer pipeline parse failed: " + message);
    }

    GstElement *jpeg_decoder = gst_bin_get_by_name(GST_BIN(pipeline_), "jpeg_decoder");
    if (jpeg_decoder == nullptr) {
      throw std::runtime_error("GStreamer MPP decoder jpeg_decoder not found");
    }
    GObjectClass *decoder_class = G_OBJECT_GET_CLASS(jpeg_decoder);
    const bool supports_resize =
      g_object_class_find_property(decoder_class, "width") != nullptr &&
      g_object_class_find_property(decoder_class, "height") != nullptr;
    if (supports_resize) {
      g_object_set(
        G_OBJECT(jpeg_decoder),
        "width", static_cast<guint>(decode_width_),
        "height", static_cast<guint>(decode_height_),
        nullptr);
      RCLCPP_INFO(
        get_logger(), "MPP decoder resize configured through GObject: %dx%d",
        decode_width_, decode_height_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "MPP decoder has no width/height properties; using original decoded size");
    }
    gst_object_unref(jpeg_decoder);

    app_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "camera_sink");
    if (app_sink_ == nullptr || !GST_IS_APP_SINK(app_sink_)) {
      throw std::runtime_error("GStreamer appsink camera_sink not found");
    }
    const GstStateChangeReturn state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("GStreamer pipeline failed to enter PLAYING state");
    }
  }

  void captureLoop()
  {
    while (running_.load() && rclcpp::ok()) {
      GstSample *raw_sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(app_sink_), 100 * GST_MSECOND);
      if (raw_sample == nullptr) {
        if (gst_app_sink_is_eos(GST_APP_SINK(app_sink_))) {
          RCLCPP_ERROR(get_logger(), "GStreamer camera pipeline reached EOS");
          rclcpp::shutdown();
          break;
        }
        reportBusError();
        continue;
      }
      SamplePtr sample(raw_sample, GstSampleDeleter{});
      GstVideoInfo info;
      if (!sampleVideoInfo(sample.get(), info)) {
        continue;
      }
      const auto now = Clock::now();
      if (last_capture_at_ != Clock::time_point{}) {
        const double seconds = elapsedSeconds(last_capture_at_, now);
        if (seconds > 0.0) {
          const double instant_fps = 1.0 / seconds;
          const double old_fps = capture_fps_.load();
          capture_fps_.store(old_fps <= 0.0 ? instant_fps : 0.9 * old_fps + 0.1 * instant_fps);
        }
      }
      last_capture_at_ = now;
      actual_width_.store(static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)));
      actual_height_.store(static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info)));
      received_count_.fetch_add(1);

      FrameTask task;
      task.frame_id = next_frame_id_.fetch_add(1);
      task.sample = std::move(sample);
      {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (task_queue_.size() >= kTaskQueueCapacity) {
          task_queue_.pop_front();
          dropped_count_.fetch_add(1);
        }
        task_queue_.push_back(std::move(task));
      }
      task_ready_.notify_one();
    }
  }

  bool sampleVideoInfo(GstSample *sample, GstVideoInfo &info)
  {
    GstCaps *caps = gst_sample_get_caps(sample);
    if (caps == nullptr || !gst_video_info_from_caps(&info, caps)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unable to read video caps from GStreamer sample");
      return false;
    }
    if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "MPP output format is %s, expected RGB",
        gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&info)));
      return false;
    }
    return true;
  }

  void reportBusError()
  {
    GstBus *bus = gst_element_get_bus(pipeline_);
    GstMessage *message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return;
    }
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError *error = nullptr;
      gchar *debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      RCLCPP_ERROR(get_logger(), "GStreamer error from %s: %s; debug=%s",
        GST_OBJECT_NAME(message->src), error == nullptr ? "unknown" : error->message,
        debug == nullptr ? "n/a" : debug);
      g_clear_error(&error);
      g_free(debug);
      running_.store(false);
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR(get_logger(), "GStreamer bus reported EOS");
      running_.store(false);
      rclcpp::shutdown();
    }
    gst_message_unref(message);
  }

  void workerLoop(std::size_t worker_index)
  {
    configureWorkerAffinity(worker_index);
    while (running_.load() && rclcpp::ok()) {
      FrameTask task;
      {
        std::unique_lock<std::mutex> lock(task_mutex_);
        task_ready_.wait(lock, [this] {
          return !task_queue_.empty() || !running_.load() || !rclcpp::ok();
        });
        if (!running_.load() || !rclcpp::ok()) {
          break;
        }
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      }
      processTask(std::move(task), worker_index);
    }
  }

  void configureWorkerAffinity(std::size_t worker_index)
  {
    if (!cpu_affinity_enabled_) {
      return;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(5 + static_cast<int>(worker_index), &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      RCLCPP_WARN(get_logger(), "Unable to bind RKNN worker %zu to CPU%zu",
        worker_index, worker_index + 5);
    }
  }

  void processTask(FrameTask task, std::size_t worker_index)
  {
    try {
      GstVideoInfo info;
      if (!sampleVideoInfo(task.sample.get(), info)) {
        return;
      }
      GstVideoFrame frame;
      std::memset(&frame, 0, sizeof(frame));
      GstBuffer *buffer = gst_sample_get_buffer(task.sample.get());
      if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
        throw std::runtime_error("gst_video_frame_map failed");
      }
      struct FrameUnmapper
      {
        GstVideoFrame *frame;
        ~FrameUnmapper()
        {
          gst_video_frame_unmap(frame);
        }
      } frame_unmapper{&frame};
      cv::Mat rgb(
        static_cast<int>(GST_VIDEO_FRAME_HEIGHT(&frame)),
        static_cast<int>(GST_VIDEO_FRAME_WIDTH(&frame)), CV_8UC3,
        GST_VIDEO_FRAME_PLANE_DATA(&frame, 0),
        static_cast<std::size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0)));
      RknnYoloDetector &detector = *detectors_[worker_index];
      std::vector<Detection> detections = detector.infer(rgb);

      InferenceResult result;
      result.frame_id = task.frame_id;
      result.worker_index = worker_index;
      result.sample = std::move(task.sample);
      result.detections = std::move(detections);
      result.timing = detector.lastTiming();
      result.api_run_ms = detector.lastRknnRunMs();
      core_run_ms_[worker_index].store(result.api_run_ms);
      processed_count_.fetch_add(1);

      if (display_enabled_) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result.frame_id <= displayed_frame_id_ ||
          (latest_result_ && result.frame_id <= latest_result_->frame_id))
        {
          stale_result_count_.fetch_add(1);
          return;
        }
        if (latest_result_) {
          stale_result_count_.fetch_add(1);
        }
        latest_result_ = std::move(result);
        result_ready_.notify_one();
      } else {
        reportBaseline(result);
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Industrial camera RKNN inference failed: %s", error.what());
    }
  }

  void uiLoop()
  {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    while (running_.load() && rclcpp::ok()) {
      std::optional<InferenceResult> result;
      {
        std::unique_lock<std::mutex> lock(result_mutex_);
        result_ready_.wait_for(lock, std::chrono::milliseconds(10), [this] {
          return latest_result_.has_value() || !running_.load() || !rclcpp::ok();
        });
        const auto now = Clock::now();
        const bool interval_ready = display_fps_limit_ <= 0.0 ||
          last_display_at_ == Clock::time_point{} ||
          elapsedSeconds(last_display_at_, now) >= 1.0 / display_fps_limit_;
        if (latest_result_ && interval_ready) {
          result = std::move(latest_result_);
          latest_result_.reset();
          displayed_frame_id_ = result->frame_id;
        }
      }
      if (result) {
        displayResult(*result);
      }
      const int key = cv::waitKey(1) & 0xff;
      if (key == 's' && !last_display_image_.empty()) {
        const std::string path = "/tmp/industrial_animal_" +
          std::to_string(last_displayed_frame_) + ".jpg";
        cv::imwrite(path, last_display_image_);
        RCLCPP_INFO(get_logger(), "Saved annotated frame: %s", path.c_str());
      } else if (key == 'q' || key == 27) {
        rclcpp::shutdown();
        break;
      }
    }
    cv::destroyWindow(window_name_);
  }

  void displayResult(const InferenceResult &result)
  {
    GstVideoInfo info;
    if (!sampleVideoInfo(result.sample.get(), info)) {
      return;
    }
    GstVideoFrame frame;
    std::memset(&frame, 0, sizeof(frame));
    if (!gst_video_frame_map(
        &frame, &info, gst_sample_get_buffer(result.sample.get()), GST_MAP_READ))
    {
      return;
    }
    cv::Mat rgb(
      static_cast<int>(GST_VIDEO_FRAME_HEIGHT(&frame)),
      static_cast<int>(GST_VIDEO_FRAME_WIDTH(&frame)), CV_8UC3,
      GST_VIDEO_FRAME_PLANE_DATA(&frame, 0),
      static_cast<std::size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0)));
    cv::Mat display;
    cv::cvtColor(rgb, display, cv::COLOR_RGB2BGR);
    gst_video_frame_unmap(&frame);

    for (const Detection &detection : result.detections) {
      cv::rectangle(display, detection.box, cv::Scalar(0, 255, 0), 2);
      const std::string label = cv::format("%s %.2f",
        detectors_[result.worker_index]->classLabel(detection.class_id).c_str(), detection.score);
      cv::putText(display, label, cv::Point(detection.box.x, std::max(20, detection.box.y - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
    }

    const auto now = Clock::now();
    if (last_display_at_ != Clock::time_point{}) {
      const double seconds = elapsedSeconds(last_display_at_, now);
      if (seconds > 0.0) {
        const double current = 1.0 / seconds;
        display_fps_ = display_fps_ <= 0.0 ? current : 0.9 * display_fps_ + 0.1 * current;
      }
    }
    last_display_at_ = now;
    drawHud(display, result);
    cv::imshow(window_name_, display);
    last_display_image_ = display;
    last_displayed_frame_ = result.frame_id;
    reportBaseline(result);
  }

  void drawHud(cv::Mat &display, const InferenceResult &result)
  {
    std::array<double, kWorkerCount> core_ms{};
    double npu_capacity_fps = 0.0;
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      core_ms[index] = core_run_ms_[index].load();
      if (core_ms[index] > 0.0) {
        npu_capacity_fps += 1000.0 / core_ms[index];
      }
    }
    const cv::Scalar color(255, 0, 255);
    std::vector<std::string> lines{
      cv::format("Capture %.1f  Process %.1f  Display %.1f  NPU cap %.1f FPS",
        capture_fps_.load(), process_fps_.load(), display_fps_, npu_capacity_fps),
      cv::format("Queue drop %llu  Stale result %llu  Frame %llu  Worker C%zu",
        static_cast<unsigned long long>(dropped_count_.load()),
        static_cast<unsigned long long>(stale_result_count_.load()),
        static_cast<unsigned long long>(result.frame_id), result.worker_index),
      cv::format("Core0 %.2f  Core1 %.2f  Core2 %.2f ms",
        core_ms[0], core_ms[1], core_ms[2]),
      cv::format("Pre %.2f  Input %.2f  Run %.2f ms",
        result.timing.preprocess_ms, result.timing.input_prepare_ms,
        result.timing.rknn_run_ms),
      cv::format("Output %.2f  Post %.2f  Total %.2f ms",
        result.timing.output_get_ms,
        result.timing.postprocess_ms, result.timing.detector_total_ms),
      cv::format("Profile %s  request MJPEG %dx%d@%d",
        camera_profile_.c_str(), camera_width_, camera_height_, camera_fps_),
      cv::format("Actual RGB %dx%d %.1f FPS  exposure %d/%d  gain %d",
        actual_width_.load(), actual_height_.load(), capture_fps_.load(),
        exposure_auto_, exposure_absolute_, gain_),
      cv::format("RKNN %dx%d in / %u out  zc=%s  detections %zu",
        model_input_width_, model_input_height_, model_output_count_, zero_copy_mode_.c_str(),
        result.detections.size()),
      cv::format("API %s  driver %s", api_version_.c_str(), driver_version_.c_str()),
      cv::format("3-context memory: weight %.1f MiB  internal %.1f MiB  DMA %.1f MiB",
        weight_mib_, internal_mib_, dma_mib_),
      "q/Esc: quit  s: save annotated frame"
    };
    int y = 22;
    for (const std::string &line : lines) {
      cv::putText(display, line, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX,
        0.43, color, 1, cv::LINE_AA);
      y += 21;
    }
  }

  void reportBaseline(const InferenceResult &result)
  {
    std::lock_guard<std::mutex> lock(report_mutex_);
    const auto now = Clock::now();
    const double report_seconds = elapsedSeconds(last_report_at_, now);
    if (report_seconds < 1.0) {
      return;
    }
    const std::uint64_t processed = processed_count_.load();
    const double process_fps = static_cast<double>(processed - last_report_processed_) /
      report_seconds;
    process_fps_.store(process_fps);
    std::array<double, kWorkerCount> core_ms{};
    double npu_capacity_fps = 0.0;
    for (std::size_t index = 0; index < kWorkerCount; ++index) {
      core_ms[index] = core_run_ms_[index].load();
      if (core_ms[index] > 0.0) {
        npu_capacity_fps += 1000.0 / core_ms[index];
      }
    }
    RCLCPP_INFO(get_logger(),
      "ANIMAL_RKNN_BASELINE profile=%s request=MJPEG_%dx%d@%d actual=RGB_%dx%d "
      "frames=%llu received=%llu queue_drop=%llu stale_result=%llu capture_fps=%.2f "
      "process_fps=%.2f display_fps=%.2f npu_capacity_fps=%.2f "
      "core_ms=[%.2f,%.2f,%.2f] pre_ms=%.2f input_ms=%.2f run_ms=%.2f "
      "output_ms=%.2f post_ms=%.2f total_ms=%.2f latest_frame=%llu worker=%zu detections=%zu",
      camera_profile_.c_str(), camera_width_, camera_height_, camera_fps_,
      actual_width_.load(), actual_height_.load(),
      static_cast<unsigned long long>(processed),
      static_cast<unsigned long long>(received_count_.load()),
      static_cast<unsigned long long>(dropped_count_.load()),
      static_cast<unsigned long long>(stale_result_count_.load()),
      capture_fps_.load(), process_fps, display_fps_, npu_capacity_fps,
      core_ms[0], core_ms[1], core_ms[2], result.timing.preprocess_ms,
      result.timing.input_prepare_ms, result.timing.rknn_run_ms,
      result.timing.output_get_ms, result.timing.postprocess_ms,
      result.timing.detector_total_ms,
      static_cast<unsigned long long>(result.frame_id), result.worker_index,
      result.detections.size());
    last_report_processed_ = processed;
    last_report_at_ = now;
  }

  std::array<std::unique_ptr<RknnYoloDetector>, kWorkerCount> detectors_;
  std::array<std::thread, kWorkerCount> workers_;
  std::thread capture_thread_;
  std::thread ui_thread_;
  GstElement *pipeline_ = nullptr;
  GstElement *app_sink_ = nullptr;
  std::mutex task_mutex_;
  std::condition_variable task_ready_;
  std::deque<FrameTask> task_queue_;
  std::mutex result_mutex_;
  std::condition_variable result_ready_;
  std::optional<InferenceResult> latest_result_;
  std::mutex report_mutex_;
  std::atomic<bool> running_{true};
  std::atomic<std::uint64_t> next_frame_id_{1};
  std::atomic<std::uint64_t> received_count_{0};
  std::atomic<std::uint64_t> dropped_count_{0};
  std::atomic<std::uint64_t> stale_result_count_{0};
  std::atomic<std::uint64_t> processed_count_{0};
  std::array<std::atomic<double>, kWorkerCount> core_run_ms_{};
  std::atomic<double> capture_fps_{0.0};
  std::atomic<double> process_fps_{0.0};
  std::atomic<int> actual_width_{0};
  std::atomic<int> actual_height_{0};
  Clock::time_point started_at_;
  Clock::time_point last_capture_at_{};
  Clock::time_point last_report_at_;
  Clock::time_point last_display_at_{};
  std::uint64_t last_report_processed_ = 0;
  std::uint64_t displayed_frame_id_ = 0;
  std::uint64_t last_displayed_frame_ = 0;
  double display_fps_ = 0.0;
  double weight_mib_ = 0.0;
  double internal_mib_ = 0.0;
  double dma_mib_ = 0.0;
  cv::Mat last_display_image_;
  std::string camera_profile_;
  std::string camera_device_;
  std::string model_path_;
  std::string pipeline_description_;
  std::string api_version_;
  std::string driver_version_;
  std::string zero_copy_mode_;
  int camera_width_ = 1280;
  int camera_height_ = 720;
  int camera_fps_ = 230;
  int decode_width_ = 640;
  int decode_height_ = 360;
  int exposure_auto_ = 1;
  int exposure_absolute_ = 40;
  int exposure_auto_priority_ = 0;
  int gain_ = 190;
  int brightness_ = 128;
  int contrast_ = 65;
  int saturation_ = 90;
  int gamma_ = 130;
  int sharpness_ = 128;
  int backlight_compensation_ = 16;
  int white_balance_auto_ = 1;
  int white_balance_temperature_ = 4650;
  int power_line_frequency_ = 1;
  int focus_auto_ = 1;
  int focus_absolute_ = 0;
  int zoom_absolute_ = 120;
  int model_input_width_ = 0;
  int model_input_height_ = 0;
  std::uint32_t model_output_count_ = 0;
  bool display_enabled_ = true;
  bool enable_zero_copy_ = true;
  bool cpu_affinity_enabled_ = true;
  double display_fps_limit_ = 60.0;
  float confidence_threshold_ = 0.5F;
  float nms_threshold_ = 0.45F;
  const std::string window_name_ = "Industrial Camera Animal RKNN";
};
}  // namespace

namespace drone_perception
{
std::shared_ptr<rclcpp::Node> makeIndustrialAnimalVisionNode()
{
  return std::make_shared<IndustrialAnimalVisionNode>();
}
}  // namespace drone_perception

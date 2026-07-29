#pragma once

#include "drone_line_vision/frame_statistics.hpp"
#include "drone_line_vision/line_vision_config.hpp"
#include "drone_line_vision/line_detector.hpp"
#include <drone_msgs/msg/line_pixel_observation.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace drone_line_vision
{
class LineVisionNode : public rclcpp::Node {
public:
  LineVisionNode();
  ~LineVisionNode() override;
private:
  bool loadConfig(const std::string & path, LineVisionConfig & out, std::string & error) const;
  void startPipeline();
  void captureLoop();
  bool sampleInfo(GstSample * sample, GstVideoInfo & info) const;
  void processSample(GstSample * sample, const GstVideoInfo & info, uint64_t frame_id);
  void display(const cv::Mat & y, const cv::Mat & mask, const LineResult & result,
    double processing_us, bool decode_ok);
  void saveCurrent(const cv::Mat & y, const cv::Mat & mask, const cv::Mat & debug,
    const LineResult & result, double processing_us);
  void stopPipeline();
  std::string pipeline_description_;
  LineVisionConfig config_; mutable std::mutex config_mutex_;
  GstElement * pipeline_{nullptr}; GstElement * app_sink_{nullptr};
  std::thread capture_thread_; std::atomic<bool> running_{false};
  std::atomic<uint64_t> frame_id_{0}; std::atomic<uint32_t> lost_frames_{0};
  FrameStatistics stats_; std::mutex display_mutex_;
  cv::Mat last_y_, last_mask_, last_debug_; LineResult last_result_; double last_processing_us_{0.0};
  rclcpp::Publisher<drone_msgs::msg::LinePixelObservation>::SharedPtr publisher_;
};
}

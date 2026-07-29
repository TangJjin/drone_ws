#pragma once

#include "drone_line_vision/frame_statistics.hpp"
#include "drone_line_vision/line_vision_config.hpp"
#include "drone_line_vision/line_detector.hpp"
#include <drone_msgs/msg/line_pixel_observation.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <mutex>
#ifdef DRONE_LINE_VISION_HAS_HBM
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#endif

namespace drone_line_vision
{
class LineVisionNode : public rclcpp::Node {
public:
  LineVisionNode();
  ~LineVisionNode() override;
private:
  bool loadConfig(const std::string & path, LineVisionConfig & out, std::string & error) const;
  void processFrame(const cv::Mat & y_plane, const rclcpp::Time & stamp, uint64_t frame_id);
  void display(const cv::Mat & y, const cv::Mat & mask, const LineResult & result,
    const drone_msgs::msg::LinePixelObservation & observation);
  void saveCurrent(const cv::Mat & y, const cv::Mat & mask, const cv::Mat & debug,
    const LineResult & result, double processing_us);
  LineVisionConfig config_; mutable std::mutex config_mutex_;
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> frame_id_{0}; std::atomic<uint32_t> lost_frames_{0};
  FrameStatistics stats_; std::mutex display_mutex_;
  std::chrono::steady_clock::time_point last_frame_time_{};
  std::chrono::steady_clock::time_point last_display_time_{};
  cv::Mat last_y_, last_mask_, last_debug_; LineResult last_result_; double last_processing_us_{0.0};
  rclcpp::Publisher<drone_msgs::msg::LinePixelObservation>::SharedPtr publisher_;
#ifdef DRONE_LINE_VISION_HAS_HBM
  void imageCallback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr message);
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_subscription_;
#endif
};
}

#pragma once

#include "drone_line_vision/line_vision_config.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace drone_line_vision
{
struct LineResult {
  bool valid{false}; double center_u{0.0}; double center_v{0.0}; double angle_rad{0.0};
  bool curve_valid{false}; double heading_error_rad{0.0}; double curvature_px_inv{0.0};
  int curve_direction{0};
  double confidence{0.0}; int candidate_pixels{0}; cv::Rect roi;
  cv::Mat mask; std::vector<cv::Point> component;
};

LineResult detectLine(const cv::Mat & y_plane, const LineVisionConfig & config);
}

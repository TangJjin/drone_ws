#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace drone_perception
{

struct TargetGeometryConfig
{
  int canny_low_threshold = 50;
  int canny_high_threshold = 150;
  int hough_threshold = 35;
  int hough_min_line_length = 45;
  int hough_max_line_gap = 12;
  double line_angle_tolerance_deg = 20.0;
  double min_cross_angle_deg = 60.0;
  int min_ring_radius_px = 20;
  int max_ring_radius_px = 220;
  int min_ring_separation_px = 15;
  double min_ring_coverage_ratio = 0.30;
};

struct TargetGeometryResult
{
  bool valid = false;
  bool outer_ring_valid = false;
  bool inner_ring_valid = false;
  cv::Point2f center;
  cv::Vec4i horizontal_line;
  cv::Vec4i vertical_line;
  double score = 0.0;
  double outer_ring_radius_px = 0.0;
  double inner_ring_radius_px = 0.0;
  std::vector<cv::Vec4i> lines;
};

TargetGeometryResult detectCrossTarget(
  const cv::Mat &bgr_image, const TargetGeometryConfig &config);

}  // namespace drone_perception

#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace drone_perception
{

struct TargetGeometryConfig
{
  int gaussian_blur_kernel = 5;
  double clahe_clip_limit = 2.0;
  int clahe_tile_grid_size = 8;
  int adaptive_threshold_block_size = 61;
  double adaptive_threshold_c = 5.0;
  int min_ring_radius_px = 20;
  int max_ring_radius_px = 220;
  double min_circularity = 0.70;
  double min_axis_ratio = 0.70;
  double min_inner_ring_score = 0.55;
  double min_cross_score = 0.58;
  double inner_ring_ratio_min = 0.48;
  double inner_ring_ratio_max = 0.74;
};

struct TargetGeometryResult
{
  bool valid = false;
  bool outer_ring_valid = false;
  bool inner_ring_valid = false;
  bool has_cross = false;
  std::string marker_type = "none";
  cv::Point2f center;
  cv::Size2f outer_axes;
  double ellipse_angle_deg = 0.0;
  double inner_outer_ratio = 0.0;
  double ring_score = 0.0;
  double cross_score = 0.0;
  double circularity = 0.0;
  double score = 0.0;
  cv::Mat dark_mask;
  std::vector<std::vector<cv::Point>> contours;
};

TargetGeometryResult detectCrossTarget(
  const cv::Mat &bgr_image, const TargetGeometryConfig &config);

}  // namespace drone_perception

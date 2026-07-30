#pragma once

#include <cstddef>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace drone_perception
{

struct DepthWindowSample
{
  bool valid = false;
  double depth_m = 0.0;
  std::size_t valid_count = 0U;
  std::size_t total_count = 0U;
  cv::Rect roi;
};

DepthWindowSample sampleDepthWindow(
  const cv::Mat &depth_image,
  int center_x,
  int center_y,
  int window_size,
  double min_depth_m,
  double max_depth_m);

}  // namespace drone_perception

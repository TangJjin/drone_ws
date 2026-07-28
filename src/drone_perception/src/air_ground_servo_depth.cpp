#include "drone_perception/air_ground_servo_depth.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace drone_perception
{

DepthWindowSample sampleDepthWindow(
  const cv::Mat &depth_image,
  int center_x,
  int center_y,
  int window_size,
  double min_depth_m,
  double max_depth_m)
{
  DepthWindowSample result;
  if (depth_image.empty() || window_size <= 0 || min_depth_m < 0.0 ||
    max_depth_m <= min_depth_m || center_x < 0 || center_y < 0 ||
    center_x >= depth_image.cols || center_y >= depth_image.rows)
  {
    return result;
  }

  if (depth_image.type() != CV_16UC1 && depth_image.type() != CV_32FC1) {
    return result;
  }

  const int left = std::max(0, center_x - window_size / 2);
  const int top = std::max(0, center_y - window_size / 2);
  const int right = std::min(depth_image.cols, center_x - window_size / 2 + window_size);
  const int bottom = std::min(depth_image.rows, center_y - window_size / 2 + window_size);
  if (left >= right || top >= bottom) {
    return result;
  }

  result.roi = cv::Rect(left, top, right - left, bottom - top);
  result.total_count = static_cast<std::size_t>(result.roi.area());

  std::vector<double> valid_depths;
  valid_depths.reserve(result.total_count);
  for (int y = top; y < bottom; ++y) {
    for (int x = left; x < right; ++x) {
      double depth_m = 0.0;
      if (depth_image.type() == CV_16UC1) {
        const std::uint16_t raw_depth = depth_image.at<std::uint16_t>(y, x);
        depth_m = static_cast<double>(raw_depth) * 0.001;
      } else {
        depth_m = static_cast<double>(depth_image.at<float>(y, x));
      }

      if (std::isfinite(depth_m) && depth_m >= min_depth_m && depth_m <= max_depth_m) {
        valid_depths.push_back(depth_m);
      }
    }
  }

  result.valid_count = valid_depths.size();
  if (valid_depths.empty()) {
    return result;
  }

  std::sort(valid_depths.begin(), valid_depths.end());
  const std::size_t middle = valid_depths.size() / 2U;
  result.depth_m = valid_depths.size() % 2U == 0U ?
    (valid_depths[middle - 1U] + valid_depths[middle]) * 0.5 : valid_depths[middle];
  result.valid = true;
  return result;
}

CameraPointSample projectPixelToCamera(
  int pixel_x,
  int pixel_y,
  double depth_m,
  int image_width,
  int image_height,
  const sensor_msgs::msg::CameraInfo &camera_info)
{
  CameraPointSample result;
  if (image_width <= 0 || image_height <= 0 || pixel_x < 0 || pixel_y < 0 ||
    pixel_x >= image_width || pixel_y >= image_height ||
    camera_info.width != static_cast<uint32_t>(image_width) ||
    camera_info.height != static_cast<uint32_t>(image_height) ||
    !std::isfinite(depth_m) || depth_m <= 0.0)
  {
    return result;
  }

  const double fx = camera_info.k[0];
  const double cx = camera_info.k[2];
  const double fy = camera_info.k[4];
  const double cy = camera_info.k[5];
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
    !std::isfinite(cy) || fx <= 0.0 || fy <= 0.0)
  {
    return result;
  }

  result.point.x = (static_cast<double>(pixel_x) - cx) * depth_m / fx;
  result.point.y = (static_cast<double>(pixel_y) - cy) * depth_m / fy;
  result.point.z = depth_m;
  result.valid = std::isfinite(result.point.x) && std::isfinite(result.point.y) &&
    std::isfinite(result.point.z);
  if (!result.valid) {
    result.point = cv::Point3d();
  }
  return result;
}

}  // namespace drone_perception

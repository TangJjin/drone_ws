#include "drone_line_vision/line_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace drone_line_vision
{
LineResult detectLine(const cv::Mat & y_plane, const LineVisionConfig & config)
{
  LineResult result;
  if (y_plane.empty() || y_plane.type() != CV_8UC1) {return result;}
  const auto & roi_cfg = config.roi;
  cv::Rect full(0, 0, y_plane.cols, y_plane.rows);
  cv::Rect roi = full;
  if (roi_cfg.enabled) {
    roi = cv::Rect(roi_cfg.x, roi_cfg.y, roi_cfg.width, roi_cfg.height) & full;
    if (roi.width <= 0 || roi.height <= 0) {return result;}
  }
  result.roi = roi;
  cv::Mat gray = y_plane(roi);
  cv::Mat mask;
  if (config.threshold.mode == "otsu") {
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  } else if (config.threshold.mode == "adaptive") {
    cv::adaptiveThreshold(gray, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
      cv::THRESH_BINARY_INV, 11, 4);
  } else {
    int type = config.threshold.invert ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
    cv::threshold(gray, mask, config.threshold.gray_threshold, 255, type);
  }
  if (config.threshold.invert && config.threshold.mode != "fixed") {cv::bitwise_not(mask, mask);}
  if (config.morphology.enabled) {
    int k = std::max(1, config.morphology.kernel_size);
    if ((k % 2) == 0) {++k;}
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1),
      std::max(0, config.morphology.open_iterations));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1),
      std::max(0, config.morphology.close_iterations));
  }
  result.candidate_pixels = cv::countNonZero(mask);
  result.mask = cv::Mat::zeros(y_plane.size(), CV_8UC1);
  mask.copyTo(result.mask(roi));
  if (result.candidate_pixels < config.threshold.min_candidate_pixels ||
    result.candidate_pixels > config.threshold.max_candidate_pixels) {return result;}

  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
  int best = -1; int best_area = 0;
  for (int i = 1; i < count; ++i) {
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area >= config.line_fit.min_component_area && area > best_area) {best = i; best_area = area;}
  }
  if (best < 0) {return result;}
  for (int y = 0; y < labels.rows; ++y) {
    for (int x = 0; x < labels.cols; ++x) {
      if (labels.at<int>(y, x) == best) {result.component.emplace_back(x + roi.x, y + roi.y);}
    }
  }
  if (result.component.size() < 2U) {return result;}
  cv::Vec4f line;
  cv::fitLine(result.component, line, cv::DIST_L2, 0, 0.01, 0.01);
  const cv::Point2f origin(line[2], line[3]);
  const cv::Point2f direction(line[0], line[1]);
  double residual_sum = 0.0;
  double min_proj = 0.0, max_proj = 0.0;
  for (size_t i = 0; i < result.component.size(); ++i) {
    const cv::Point2f point(result.component[i]);
    const cv::Point2f delta = point - origin;
    const double proj = delta.x * direction.x + delta.y * direction.y;
    if (i == 0) {min_proj = max_proj = proj;} else {min_proj = std::min(min_proj, proj); max_proj = std::max(max_proj, proj);}
    const double distance = std::abs(delta.x * direction.y - delta.y * direction.x);
    residual_sum += distance * distance;
  }
  const double residual = std::sqrt(residual_sum / static_cast<double>(result.component.size()));
  const double length = max_proj - min_proj;
  if (residual > config.line_fit.max_fit_residual_px || length < config.line_fit.min_line_length_px) {return result;}
  const cv::Moments moments = cv::moments(result.component, false);
  result.center_u = moments.m00 > 0.0 ? moments.m10 / moments.m00 : origin.x;
  result.center_v = moments.m00 > 0.0 ? moments.m01 / moments.m00 : origin.y;
  result.angle_rad = std::atan2(static_cast<double>(direction.y), static_cast<double>(direction.x));
  result.confidence = std::clamp((static_cast<double>(best_area) / 10000.0) *
    (1.0 - residual / std::max(1.0, config.line_fit.max_fit_residual_px)), 0.0, 1.0);
  result.valid = true;
  return result;
}
}

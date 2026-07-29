#include "drone_line_vision/line_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace drone_line_vision
{
namespace
{
bool fitQuadratic(const std::vector<cv::Point> & points, double & a, double & b, double & c,
  double & reference_v, double & residual)
{
  if (points.size() < 3U) {return false;}
  reference_v = 0.0;
  for (const auto & point : points) {reference_v += point.y;}
  reference_v /= static_cast<double>(points.size());

  cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
  cv::Mat values(static_cast<int>(points.size()), 1, CV_64F);
  for (size_t i = 0; i < points.size(); ++i) {
    const double t = static_cast<double>(points[i].y) - reference_v;
    design.at<double>(static_cast<int>(i), 0) = t * t;
    design.at<double>(static_cast<int>(i), 1) = t;
    design.at<double>(static_cast<int>(i), 2) = 1.0;
    values.at<double>(static_cast<int>(i), 0) = points[i].x;
  }
  cv::Mat coefficients;
  if (!cv::solve(design, values, coefficients, cv::DECOMP_SVD)) {return false;}
  a = coefficients.at<double>(0, 0);
  b = coefficients.at<double>(1, 0);
  c = coefficients.at<double>(2, 0);

  double squared_error = 0.0;
  for (const auto & point : points) {
    const double t = static_cast<double>(point.y) - reference_v;
    const double error = static_cast<double>(point.x) - (a * t * t + b * t + c);
    squared_error += error * error;
  }
  residual = std::sqrt(squared_error / static_cast<double>(points.size()));
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(residual);
}
}

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
  if (length < config.line_fit.min_line_length_px) {return result;}
  const cv::Moments moments = cv::moments(result.component, false);
  result.center_u = moments.m00 > 0.0 ? moments.m10 / moments.m00 : origin.x;
  result.center_v = moments.m00 > 0.0 ? moments.m01 / moments.m00 : origin.y;
  result.angle_rad = std::atan2(static_cast<double>(direction.y), static_cast<double>(direction.x));
  double fit_residual = residual;
  if (static_cast<int>(result.component.size()) >= config.curve.min_fit_points) {
    double a = 0.0, b = 0.0, c = 0.0, reference_v = 0.0, quadratic_residual = 0.0;
    if (fitQuadratic(result.component, a, b, c, reference_v, quadratic_residual) &&
      quadratic_residual <= config.curve.max_fit_residual_px) {
      const double row = static_cast<double>(roi.y) +
        config.curve.reference_row_ratio * static_cast<double>(std::max(0, roi.height - 1));
      const double t = row - reference_v;
      const double slope = 2.0 * a * t + b;
      const double signed_curvature = 2.0 * a / std::pow(1.0 + slope * slope, 1.5);
      result.heading_error_rad = -std::atan(slope);
      result.curvature_px_inv = std::abs(signed_curvature);
      result.curve_direction = result.curvature_px_inv < config.curve.straight_curvature_threshold_px_inv ?
        0 : (signed_curvature > 0.0 ? config.curve.direction_sign : -config.curve.direction_sign);
      result.curve_valid = true;
      fit_residual = quadratic_residual;
    }
  }
  if (!result.curve_valid && residual > config.line_fit.max_fit_residual_px) {return result;}
  result.confidence = std::clamp((static_cast<double>(best_area) / 10000.0) *
    (1.0 - fit_residual / std::max(1.0, config.curve.max_fit_residual_px)), 0.0, 1.0);
  result.valid = true;
  return result;
}
}

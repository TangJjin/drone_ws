#include "drone_line_vision/line_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

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

int findTargetLabel(const cv::Mat & labels, const cv::Mat & stats, const LineVisionConfig & config)
{
  const int count = stats.rows;
  const int center_x = labels.cols / 2;
  const int first_row = static_cast<int>(config.centerline.bottom_search_ratio * labels.rows);
  std::vector<int> lower_hits(static_cast<size_t>(count), 0);
  std::vector<int> center_distance(static_cast<size_t>(count), labels.cols);
  for (int y = std::clamp(first_row, 0, labels.rows - 1); y < labels.rows; ++y) {
    for (int x = 0; x < labels.cols; ++x) {
      const int label = labels.at<int>(y, x);
      if (label > 0) {
        ++lower_hits[static_cast<size_t>(label)];
        center_distance[static_cast<size_t>(label)] = std::min(
          center_distance[static_cast<size_t>(label)], std::abs(x - center_x));
      }
    }
  }

  int best = -1;
  double best_score = -std::numeric_limits<double>::infinity();
  for (int label = 1; label < count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < config.line_fit.min_component_area || lower_hits[static_cast<size_t>(label)] == 0) {
      continue;
    }
    const double score = 1000000.0 - 1000.0 * center_distance[static_cast<size_t>(label)] +
      10.0 * lower_hits[static_cast<size_t>(label)] + 0.01 * std::min(area, 200000);
    if (score > best_score) {
      best_score = score;
      best = label;
    }
  }
  return best;
}

std::vector<cv::Point> extractCenterline(const cv::Mat & labels, int target_label,
  const LineVisionConfig & config, const cv::Rect & roi)
{
  std::vector<cv::Point> centers;
  double expected_center = labels.cols / 2.0;
  int gap_rows = 0;
  for (int y = labels.rows - 1; y >= 0; y -= config.centerline.row_step_px) {
    int best_left = -1;
    int best_right = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int x = 0; x < labels.cols;) {
      if (labels.at<int>(y, x) != target_label) {
        ++x;
        continue;
      }
      const int left = x;
      while (x < labels.cols && labels.at<int>(y, x) == target_label) {++x;}
      const int right = x - 1;
      const int width = right - left + 1;
      if (width < config.centerline.min_band_width_px || width > config.centerline.max_band_width_px) {
        continue;
      }
      const double center = 0.5 * static_cast<double>(left + right);
      const double distance = std::abs(center - expected_center);
      if ((!centers.empty() && distance > config.centerline.max_center_jump_px) || distance >= best_distance) {
        continue;
      }
      {
        best_left = left;
        best_right = right;
        best_distance = distance;
      }
    }
    if (best_left >= 0) {
      const int center_x = (best_left + best_right) / 2;
      centers.emplace_back(center_x + roi.x, y + roi.y);
      expected_center = center_x;
      gap_rows = 0;
    } else if (!centers.empty()) {
      gap_rows += config.centerline.row_step_px;
      if (gap_rows > config.centerline.max_gap_rows) {break;}
    }
  }
  return centers;
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
      cv::THRESH_BINARY_INV, config.threshold.adaptive_block_size, config.threshold.adaptive_c);
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
  if (result.candidate_pixels < config.threshold.min_candidate_pixels) {return result;}

  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
  if (count <= 1) {return result;}
  const int best = findTargetLabel(labels, stats, config);
  if (best < 0) {return result;}
  result.selected_component_pixels = stats.at<int>(best, cv::CC_STAT_AREA);
  result.component = extractCenterline(labels, best, config, roi);
  if (static_cast<int>(result.component.size()) < config.centerline.min_valid_rows) {return result;}
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
      result.center_u = a * t * t + b * t + c;
      result.center_v = row;
      result.heading_error_rad = -std::atan(slope);
      result.curvature_px_inv = std::abs(signed_curvature);
      result.curve_direction = result.curvature_px_inv < config.curve.straight_curvature_threshold_px_inv ?
        0 : (signed_curvature > 0.0 ? config.curve.direction_sign : -config.curve.direction_sign);
      result.curve_valid = true;
      fit_residual = quadratic_residual;
    }
  }
  if (!result.curve_valid && residual > config.line_fit.max_fit_residual_px) {return result;}
  const double expected_rows = std::max(1.0, static_cast<double>(roi.height) /
    static_cast<double>(config.centerline.row_step_px));
  const double coverage = std::min(1.0, static_cast<double>(result.component.size()) / expected_rows);
  result.confidence = std::clamp(coverage *
    (1.0 - fit_residual / std::max(1.0, config.curve.max_fit_residual_px)), 0.0, 1.0);
  result.valid = true;
  return result;
}
}

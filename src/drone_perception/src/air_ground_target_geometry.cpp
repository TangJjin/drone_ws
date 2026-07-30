#include "drone_perception/air_ground_target_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace drone_perception
{
namespace
{

double segmentLength(const cv::Vec4i &line)
{
  return std::hypot(
    static_cast<double>(line[2] - line[0]), static_cast<double>(line[3] - line[1]));
}

bool intersectLines(const cv::Vec4i &first, const cv::Vec4i &second, cv::Point2f *intersection)
{
  const double x1 = first[0];
  const double y1 = first[1];
  const double x2 = first[2];
  const double y2 = first[3];
  const double x3 = second[0];
  const double y3 = second[1];
  const double x4 = second[2];
  const double y4 = second[3];
  const double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (std::abs(denominator) < 1e-6) {
    return false;
  }
  const double determinant_one = x1 * y2 - y1 * x2;
  const double determinant_two = x3 * y4 - y3 * x4;
  intersection->x = static_cast<float>(
    (determinant_one * (x3 - x4) - (x1 - x2) * determinant_two) / denominator);
  intersection->y = static_cast<float>(
    (determinant_one * (y3 - y4) - (y1 - y2) * determinant_two) / denominator);
  return std::isfinite(intersection->x) && std::isfinite(intersection->y);
}

}  // namespace

TargetGeometryResult detectCrossTarget(
  const cv::Mat &bgr_image, const TargetGeometryConfig &config)
{
  TargetGeometryResult result;
  if (bgr_image.empty() || bgr_image.channels() != 3 || config.canny_low_threshold <= 0 ||
    config.canny_high_threshold <= config.canny_low_threshold || config.hough_threshold <= 0 ||
    config.hough_min_line_length <= 0 || config.hough_max_line_gap < 0 ||
    config.line_angle_tolerance_deg <= 0.0 || config.line_angle_tolerance_deg >= 45.0 ||
    config.min_cross_angle_deg <= 0.0 || config.min_cross_angle_deg > 90.0 ||
    config.min_ring_radius_px <= 0 || config.max_ring_radius_px <= config.min_ring_radius_px ||
    config.min_ring_separation_px <= 0 ||
    config.min_ring_coverage_ratio <= 0.0 || config.min_ring_coverage_ratio > 1.0)
  {
    return result;
  }

  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat edges;
  cv::Canny(gray, edges, config.canny_low_threshold, config.canny_high_threshold);
  cv::HoughLinesP(edges, result.lines, 1.0, CV_PI / 180.0, config.hough_threshold,
    config.hough_min_line_length, config.hough_max_line_gap);

  const double angle_tolerance = config.line_angle_tolerance_deg * CV_PI / 180.0;
  double best_score = -std::numeric_limits<double>::infinity();
  for (const cv::Vec4i &horizontal : result.lines) {
    const double horizontal_length = segmentLength(horizontal);
    if (horizontal_length <= 1e-6) {
      continue;
    }
    const double horizontal_angle = std::atan2(
      static_cast<double>(horizontal[3] - horizontal[1]),
      static_cast<double>(horizontal[2] - horizontal[0]));
    const double horizontal_error = std::min(
      std::abs(horizontal_angle), std::abs(std::abs(horizontal_angle) - CV_PI));
    if (horizontal_error > angle_tolerance) {
      continue;
    }
    for (const cv::Vec4i &vertical : result.lines) {
      const double vertical_length = segmentLength(vertical);
      if (vertical_length <= 1e-6) {
        continue;
      }
      const double vertical_angle = std::atan2(
        static_cast<double>(vertical[3] - vertical[1]),
        static_cast<double>(vertical[2] - vertical[0]));
      const double vertical_error = std::abs(std::abs(vertical_angle) - CV_PI * 0.5);
      if (vertical_error > angle_tolerance) {
        continue;
      }
      const double cross_angle_deg = std::abs(horizontal_angle - vertical_angle) * 180.0 / CV_PI;
      const double folded_angle = std::min(cross_angle_deg, 180.0 - cross_angle_deg);
      if (folded_angle < config.min_cross_angle_deg) {
        continue;
      }
      cv::Point2f intersection;
      if (!intersectLines(horizontal, vertical, &intersection) ||
        intersection.x < 0.0F || intersection.x >= bgr_image.cols ||
        intersection.y < 0.0F || intersection.y >= bgr_image.rows)
      {
        continue;
      }
      const double score = horizontal_length + vertical_length + folded_angle;
      if (score > best_score) {
        best_score = score;
        result.horizontal_line = horizontal;
        result.vertical_line = vertical;
        result.center = intersection;
      }
    }
  }
  if (!std::isfinite(best_score)) {
    return result;
  }
  constexpr int kAngleBins = 72;
  const int maximum_radius = std::min(
    config.max_ring_radius_px, std::min(bgr_image.cols, bgr_image.rows) / 2);
  std::vector<std::vector<bool>> radial_support(
    static_cast<std::size_t>(maximum_radius + 1), std::vector<bool>(kAngleBins, false));
  for (int y = 0; y < edges.rows; ++y) {
    const auto *edge_row = edges.ptr<std::uint8_t>(y);
    for (int x = 0; x < edges.cols; ++x) {
      if (edge_row[x] == 0U) {
        continue;
      }
      const double dx = x - result.center.x;
      const double dy = y - result.center.y;
      const int radius = static_cast<int>(std::lround(std::hypot(dx, dy)));
      if (radius < config.min_ring_radius_px - 2 || radius > maximum_radius + 2) {
        continue;
      }
      double angle = std::atan2(dy, dx);
      if (angle < 0.0) {
        angle += 2.0 * CV_PI;
      }
      const int bin = std::min(kAngleBins - 1, static_cast<int>(angle * kAngleBins / (2.0 * CV_PI)));
      for (int offset = -2; offset <= 2; ++offset) {
        const int supported_radius = radius + offset;
        if (supported_radius >= config.min_ring_radius_px && supported_radius <= maximum_radius) {
          radial_support[static_cast<std::size_t>(supported_radius)][static_cast<std::size_t>(bin)] = true;
        }
      }
    }
  }
  std::vector<double> coverage(static_cast<std::size_t>(maximum_radius + 1), 0.0);
  for (int radius = config.min_ring_radius_px; radius <= maximum_radius; ++radius) {
    coverage[static_cast<std::size_t>(radius)] = static_cast<double>(std::count(
      radial_support[static_cast<std::size_t>(radius)].begin(),
      radial_support[static_cast<std::size_t>(radius)].end(), true)) / kAngleBins;
  }
  double best_ring_score = -1.0;
  for (int outer_radius = config.min_ring_radius_px; outer_radius <= maximum_radius; ++outer_radius) {
    if (coverage[static_cast<std::size_t>(outer_radius)] < config.min_ring_coverage_ratio) {
      continue;
    }
    for (int inner_radius = config.min_ring_radius_px;
      inner_radius + config.min_ring_separation_px < outer_radius; ++inner_radius)
    {
      if (coverage[static_cast<std::size_t>(inner_radius)] < config.min_ring_coverage_ratio) {
        continue;
      }
      const double ring_score = coverage[static_cast<std::size_t>(outer_radius)] +
        coverage[static_cast<std::size_t>(inner_radius)];
      if (ring_score > best_ring_score) {
        best_ring_score = ring_score;
        result.outer_ring_radius_px = outer_radius;
        result.inner_ring_radius_px = inner_radius;
      }
    }
  }
  if (best_ring_score >= 0.0) {
    result.outer_ring_valid = true;
    result.inner_ring_valid = true;
    result.valid = true;
    result.score = best_score + best_ring_score;
  }
  return result;
}

}  // namespace drone_perception

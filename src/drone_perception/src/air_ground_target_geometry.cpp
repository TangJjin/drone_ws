#include "drone_perception/air_ground_target_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace drone_perception
{
namespace
{

constexpr char kCarPlatform[] = "car_platform";
constexpr char kHome[] = "home";

bool isValidConfig(const TargetGeometryConfig &config)
{
  return config.gaussian_blur_kernel >= 3 && config.gaussian_blur_kernel % 2 == 1 &&
         config.clahe_clip_limit > 0.0 && config.clahe_tile_grid_size > 0 &&
         config.adaptive_threshold_block_size >= 3 && config.adaptive_threshold_block_size % 2 == 1 &&
         config.min_ring_radius_px > 0 && config.max_ring_radius_px > config.min_ring_radius_px &&
         config.min_circularity > 0.0 && config.min_circularity <= 1.0 &&
         config.min_axis_ratio > 0.0 && config.min_axis_ratio <= 1.0 &&
         config.min_inner_ring_score > 0.0 && config.min_inner_ring_score <= 1.0 &&
         config.min_cross_score > 0.0 && config.min_cross_score <= 1.0 &&
         config.inner_ring_ratio_min > 0.0 &&
         config.inner_ring_ratio_max > config.inner_ring_ratio_min && config.inner_ring_ratio_max < 1.0;
}

void ellipsePoints(
  const cv::Point2f &center, const cv::Size2f &semi_axes, double angle_deg, double ratio,
  const std::vector<double> &angles, std::vector<cv::Point> *points)
{
  points->clear();
  points->reserve(angles.size());
  const double angle = angle_deg * CV_PI / 180.0;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  for (const double theta : angles) {
    const double local_x = semi_axes.width * ratio * std::cos(theta);
    const double local_y = semi_axes.height * ratio * std::sin(theta);
    points->emplace_back(
      static_cast<int>(std::lround(center.x + local_x * cosine - local_y * sine)),
      static_cast<int>(std::lround(center.y + local_x * sine + local_y * cosine)));
  }
}

double innerRingScore(
  const cv::Mat &mask, const cv::Point2f &center, const cv::Size2f &semi_axes,
  double angle_deg, const TargetGeometryConfig &config, double *best_ratio)
{
  std::vector<double> angles;
  angles.reserve(360U);
  for (int degree = 0; degree < 360; ++degree) {
    angles.push_back(static_cast<double>(degree) * CV_PI / 180.0);
  }

  double best = 0.0;
  *best_ratio = 0.0;
  std::vector<cv::Point> points;
  for (double ratio = config.inner_ring_ratio_min; ratio <= config.inner_ring_ratio_max + 1e-6;
    ratio += 0.01)
  {
    std::vector<bool> hits(angles.size(), false);
    for (double offset = -0.035; offset <= 0.0351; offset += 0.0175) {
      ellipsePoints(center, semi_axes, angle_deg, ratio + offset, angles, &points);
      for (std::size_t index = 0; index < points.size(); ++index) {
        const cv::Point &point = points[index];
        if (point.x >= 0 && point.x < mask.cols && point.y >= 0 && point.y < mask.rows &&
          mask.at<std::uint8_t>(point) != 0U)
        {
          hits[index] = true;
        }
      }
    }
    const double score = static_cast<double>(std::count(hits.begin(), hits.end(), true)) /
      static_cast<double>(hits.size());
    if (score > best) {
      best = score;
      *best_ratio = ratio;
    }
  }
  return best;
}

double diameterLineScore(
  const cv::Mat &mask, const cv::Point2f &center, double radius, double angle_deg)
{
  const double angle = angle_deg * CV_PI / 180.0;
  const cv::Point2f direction(std::cos(angle), std::sin(angle));
  const cv::Point2f normal(-direction.y, direction.x);
  const int half_width = std::max(1, static_cast<int>(std::lround(radius * 0.045)));
  int hit_count = 0;
  constexpr int kSamplesPerSide = 80;
  constexpr int kTotalSamples = kSamplesPerSide * 2;
  for (int index = 0; index < kTotalSamples; ++index) {
    const double fraction = index < kSamplesPerSide ?
      -0.86 + static_cast<double>(index) * 0.71 / (kSamplesPerSide - 1) :
      0.15 + static_cast<double>(index - kSamplesPerSide) * 0.71 / (kSamplesPerSide - 1);
    bool hit = false;
    for (int offset = -half_width; offset <= half_width; ++offset) {
      const int x = static_cast<int>(std::lround(center.x + fraction * radius * direction.x + offset * normal.x));
      const int y = static_cast<int>(std::lround(center.y + fraction * radius * direction.y + offset * normal.y));
      if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows && mask.at<std::uint8_t>(y, x) != 0U) {
        hit = true;
        break;
      }
    }
    hit_count += hit ? 1 : 0;
  }
  return static_cast<double>(hit_count) / kTotalSamples;
}

double crossScore(const cv::Mat &mask, const cv::Point2f &center, const cv::Size2f &semi_axes)
{
  const double radius = std::min(semi_axes.width, semi_axes.height);
  double best = 0.0;
  for (int angle = 0; angle < 90; angle += 2) {
    best = std::max(best, std::min(
      diameterLineScore(mask, center, radius, static_cast<double>(angle)),
      diameterLineScore(mask, center, radius, static_cast<double>(angle + 90))));
  }
  return best;
}

}  // namespace

TargetGeometryResult detectCrossTarget(
  const cv::Mat &bgr_image, const TargetGeometryConfig &config)
{
  TargetGeometryResult result;
  if (bgr_image.empty() || bgr_image.channels() != 3 || !isValidConfig(config)) {
    return result;
  }

  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(config.gaussian_blur_kernel, config.gaussian_blur_kernel), 0.0);
  const cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
    config.clahe_clip_limit, cv::Size(config.clahe_tile_grid_size, config.clahe_tile_grid_size));
  cv::Mat enhanced;
  clahe->apply(blurred, enhanced);
  cv::Mat otsu;
  cv::threshold(enhanced, otsu, 0.0, 255.0, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  int adaptive_block = std::min(config.adaptive_threshold_block_size, std::min(gray.cols, gray.rows) | 1);
  adaptive_block = std::max(3, adaptive_block);
  cv::Mat adaptive;
  cv::adaptiveThreshold(enhanced, adaptive, 255.0, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
    cv::THRESH_BINARY_INV, adaptive_block, config.adaptive_threshold_c);
  cv::bitwise_and(otsu, adaptive, result.dark_mask);
  cv::findContours(result.dark_mask, result.contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  const double min_radius = std::max(7.0, std::min(gray.cols, gray.rows) * 0.018);
  const double max_radius = std::min(
    static_cast<double>(config.max_ring_radius_px), std::min(gray.cols, gray.rows) * 0.52);
  double best_confidence = -std::numeric_limits<double>::infinity();
  for (const std::vector<cv::Point> &contour : result.contours) {
    if (contour.size() < 5U) {
      continue;
    }
    const double area = cv::contourArea(contour);
    const double perimeter = cv::arcLength(contour, true);
    if (area <= 0.0 || perimeter <= 0.0) {
      continue;
    }
    const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    if (circularity < config.min_circularity) {
      continue;
    }
    const cv::RotatedRect ellipse = cv::fitEllipse(contour);
    const double major = std::max(ellipse.size.width, ellipse.size.height);
    const double minor = std::min(ellipse.size.width, ellipse.size.height);
    const double mean_radius = (major + minor) * 0.25;
    const double axis_ratio = minor / major;
    if (axis_ratio < config.min_axis_ratio || mean_radius < min_radius || mean_radius > max_radius) {
      continue;
    }
    const double ring_angle = ellipse.angle;
    const cv::Size2f semi_axes(ellipse.size.width * 0.5F, ellipse.size.height * 0.5F);
    double ratio = 0.0;
    const double ring_score = innerRingScore(
      result.dark_mask, ellipse.center, semi_axes, ring_angle, config, &ratio);
    if (ring_score < config.min_inner_ring_score) {
      continue;
    }
    const double candidate_cross_score = crossScore(result.dark_mask, ellipse.center, semi_axes);
    const bool has_cross = candidate_cross_score >= config.min_cross_score;
    const double geometry_score = std::min(1.0, 0.55 * ring_score + 0.25 * circularity + 0.20 * axis_ratio);
    const double confidence = 0.75 * geometry_score + 0.25 * (has_cross ? candidate_cross_score : 1.0 - candidate_cross_score);
    if (confidence <= best_confidence) {
      continue;
    }
    best_confidence = confidence;
    result.outer_ring_valid = true;
    result.inner_ring_valid = true;
    result.has_cross = has_cross;
    result.marker_type = has_cross ? kCarPlatform : kHome;
    result.center = ellipse.center;
    result.outer_axes = cv::Size2f(static_cast<float>(major), static_cast<float>(minor));
    result.ellipse_angle_deg = ellipse.size.width >= ellipse.size.height ? ellipse.angle :
      std::fmod(ellipse.angle + 90.0, 180.0);
    result.inner_outer_ratio = ratio;
    result.ring_score = ring_score;
    result.cross_score = candidate_cross_score;
    result.circularity = circularity;
    result.score = confidence;
  }
  result.valid = result.marker_type == kCarPlatform;
  return result;
}

}  // namespace drone_perception

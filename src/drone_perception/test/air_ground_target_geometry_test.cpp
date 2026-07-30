#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "drone_perception/air_ground_target_geometry.hpp"

namespace drone_perception
{
namespace
{

TEST(AirGroundTargetGeometryTest, DetectsDoubleRingAndCrossIntersection)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Point expected_center(370, 210);
  cv::ellipse(image, expected_center, cv::Size(110, 92), 0.0, 0.0, 360.0, cv::Scalar(0, 0, 0), 4);
  cv::ellipse(image, expected_center, cv::Size(66, 55), 0.0, 0.0, 360.0, cv::Scalar(0, 0, 0), 4);
  cv::line(image, cv::Point(240, 210), cv::Point(500, 210), cv::Scalar(0, 0, 0), 4);
  cv::line(image, cv::Point(370, 90), cv::Point(370, 330), cv::Scalar(0, 0, 0), 4);

  const TargetGeometryResult result = detectCrossTarget(image, TargetGeometryConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.center.x, expected_center.x, 4.0);
  EXPECT_NEAR(result.center.y, expected_center.y, 4.0);
}

TEST(AirGroundTargetGeometryTest, RejectsCrossWithoutDoubleRing)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::line(image, cv::Point(240, 210), cv::Point(500, 210), cv::Scalar(0, 0, 0), 4);
  cv::line(image, cv::Point(370, 90), cv::Point(370, 330), cv::Scalar(0, 0, 0), 4);

  EXPECT_FALSE(detectCrossTarget(image, TargetGeometryConfig{}).valid);
}

TEST(AirGroundTargetGeometryTest, RejectsParallelLines)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::line(image, cv::Point(120, 180), cv::Point(520, 180), cv::Scalar(0, 0, 0), 4);
  cv::line(image, cv::Point(120, 280), cv::Point(520, 280), cv::Scalar(0, 0, 0), 4);

  EXPECT_FALSE(detectCrossTarget(image, TargetGeometryConfig{}).valid);
}

TEST(AirGroundTargetGeometryTest, RejectsEmptyImage)
{
  EXPECT_FALSE(detectCrossTarget(cv::Mat(), TargetGeometryConfig{}).valid);
}

}  // namespace
}  // namespace drone_perception

#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>

#include "drone_perception/air_ground_servo_depth.hpp"

namespace drone_perception
{
namespace
{

TEST(AirGroundServoDepthTest, SamplesExactTenByTenUint16Window)
{
  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1000));
  depth.at<std::uint16_t>(235, 315) = 0U;
  depth.at<std::uint16_t>(244, 324) = 9000U;

  const DepthWindowSample result = sampleDepthWindow(depth, 320, 240, 10, 0.1, 5.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.depth_m, 1.0);
  EXPECT_EQ(result.valid_count, 98U);
  EXPECT_EQ(result.total_count, 100U);
  EXPECT_EQ(result.roi, cv::Rect(315, 235, 10, 10));
}

TEST(AirGroundServoDepthTest, AveragesMiddlePairForEvenFloatSamples)
{
  cv::Mat depth(2, 2, CV_32FC1);
  depth.at<float>(0, 0) = 1.0F;
  depth.at<float>(0, 1) = 2.0F;
  depth.at<float>(1, 0) = 3.0F;
  depth.at<float>(1, 1) = 4.0F;

  const DepthWindowSample result = sampleDepthWindow(depth, 1, 1, 2, 0.1, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.depth_m, 2.5);
  EXPECT_EQ(result.valid_count, 4U);
}

TEST(AirGroundServoDepthTest, RejectsInvalidAndOutOfRangeSamples)
{
  cv::Mat depth(10, 10, CV_32FC1, cv::Scalar(0.0F));
  depth.at<float>(5, 5) = std::numeric_limits<float>::quiet_NaN();
  depth.at<float>(5, 6) = 20.0F;

  const DepthWindowSample result = sampleDepthWindow(depth, 5, 5, 10, 0.1, 10.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.valid_count, 0U);
  EXPECT_EQ(result.total_count, 100U);
}

TEST(AirGroundServoDepthTest, ClipsWindowAtImageBoundary)
{
  cv::Mat depth(10, 10, CV_16UC1, cv::Scalar(1500));

  const DepthWindowSample result = sampleDepthWindow(depth, 0, 0, 10, 0.1, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.depth_m, 1.5);
  EXPECT_EQ(result.total_count, 25U);
  EXPECT_EQ(result.roi, cv::Rect(0, 0, 5, 5));
}

}  // namespace
}  // namespace drone_perception

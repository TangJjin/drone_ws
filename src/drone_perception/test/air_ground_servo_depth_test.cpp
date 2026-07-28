#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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

TEST(AirGroundServoDepthTest, ProjectsPrincipalPointAlongOpticalAxis)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.k[0] = 500.0;
  info.k[2] = 320.0;
  info.k[4] = 500.0;
  info.k[5] = 240.0;

  const CameraPointSample result = projectPixelToCamera(320, 240, 1.0, 640, 480, info);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.point.x, 0.0, 1e-9);
  EXPECT_NEAR(result.point.y, 0.0, 1e-9);
  EXPECT_NEAR(result.point.z, 1.0, 1e-9);
}

TEST(AirGroundServoDepthTest, ProjectsPixelSignsAndRejectsInvalidInputs)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.k[0] = 500.0;
  info.k[2] = 320.0;
  info.k[4] = 500.0;
  info.k[5] = 240.0;

  const CameraPointSample right_down = projectPixelToCamera(370, 290, 2.0, 640, 480, info);
  ASSERT_TRUE(right_down.valid);
  EXPECT_GT(right_down.point.x, 0.0);
  EXPECT_GT(right_down.point.y, 0.0);

  const CameraPointSample bad_depth = projectPixelToCamera(320, 240, 0.0, 640, 480, info);
  EXPECT_FALSE(bad_depth.valid);

  info.k[0] = 0.0;
  const CameraPointSample bad_intrinsics = projectPixelToCamera(320, 240, 1.0, 640, 480, info);
  EXPECT_FALSE(bad_intrinsics.valid);
}

}  // namespace
}  // namespace drone_perception

#include "drone_line_vision/frame_statistics.hpp"
#include <algorithm>
#include <vector>

namespace drone_line_vision
{
void FrameStatistics::addCapture(double fps)
{
  capture_fps_ = capture_fps_ <= 0.0 ? fps : 0.9 * capture_fps_ + 0.1 * fps;
}

void FrameStatistics::addObservation(double processing_us, double interval_s)
{
  if (interval_s > 0.0) {
    const double fps = 1.0 / interval_s;
    observation_fps_ = observation_fps_ <= 0.0 ? fps : 0.9 * observation_fps_ + 0.1 * fps;
  }
  timings_.push_back(processing_us);
  if (timings_.size() > 300) {timings_.pop_front();}
}

double FrameStatistics::p50Us() const
{
  if (timings_.empty()) {return 0.0;}
  std::vector<double> values(timings_.begin(), timings_.end());
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

double FrameStatistics::p95Us() const
{
  if (timings_.empty()) {return 0.0;}
  std::vector<double> values(timings_.begin(), timings_.end());
  std::sort(values.begin(), values.end());
  const auto index = static_cast<size_t>(0.95 * static_cast<double>(values.size() - 1));
  return values[index];
}
}

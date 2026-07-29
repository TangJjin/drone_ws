#pragma once

#include <cstdint>
#include <deque>

namespace drone_line_vision
{
class FrameStatistics {
public:
  void addCapture(double fps);
  void addObservation(double processing_us, double interval_s);
  double captureFps() const {return capture_fps_;}
  double observationFps() const {return observation_fps_;}
  double p50Us() const;
  double p95Us() const;
private:
  double capture_fps_{0.0}; double observation_fps_{0.0}; std::deque<double> timings_;
};
}

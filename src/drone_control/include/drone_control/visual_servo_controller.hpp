#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace offboard_run {

enum class BodyAxis {
  X = 0,
  Y = 1,
  Z = 2,
};

struct VisualServoConfig {
  std::string target_id;
  bool require_confirmed = true;

  BodyAxis image_x_axis = BodyAxis::Y;
  BodyAxis image_y_axis = BodyAxis::Z;
  double image_x_sign = -1.0;
  double image_y_sign = -1.0;

  double kp_x = 0.35;
  double ki_x = 0.0;
  double kd_x = 0.02;
  double kp_y = 0.35;
  double ki_y = 0.0;
  double kd_y = 0.02;
  double integral_limit = 0.5;
  double filter_alpha = 0.35;

  double enter_tolerance_x = 0.04;
  double enter_tolerance_y = 0.04;
  double exit_tolerance_x = 0.07;
  double exit_tolerance_y = 0.07;
  double settle_time_s = 0.6;

  double acquire_timeout_s = 5.0;
  double lost_timeout_s = 1.0;
  double overall_timeout_s = 20.0;
  double max_body_speed_mps = 0.20;
};

struct VisualServoObservation {
  uint32_t sequence = 0;
  std::string target_id;
  bool valid = false;
  bool confirmed = false;
  double error_x = 0.0;
  double error_y = 0.0;
  rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
};

enum class VisualServoState {
  IDLE = 0,
  WAIT_TARGET,
  TRACKING,
  TARGET_LOST,
  ALIGNED,
  SUCCEEDED,
  TIMED_OUT,
};

struct VisualServoOutput {
  VisualServoState state = VisualServoState::IDLE;
  Eigen::Vector3d body_delta = Eigen::Vector3d::Zero();
  bool target_visible = false;
  bool aligned = false;
  double filtered_error_x = 0.0;
  double filtered_error_y = 0.0;
  std::string detail;
};

class VisualServoController {
 public:
  void start(const VisualServoConfig &config, const rclcpp::Time &now) {
    config_ = config;
    start_time_ = now;
    last_update_time_ = now;
    last_valid_target_time_ = rclcpp::Time(0, 0, now.get_clock_type());
    aligned_since_ = rclcpp::Time(0, 0, now.get_clock_type());
    state_ = VisualServoState::WAIT_TARGET;
    has_locked_target_ = false;
    locked_target_id_.clear();
    filter_initialized_ = false;
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    last_error_x_ = 0.0;
    last_error_y_ = 0.0;
  }

  void reset() {
    state_ = VisualServoState::IDLE;
    has_locked_target_ = false;
    locked_target_id_.clear();
    filter_initialized_ = false;
    integral_x_ = 0.0;
    integral_y_ = 0.0;
  }

  VisualServoOutput update(
      const VisualServoObservation *observation,
      const rclcpp::Time &now) {
    VisualServoOutput output;

    if (state_ == VisualServoState::IDLE) {
      output.detail = "visual servo is idle";
      return output;
    }

    if ((now - start_time_).seconds() > config_.overall_timeout_s) {
      state_ = VisualServoState::TIMED_OUT;
      output.state = state_;
      output.detail = "overall timeout";
      return output;
    }

    const bool matches_requested_target = observation != nullptr &&
        (config_.target_id.empty() || observation->target_id == config_.target_id);
    const bool matches_locked_target = observation != nullptr &&
        (!has_locked_target_ || observation->target_id == locked_target_id_);
    const bool matching_target = matches_requested_target && matches_locked_target;
    const bool acceptable_target = matching_target && observation->valid &&
        (!config_.require_confirmed || observation->confirmed) &&
        observation->received_time >= start_time_;
    const bool fresh_target = acceptable_target &&
        (now - observation->received_time).seconds() <= config_.lost_timeout_s;

    if (!fresh_target) {
      aligned_since_ = rclcpp::Time(0, 0, now.get_clock_type());
      if (last_valid_target_time_.nanoseconds() == 0) {
        state_ = VisualServoState::WAIT_TARGET;
        if ((now - start_time_).seconds() > config_.acquire_timeout_s) {
          state_ = VisualServoState::TIMED_OUT;
          output.detail = "target acquisition timeout";
        } else {
          output.detail = "waiting for a valid target";
        }
      } else {
        state_ = VisualServoState::TARGET_LOST;
        if ((now - last_valid_target_time_).seconds() > config_.lost_timeout_s) {
          state_ = VisualServoState::TIMED_OUT;
          output.detail = "target lost timeout";
        } else {
          output.detail = "target temporarily lost; waiting for reacquisition";
        }
      }
      output.state = state_;
      output.filtered_error_x = filtered_error_x_;
      output.filtered_error_y = filtered_error_y_;
      return output;
    }

    if (!has_locked_target_) {
      locked_target_id_ = observation->target_id;
      has_locked_target_ = true;
    }

    last_valid_target_time_ = now;
    output.target_visible = true;

    const double alpha = std::clamp(config_.filter_alpha, 0.0, 1.0);
    if (!filter_initialized_) {
      filtered_error_x_ = observation->error_x;
      filtered_error_y_ = observation->error_y;
      filter_initialized_ = true;
    } else {
      filtered_error_x_ = alpha * observation->error_x +
          (1.0 - alpha) * filtered_error_x_;
      filtered_error_y_ = alpha * observation->error_y +
          (1.0 - alpha) * filtered_error_y_;
    }

    const bool was_aligned = aligned_since_.nanoseconds() != 0;
    const bool inside_enter =
        std::abs(filtered_error_x_) <= config_.enter_tolerance_x &&
        std::abs(filtered_error_y_) <= config_.enter_tolerance_y;
    const bool outside_exit =
        std::abs(filtered_error_x_) > config_.exit_tolerance_x ||
        std::abs(filtered_error_y_) > config_.exit_tolerance_y;

    if ((!was_aligned && inside_enter) || (was_aligned && !outside_exit)) {
      if (!was_aligned) {
        aligned_since_ = now;
        resetPidState();
      }
      state_ = VisualServoState::ALIGNED;
      output.aligned = true;
      output.detail = "inside alignment tolerance";
      if ((now - aligned_since_).seconds() >= config_.settle_time_s) {
        state_ = VisualServoState::SUCCEEDED;
        output.detail = "alignment settled";
      }
    } else {
      aligned_since_ = rclcpp::Time(0, 0, now.get_clock_type());
      state_ = VisualServoState::TRACKING;
      output.detail = "tracking target";
      output.body_delta = calculateBodyDelta(now);
    }

    output.state = state_;
    output.filtered_error_x = filtered_error_x_;
    output.filtered_error_y = filtered_error_y_;
    return output;
  }

  VisualServoState state() const { return state_; }
  const std::string &lockedTargetId() const { return locked_target_id_; }

  static const char *stateName(VisualServoState state) {
    switch (state) {
      case VisualServoState::IDLE: return "idle";
      case VisualServoState::WAIT_TARGET: return "wait_target";
      case VisualServoState::TRACKING: return "tracking";
      case VisualServoState::TARGET_LOST: return "target_lost";
      case VisualServoState::ALIGNED: return "aligned";
      case VisualServoState::SUCCEEDED: return "succeeded";
      case VisualServoState::TIMED_OUT: return "timed_out";
    }
    return "unknown";
  }

 private:
  void resetPidState() {
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    last_error_x_ = filtered_error_x_;
    last_error_y_ = filtered_error_y_;
  }

  Eigen::Vector3d calculateBodyDelta(const rclcpp::Time &now) {
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    dt = std::clamp(dt, 0.001, 0.1);

    integral_x_ = std::clamp(
        integral_x_ + filtered_error_x_ * dt,
        -config_.integral_limit, config_.integral_limit);
    integral_y_ = std::clamp(
        integral_y_ + filtered_error_y_ * dt,
        -config_.integral_limit, config_.integral_limit);

    const double derivative_x = (filtered_error_x_ - last_error_x_) / dt;
    const double derivative_y = (filtered_error_y_ - last_error_y_) / dt;
    last_error_x_ = filtered_error_x_;
    last_error_y_ = filtered_error_y_;

    double velocity_x = config_.kp_x * filtered_error_x_ +
        config_.ki_x * integral_x_ + config_.kd_x * derivative_x;
    double velocity_y = config_.kp_y * filtered_error_y_ +
        config_.ki_y * integral_y_ + config_.kd_y * derivative_y;
    velocity_x = std::clamp(
        velocity_x, -config_.max_body_speed_mps, config_.max_body_speed_mps);
    velocity_y = std::clamp(
        velocity_y, -config_.max_body_speed_mps, config_.max_body_speed_mps);

    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta[static_cast<int>(config_.image_x_axis)] +=
        config_.image_x_sign * velocity_x * dt;
    delta[static_cast<int>(config_.image_y_axis)] +=
        config_.image_y_sign * velocity_y * dt;
    return delta;
  }

  VisualServoConfig config_;
  VisualServoState state_ = VisualServoState::IDLE;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time aligned_since_{0, 0, RCL_ROS_TIME};
  bool filter_initialized_ = false;
  bool has_locked_target_ = false;
  std::string locked_target_id_;
  double filtered_error_x_ = 0.0;
  double filtered_error_y_ = 0.0;
  double integral_x_ = 0.0;
  double integral_y_ = 0.0;
  double last_error_x_ = 0.0;
  double last_error_y_ = 0.0;
};

}  // namespace offboard_run

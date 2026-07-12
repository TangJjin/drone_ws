#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

class PoseYawComparerNode : public rclcpp::Node {
 public:
  PoseYawComparerNode() : Node("pose_yaw_comparer") {
    rclcpp::QoS mavros_pose_qos(rclcpp::KeepLast(10));
    mavros_pose_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    mavros_pose_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    vision_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", mavros_pose_qos,
        std::bind(&PoseYawComparerNode::visionCallback, this,
                  std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/odometry/out", mavros_pose_qos,
        std::bind(&PoseYawComparerNode::odomCallback, this,
                  std::placeholders::_1));

    local_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", mavros_pose_qos,
        std::bind(&PoseYawComparerNode::localCallback, this,
                  std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", mavros_pose_qos,
        std::bind(&PoseYawComparerNode::stateCallback, this,
                  std::placeholders::_1));

    delta_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
        "/pose_yaw_compare/delta", 10);

    max_dx_ = 1.0;
    max_dy_ = 1.0;
    max_dyaw_deg_ = 30.0;
    startup_grace_sec_ = 15.0;
    consecutive_limit_ = 10;
    consecutive_exceed_count_ = 0;
    start_time_ = now();
    state_received_ = false;
    armed_ = false;
    last_state_wait_log_time_ =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&PoseYawComparerNode::printResult, this));

    RCLCPP_INFO(get_logger(), "pose yaw comparer started");
    RCLCPP_INFO(get_logger(),
                "subscribed: /mavros/vision_pose/pose (PoseStamped)");
    RCLCPP_INFO(get_logger(),
                "subscribed: /mavros/odometry/out (Odometry)");
    RCLCPP_INFO(get_logger(), "subscribed: /mavros/local_position/pose");
    RCLCPP_INFO(get_logger(), "subscribed: /mavros/state");
    RCLCPP_INFO(get_logger(), "subscription QoS: BEST_EFFORT / VOLATILE");
    RCLCPP_INFO(get_logger(), "publishing delta to: /pose_yaw_compare/delta");
    RCLCPP_INFO(get_logger(), "delta meaning: x=dx(m), y=dy(m), z=dyaw(deg)");
    RCLCPP_INFO(
        get_logger(),
        "health thresholds: dx<=%.1fm, dy<=%.1fm, dyaw<=%.1fdeg, grace=%.1fs, consecutive_limit=%d",
        max_dx_, max_dy_, max_dyaw_deg_, startup_grace_sec_,
        consecutive_limit_);
  }

 private:
  struct PoseData {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw_deg = 0.0;
  };

  static double quatToYaw(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  static double radToDeg(double rad) { return rad * 180.0 / M_PI; }

  static double wrapDeg180(double angle_deg) {
    while (angle_deg > 180.0) {
      angle_deg -= 360.0;
    }
    while (angle_deg < -180.0) {
      angle_deg += 360.0;
    }
    return angle_deg;
  }

  static PoseData poseToData(const geometry_msgs::msg::Pose& pose) {
    const auto& p = pose.position;
    const auto& q = pose.orientation;
    const double yaw_rad = quatToYaw(q.x, q.y, q.z, q.w);

    PoseData data;
    data.x = p.x;
    data.y = p.y;
    data.z = p.z;
    data.yaw_deg = radToDeg(yaw_rad);
    return data;
  }

  static PoseData poseToData(const geometry_msgs::msg::PoseStamped& msg) {
    return poseToData(msg.pose);
  }

  static PoseData poseToData(const nav_msgs::msg::Odometry& msg) {
    return poseToData(msg.pose.pose);
  }

  void setExternalPose(const PoseData& pose, const char* source) {
    external_pose_ = pose;
    external_source_ = source;
    external_received_ = true;
  }

  void visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    vision_received_ = true;
    setExternalPose(poseToData(*msg), "vision_pose");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_received_ = true;
    setExternalPose(poseToData(*msg), "odometry_out");
  }

  void localCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    local_pose_ = poseToData(*msg);
    local_received_ = true;
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    armed_ = msg->armed;
    state_received_ = true;
  }

  void printResult() {
    const rclcpp::Time now_time = now();

    if (!external_received_ || !local_received_) {
      if ((now_time - last_wait_log_time_).seconds() > 1.0) {
        last_wait_log_time_ = now_time;
        if (!vision_received_) {
          RCLCPP_WARN(get_logger(),
                      "/mavros/vision_pose/pose not received yet");
        }
        if (!odom_received_) {
          RCLCPP_WARN(get_logger(), "/mavros/odometry/out not received yet");
        }
        if (!external_received_) {
          RCLCPP_WARN(get_logger(),
                      "no external pose source active yet (waiting vision_pose or odometry_out)");
        }
        if (!local_received_) {
          RCLCPP_WARN(get_logger(),
                      "/mavros/local_position/pose not received yet");
        }
      }
      return;
    }

    const double ex = external_pose_.x;
    const double ey = external_pose_.y;
    const double ez = external_pose_.z;
    const double eyaw = external_pose_.yaw_deg;

    const double lx = local_pose_.x;
    const double ly = local_pose_.y;
    const double lz = local_pose_.z;
    const double lyaw = local_pose_.yaw_deg;

    const double dx = lx - ex;
    const double dy = ly - ey;
    const double dz = lz - ez;
    const double dyaw = wrapDeg180(lyaw - eyaw);

    geometry_msgs::msg::Vector3 delta_msg;
    delta_msg.x = dx;
    delta_msg.y = dy;
    delta_msg.z = dyaw;
    delta_pub_->publish(delta_msg);

    RCLCPP_INFO(
        get_logger(),
        "\nsource: %s\nexternal: x=%+8.3f  y=%+8.3f  z=%+8.3f  yaw=%+8.2f deg\nlocal   : x=%+8.3f  y=%+8.3f  z=%+8.3f  yaw=%+8.2f deg\ndelta   : dx=%+8.3f dy=%+8.3f dz=%+8.3f dyaw=%+8.2f deg",
        external_source_.c_str(), ex, ey, ez, eyaw, lx, ly, lz, lyaw, dx, dy,
        dz, dyaw);

    if ((now_time - start_time_).seconds() < startup_grace_sec_) {
      consecutive_exceed_count_ = 0;
      return;
    }

    if (!state_received_) {
      consecutive_exceed_count_ = 0;
      if ((now_time - last_state_wait_log_time_).seconds() > 1.0) {
        last_state_wait_log_time_ = now_time;
        RCLCPP_WARN(get_logger(),
                    "/mavros/state not received yet, restart gating disabled");
      }
      return;
    }

    if (armed_) {
      consecutive_exceed_count_ = 0;
      return;
    }

    const bool exceeded = std::abs(dx) > max_dx_ || std::abs(dy) > max_dy_;

    if (exceeded) {
      ++consecutive_exceed_count_;
      RCLCPP_WARN(
          get_logger(),
          "source=%s delta exceeded threshold (%d/%d): dx=%+.3fm dy=%+.3fm dyaw=%+.2fdeg; thresholds=(%.1fm, %.1fm, %.1fdeg); armed=%s",
          external_source_.c_str(), consecutive_exceed_count_,
          consecutive_limit_, dx, dy, dyaw, max_dx_, max_dy_, max_dyaw_deg_,
          armed_ ? "true" : "false");
    } else {
      consecutive_exceed_count_ = 0;
    }

    if (consecutive_exceed_count_ >= consecutive_limit_) {
      throw std::runtime_error(
          "pose delta exceeded threshold continuously while disarmed: source=" +
          external_source_ + " dx=" + std::to_string(dx) +
          "m dy=" + std::to_string(dy) + "m dyaw=" +
          std::to_string(dyaw) + "deg armed=false");
    }
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr delta_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  PoseData external_pose_;
  PoseData local_pose_;
  std::string external_source_ = "none";
  bool vision_received_ = false;
  bool odom_received_ = false;
  bool external_received_ = false;
  bool local_received_ = false;
  bool state_received_ = false;
  bool armed_ = false;

  double max_dx_ = 1.0;
  double max_dy_ = 1.0;
  double max_dyaw_deg_ = 30.0;
  double startup_grace_sec_ = 15.0;
  int consecutive_limit_ = 10;
  int consecutive_exceed_count_ = 0;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wait_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_wait_log_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto node = std::make_shared<PoseYawComparerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_ERROR(rclcpp::get_logger("compare_yaw_node"),
                 "compare_yaw fatal: %s", exc.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}

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

    external_odom_topic_ = declare_parameter<std::string>(
        "external_odom_topic", "/mavros/odometry/out");
    local_pose_topic_ = declare_parameter<std::string>(
        "local_pose_topic", "/mavros/local_position/pose");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");

    external_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        external_odom_topic_, mavros_pose_qos,
        std::bind(&PoseYawComparerNode::externalOdomCallback, this,
                  std::placeholders::_1));

    local_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        local_pose_topic_, mavros_pose_qos,
        std::bind(&PoseYawComparerNode::localCallback, this,
                  std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        state_topic_, mavros_pose_qos,
        std::bind(&PoseYawComparerNode::stateCallback, this,
                  std::placeholders::_1));

    delta_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
        "/pose_yaw_compare/delta", 10);

    // ===== 可调参数：重启判定阈值 =====
    // max_dx: dx 的绝对值阈值，单位米。
    // 当 abs(dx) > max_dx 时，认为 x 方向偏差超限。
    max_dx_ = 1.0;
    // max_dy: dy 的绝对值阈值，单位米。
    // 当 abs(dy) > max_dy 时，认为 y 方向偏差超限。
    max_dy_ = 1.0;
    // max_dyaw_deg: dyaw 的绝对值阈值，单位度。
    // 当 abs(dyaw) > max_dyaw_deg 时，认为航向偏差超限。
    max_dyaw_deg_ = 30.0;

    // startup_grace_sec: 冷启动宽限期，单位秒。
    // 节点启动后的这段时间内，只发布/打印 delta，不做超限重启判定。
    startup_grace_sec_ = 15.0;

    // consecutive_limit: 连续超限次数阈值。
    // 当前定时器周期为 0.2 秒，因此 10 次约等于持续超限 2 秒才触发重启。
    // 如果想更敏感就减小；想更保守就增大。
    consecutive_limit_ = 10;

    // ===== 运行时状态：通常不需要手动改 =====
    // consecutive_exceed_count: 当前连续超限计数。
    // 一旦某次检查恢复正常，或者处于 armed==true 状态，会被清零。
    consecutive_exceed_count_ = 0;
    // start_time_: 节点启动时间，用于计算冷启动宽限期。
    start_time_ = now();
    // state_received: 是否已经收到 /mavros/state。
    // 未收到时禁止触发重启，避免启动早期误判。
    state_received_ = false;
    // armed: 当前无人机解锁状态，取自 /mavros/state.armed。
    // 约定：仅在 armed == false（关锁）时允许因持续超限触发重启。
    armed_ = false;
    // last_state_wait_log_time_: /mavros/state 未到达时的日志节流时间戳。
    last_state_wait_log_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    // printResult 定时器周期，单位秒。
    // 它同时影响：
    // 1. delta 发布/日志打印频率
    // 2. 连续超限计数换算到真实时间的速度
    // 例如当前 0.2 秒一次，consecutive_limit=10 就约等于持续 2 秒超限。
    timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&PoseYawComparerNode::printResult, this));

    RCLCPP_INFO(get_logger(), "pose yaw comparer started");
    RCLCPP_INFO(get_logger(), "subscribed external odometry: %s",
                external_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "subscribed local pose: %s",
                local_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "subscribed state: %s", state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "subscription QoS: BEST_EFFORT / VOLATILE");
    RCLCPP_INFO(get_logger(), "publishing delta to: /pose_yaw_compare/delta");
    RCLCPP_INFO(get_logger(), "delta meaning: x=dx(m), y=dy(m), z=dyaw(deg)");
    RCLCPP_INFO(
        get_logger(),
        "health thresholds: dx<=%.1fm, dy<=%.1fm, dyaw<=%.1fdeg, grace=%.1fs, consecutive_limit=%d",
        max_dx_, max_dy_, max_dyaw_deg_, startup_grace_sec_, consecutive_limit_);
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

  static PoseData poseToData(const geometry_msgs::msg::PoseStamped& msg) {
    const auto& p = msg.pose.position;
    const auto& q = msg.pose.orientation;
    const double yaw_rad = quatToYaw(q.x, q.y, q.z, q.w);

    PoseData data;
    data.x = p.x;
    data.y = p.y;
    data.z = p.z;
    data.yaw_deg = radToDeg(yaw_rad);
    return data;
  }

  static PoseData odometryToData(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    const double yaw_rad = quatToYaw(q.x, q.y, q.z, q.w);

    PoseData data;
    data.x = p.x;
    data.y = p.y;
    data.z = p.z;
    data.yaw_deg = radToDeg(yaw_rad);
    return data;
  }

  void externalOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    external_pose_ = odometryToData(*msg);
    external_received_ = true;
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
        if (!external_received_) {
          RCLCPP_WARN(get_logger(), "%s not received yet",
                      external_odom_topic_.c_str());
        }
        if (!local_received_) {
          RCLCPP_WARN(get_logger(), "%s not received yet",
                      local_pose_topic_.c_str());
        }
      }
      return;
    }

    const double vx = external_pose_.x;
    const double vy = external_pose_.y;
    const double vz = external_pose_.z;
    const double vyaw = external_pose_.yaw_deg;

    const double lx = local_pose_.x;
    const double ly = local_pose_.y;
    const double lz = local_pose_.z;
    const double lyaw = local_pose_.yaw_deg;

    const double dx = lx - vx;
    const double dy = ly - vy;
    const double dz = lz - vz;
    const double dyaw = wrapDeg180(lyaw - vyaw);

    geometry_msgs::msg::Vector3 delta_msg;
    delta_msg.x = dx;
    delta_msg.y = dy;
    delta_msg.z = dyaw;
    delta_pub_->publish(delta_msg);

    RCLCPP_INFO(
        get_logger(),
        "\nexternal: x=%+8.3f  y=%+8.3f  z=%+8.3f  yaw=%+8.2f deg\nlocal   : x=%+8.3f  y=%+8.3f  z=%+8.3f  yaw=%+8.2f deg\ndelta   : dx=%+8.3f dy=%+8.3f dz=%+8.3f dyaw=%+8.2f deg",
        vx, vy, vz, vyaw, lx, ly, lz, lyaw, dx, dy, dz, dyaw);

    // 冷启动宽限期判断：在 startup_grace_sec 时间内，即使偏差超限也不触发重启。
    // 如果你觉得系统刚启动时抖动较大，可以把 startup_grace_sec 调大。
    if ((now_time - start_time_).seconds() < startup_grace_sec_) {
      consecutive_exceed_count_ = 0;
      return;
    }

    // 未收到 /mavros/state 时，禁止进入重启判定。
    // 这是为了防止节点刚起来、状态源还没到时误判重启。
    if (!state_received_) {
      consecutive_exceed_count_ = 0;
      if ((now_time - last_state_wait_log_time_).seconds() > 1.0) {
        last_state_wait_log_time_ = now_time;
        RCLCPP_WARN(get_logger(),
                    "/mavros/state not received yet, restart gating disabled");
      }
      return;
    }

    // 解锁状态门控：armed == true 时，即使持续超阈值也不允许自动重启。
    // 这是为了避免无人机飞行中直接触发整条链重启。
    // 如果你未来想改成“解锁时也允许重启”，就是从这里调整逻辑。
    if (armed_) {
      consecutive_exceed_count_ = 0;
      return;
    }

    // 超限判定：只要 dx / dy / dyaw 三者中任意一个绝对值超阈值，就记作一次超限。
    // 如果你想把判定改得更严格，例如要求“同时两个量超限才算异常”，就改这里。
    const bool exceeded = std::abs(dx) > max_dx_ || std::abs(dy) > max_dy_;// ||
                          //std::abs(dyaw) > max_dyaw_deg_;

    if (exceeded) {
      // 一次超限就把连续计数 +1。
      // 想更快触发重启：减小 consecutive_limit。
      // 想更难触发重启：增大 consecutive_limit。
      ++consecutive_exceed_count_;
      RCLCPP_WARN(
          get_logger(),
          "delta exceeded threshold (%d/%d): dx=%+.3fm dy=%+.3fm dyaw=%+.2fdeg; thresholds=(%.1fm, %.1fm, %.1fdeg); armed=%s",
          consecutive_exceed_count_, consecutive_limit_, dx, dy, dyaw, max_dx_,
          max_dy_, max_dyaw_deg_, armed_ ? "true" : "false");
    } else {
      // 只要本次恢复正常，就把连续超限计数清零。
      // 这能避免零星毛刺累积成重启。
      consecutive_exceed_count_ = 0;
    }

    // 真正触发重启的位置：
    // 只有在 armed == false 且连续超限次数达到 consecutive_limit 时，才会抛异常退出。
    // supervisor 会把这个退出视为异常，从而触发整条链重启。
    if (consecutive_exceed_count_ >= consecutive_limit_) {
      throw std::runtime_error(
          "pose delta exceeded threshold continuously while disarmed: dx=" +
          std::to_string(dx) + "m dy=" + std::to_string(dy) +
          "m dyaw=" + std::to_string(dyaw) +
          "deg armed=false");
    }
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr delta_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  PoseData external_pose_;
  PoseData local_pose_;
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

  std::string external_odom_topic_ = "/mavros/odometry/out";
  std::string local_pose_topic_ = "/mavros/local_position/pose";
  std::string state_topic_ = "/mavros/state";

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

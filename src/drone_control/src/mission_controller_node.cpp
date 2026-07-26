#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "drone_control/action_executor.hpp"
#include "drone_control/drone_action.hpp"

#include "drone_msgs/msg/task_status.hpp"

namespace offboard_run {

class MissionController {
 public:
  explicit MissionController(const rclcpp::Node::SharedPtr &node)
      : node_(node),
        tf_buffer_(node_->get_clock()),
        tf_listener_(tf_buffer_, node_, true) {
    mission_config_path_ =
        node_->declare_parameter<std::string>("mission_config_path", "");
    auto_start_mission_ =
        node_->declare_parameter<bool>("auto_start_mission", true);
    takeoff_altitude_ =
        node_->declare_parameter<double>("takeoff_altitude", 1.2);

    tf_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/mavros_tf/status", 10,
        std::bind(&MissionController::tfStatusCb, this,
                  std::placeholders::_1));
    state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        std::bind(&MissionController::stateCb, this, std::placeholders::_1));
    start_mission_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/start_mission", 1,
        std::bind(&MissionController::startMissionCb, this,
                  std::placeholders::_1));
    stop_mission_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/stop_mission", 1,
        std::bind(&MissionController::stopMissionCb, this,
                  std::placeholders::_1));

    task_status_pub_ = node_->create_publisher<drone_msgs::msg::TaskStatus>(
      "/task/status",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local()
    );

    arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");
    set_mode_client_ =
        node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    action_executor_ = std::make_unique<ActionExecutor>(node_, tf_buffer_);

    if (!loadMissionFromFile(mission_config_path_)) {
      RCLCPP_ERROR(node_->get_logger(), "任务配置加载失败：%s",
                   mission_config_path_.c_str());
    }

    mission_start_requested_ = auto_start_mission_;

    publishTaskStatus(false, "idle", 0, static_cast<uint8_t>(mission_actions_.size()));

    RCLCPP_INFO(node_->get_logger(),
                "任务控制器已就绪。自动启动：%s，起飞高度：%.2f m",
                auto_start_mission_ ? "是" : "否",
                takeoff_altitude_);
  }

  void controlLoop() {
    if (!hasStartRequest() && !initialized_) {
      if (!waiting_start_request_logged_) {
        RCLCPP_INFO(node_->get_logger(),
                    "自动启动已关闭，控制器待命中。等待 /start_mission 后再切换 "
                    "OFFBOARD、解锁并执行任务。");
        waiting_start_request_logged_ = true;
      }
      return;
    }

    initializeIfNeeded();

    if (initialized_ && hasStartRequest() && !mission_running_) {
      startMission();
    }

    if (!mission_running_) {
      action_executor_->sendDummyPose(takeoff_altitude_);
      return;
    }

    action_executor_->controlLoop();

    const auto runtime = action_executor_->getTaskRuntimeStatus();
    const auto total_actions = static_cast<uint8_t>(mission_actions_.size());

    if (runtime.task_running) {
      publishTaskStatus(true,
                        runtime.action_name,
                        runtime.action_step,
                        total_actions);
    }

    if (mission_running_ && action_executor_->isIdle()) {
      mission_running_ = false;
      publishTaskStatus(false, "finished", static_cast<int32_t>(mission_actions_.size()), total_actions);
    }
  }

 private:
  bool loadMissionFromFile(const std::string &path) {
    if (path.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "任务配置路径 mission_config_path 为空。");
      return false;
    }

    YAML::Node config;
    try {
      config = YAML::LoadFile(path);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "YAML 加载失败：%s", e.what());
      return false;
    }

    system_config_ = config["system"] ? config["system"] : YAML::Node();
    if (system_config_) {
      auto_start_mission_ =
          system_config_["auto_start_mission"]
              ? system_config_["auto_start_mission"].as<bool>()
              : auto_start_mission_;
    }

    if (!config["mission"] || !config["mission"]["actions"]) {
      RCLCPP_ERROR(node_->get_logger(), "YAML 缺少 mission/actions 配置。");
      return false;
    }

    takeoff_altitude_ =
        config["mission"]["takeoff_altitude"]
            ? config["mission"]["takeoff_altitude"].as<double>()
            : takeoff_altitude_;

    mission_actions_.clear();
    const YAML::Node actions = config["mission"]["actions"];
    RCLCPP_INFO(node_->get_logger(), "正在加载 %zu 个任务动作...",
                actions.size());

    for (std::size_t i = 0; i < actions.size(); ++i) {
      const YAML::Node item = actions[i];
      if (!item["type"]) {
        RCLCPP_WARN(node_->get_logger(), "第 %zu 个动作缺少 type 字段，已跳过。",
                    i);
        continue;
      }

      const std::string type = item["type"].as<std::string>();
      std::shared_ptr<DroneAction> action;

      if (type == "takeoff") {
        const double altitude =
            item["altitude"] ? item["altitude"].as<double>() : takeoff_altitude_;
        action = DroneAction::createTakeoffAction(altitude, 0.15);
        RCLCPP_INFO(node_->get_logger(), "  %zu：起飞到 %.2f m", i,
                    altitude);
      } else if (type == "land") {
        action = DroneAction::createLandAction();
        RCLCPP_INFO(node_->get_logger(), "  %zu：降落", i);
      } else if (type == "move") {
        action = parseMoveAction(item, i);
      } else if (type == "hover") {
        const double duration =
            item["duration"] ? item["duration"].as<double>() : 1.0;
        const bool vision_hover =
            item["vision_hover"] ? item["vision_hover"].as<bool>() : false;
        action = DroneAction::createHoverAction(duration, vision_hover);
        RCLCPP_INFO(node_->get_logger(), "  %zu：悬停 %.2f s，视觉悬停通知：%s",
                    i, duration, vision_hover ? "true" : "false");
      } else if (type == "visual_servo") {
        action = parseVisualServoAction(item, i);
      } else {
        RCLCPP_WARN(node_->get_logger(), "未知动作类型 '%s'，已跳过。",
                    type.c_str());
      }

      if (action) {
        mission_actions_.push_back(action);
      }
    }

    RCLCPP_INFO(node_->get_logger(), "任务加载完成，有效动作数量：%zu。",
                mission_actions_.size());
    return !mission_actions_.empty();
  }

  template <typename T>
  T actionOrSystemValue(const YAML::Node &action, const char *key,
                        const T &built_in_default) const {
    if (action[key]) {
      return action[key].as<T>();
    }
    if (system_config_ && system_config_[key]) {
      return system_config_[key].as<T>();
    }
    return built_in_default;
  }

  std::shared_ptr<DroneAction> parseMoveAction(const YAML::Node &item,
                                               std::size_t index) {
    geometry_msgs::msg::PoseStamped target;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 57.29577951308232;
    const std::string frame =
        item["frame"] ? item["frame"].as<std::string>() : "world_body";
    const YAML::Node pos = item["position"];
    if (!pos || pos.size() != 3) {
      RCLCPP_WARN(node_->get_logger(),
                  "第 %zu 个移动动作缺少 position: [x, y, z]。", index);
      return nullptr;
    }

    target.header.frame_id = frame;
    target.pose.position.x = pos[0].as<double>();
    target.pose.position.y = pos[1].as<double>();
    target.pose.position.z = pos[2].as<double>();

    const double yaw = item["yaw"] ? item["yaw"].as<double>() : 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    target.pose.orientation = tf2::toMsg(q);

    const double tolerance =
        actionOrSystemValue<double>(item, "tolerance", 0.1);
    const double yaw_tolerance_deg =
        actionOrSystemValue<double>(item, "yaw_tolerance_deg", 5.0);
    const double max_xy_speed_mps =
        actionOrSystemValue<double>(item, "max_xy_speed_mps", 0.35);
    const double max_z_speed_mps =
        actionOrSystemValue<double>(item, "max_z_speed_mps", 0.20);
    const double max_yaw_rate_deg_s =
        actionOrSystemValue<double>(item, "max_yaw_rate_deg_s", 30.0);

    auto use_frame = parseFrame(frame);
    RCLCPP_INFO(node_->get_logger(),
                "  %zu：移动到 [%.2f, %.2f, %.2f]，坐标系=%s，偏航=%.2f 度，"
                "位置容差=%.2f m，偏航容差=%.2f 度，xy速度=%.2f m/s，z速度=%.2f m/s，"
                "yaw角速度=%.2f 度/s",
                index, target.pose.position.x, target.pose.position.y,
                target.pose.position.z, frame.c_str(), yaw * kRadToDeg,
                tolerance, yaw_tolerance_deg, max_xy_speed_mps,
                max_z_speed_mps, max_yaw_rate_deg_s);
    return DroneAction::createMoveToAction(
        target,
        use_frame,
        tolerance,
        yaw_tolerance_deg * kDegToRad,
        max_xy_speed_mps,
        max_z_speed_mps,
        max_yaw_rate_deg_s * kDegToRad);
  }

  BodyAxis parseBodyAxis(const std::string &axis, BodyAxis fallback,
                         std::size_t index, const char *field) const {
    if (axis == "x" || axis == "X") {
      return BodyAxis::X;
    }
    if (axis == "y" || axis == "Y") {
      return BodyAxis::Y;
    }
    if (axis == "z" || axis == "Z") {
      return BodyAxis::Z;
    }
    RCLCPP_WARN(node_->get_logger(),
                "第 %zu 个 visual_servo 的 %s=%s 无效，使用默认轴。",
                index, field, axis.c_str());
    return fallback;
  }

  std::shared_ptr<DroneAction> parseVisualServoAction(
      const YAML::Node &item, std::size_t index) {
    VisualServoConfig config;
    config.target_id =
        actionOrSystemValue<std::string>(item, "target_id", "");
    config.require_confirmed =
        actionOrSystemValue<bool>(item, "require_confirmed", true);
    config.image_x_axis = parseBodyAxis(
        actionOrSystemValue<std::string>(item, "image_x_axis", "y"),
        BodyAxis::Y, index, "image_x_axis");
    config.image_y_axis = parseBodyAxis(
        actionOrSystemValue<std::string>(item, "image_y_axis", "z"),
        BodyAxis::Z, index, "image_y_axis");
    if (config.image_x_axis == config.image_y_axis) {
      RCLCPP_ERROR(node_->get_logger(),
                   "第 %zu 个 visual_servo 的图像 X/Y 不能映射到同一机体系轴。",
                   index);
      return nullptr;
    }

    config.image_x_sign =
        actionOrSystemValue<double>(item, "image_x_sign", -1.0);
    config.image_y_sign =
        actionOrSystemValue<double>(item, "image_y_sign", -1.0);
    config.kp_x = actionOrSystemValue<double>(item, "kp_x", 0.35);
    config.ki_x = actionOrSystemValue<double>(item, "ki_x", 0.0);
    config.kd_x = actionOrSystemValue<double>(item, "kd_x", 0.02);
    config.kp_y = actionOrSystemValue<double>(item, "kp_y", 0.35);
    config.ki_y = actionOrSystemValue<double>(item, "ki_y", 0.0);
    config.kd_y = actionOrSystemValue<double>(item, "kd_y", 0.02);
    config.integral_limit =
        actionOrSystemValue<double>(item, "integral_limit", 0.5);
    config.filter_alpha =
        actionOrSystemValue<double>(item, "filter_alpha", 0.35);
    config.enter_tolerance_x =
        actionOrSystemValue<double>(item, "enter_tolerance_x", 0.04);
    config.enter_tolerance_y =
        actionOrSystemValue<double>(item, "enter_tolerance_y", 0.04);
    config.exit_tolerance_x =
        actionOrSystemValue<double>(item, "exit_tolerance_x", 0.07);
    config.exit_tolerance_y =
        actionOrSystemValue<double>(item, "exit_tolerance_y", 0.07);
    config.settle_time_s =
        actionOrSystemValue<double>(item, "settle_time_s", 0.6);
    config.acquire_timeout_s =
        actionOrSystemValue<double>(item, "acquire_timeout_s", 5.0);
    config.lost_timeout_s =
        actionOrSystemValue<double>(item, "lost_timeout_s", 1.0);
    config.overall_timeout_s =
        actionOrSystemValue<double>(item, "overall_timeout_s", 20.0);
    config.max_body_speed_mps =
        actionOrSystemValue<double>(item, "max_body_speed_mps", 0.20);
    const bool continue_on_timeout =
        actionOrSystemValue<bool>(item, "continue_on_timeout", true);

    if (config.enter_tolerance_x <= 0.0 || config.enter_tolerance_y <= 0.0 ||
        config.exit_tolerance_x < config.enter_tolerance_x ||
        config.exit_tolerance_y < config.enter_tolerance_y ||
        config.settle_time_s < 0.0 || config.acquire_timeout_s <= 0.0 ||
        config.lost_timeout_s <= 0.0 || config.overall_timeout_s <= 0.0 ||
        config.max_body_speed_mps <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(),
                   "第 %zu 个 visual_servo 参数非法：请检查容差、超时和速度限制。",
                   index);
      return nullptr;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "  %zu：通用视觉伺服，target_id=%s，轴映射=(%d,%d)，最大速度=%.2f m/s，总超时=%.1f s",
        index, config.target_id.empty() ? "<any>" : config.target_id.c_str(),
        static_cast<int>(config.image_x_axis),
        static_cast<int>(config.image_y_axis), config.max_body_speed_mps,
        config.overall_timeout_s);
    return DroneAction::createVisualServoAction(config, continue_on_timeout);
  }

  DroneAction::Frame parseFrame(const std::string &frame) const {
    if (frame == "body") {
      return DroneAction::Frame::BODY;
    }
    if (frame == "world_enu") {
      return DroneAction::Frame::WORLD_ENU;
    }
    return DroneAction::Frame::WORLD_BODY;
  }

  bool hasStartRequest() const {
    return mission_start_requested_;
  }

  void initializeIfNeeded() {
    if (initialized_) {
      return;
    }

    const rclcpp::Time now = node_->now();
    if (last_init_attempt_.nanoseconds() != 0 &&
        (now - last_init_attempt_).seconds() < 1.5) {
      action_executor_->sendDummyPose(takeoff_altitude_);
      return;
    }
    last_init_attempt_ = now;

    if (!current_state_.connected) {
      RCLCPP_INFO(node_->get_logger(), "正在等待 MAVROS 连接...");
      action_executor_->sendDummyPose(takeoff_altitude_);
      return;
    }

    if (!initial_setpoints_sent_) {
      RCLCPP_INFO(node_->get_logger(), "正在发送初始位置设定点...");
      geometry_msgs::msg::PoseStamped hover_pose;
      hover_pose.header.frame_id = "world_enu";
      hover_pose.pose.position.z = takeoff_altitude_;
      hover_pose.pose.orientation.w = 1.0;

      rclcpp::Rate rate(50);
      for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        hover_pose.header.stamp = node_->now();
        action_executor_->sendPositionSetpoint(hover_pose);
        rclcpp::spin_some(node_);
        rate.sleep();
      }
      initial_setpoints_sent_ = true;
      RCLCPP_INFO(node_->get_logger(), "初始位置设定点已发送。");
    }

    if (current_state_.mode != "OFFBOARD") {
      if (callSetMode("OFFBOARD")) {
        RCLCPP_INFO(node_->get_logger(), "已切换到 OFFBOARD 模式。");
      } else {
        RCLCPP_WARN(node_->get_logger(), "OFFBOARD 模式切换失败。");
      }
      return;
    }

    if (!current_state_.armed) {
      if (callArm(true)) {
        initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "飞行器已解锁。");
      } else {
        RCLCPP_WARN(node_->get_logger(), "飞行器解锁失败。");
      }
      return;
    }

    initialized_ = true;
  }

  bool startMission() {
    if (!initialized_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "无法启动任务：控制器尚未初始化。");
      return false;
    }
    if (mission_actions_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "无法启动任务：没有有效动作。");
      return false;
    }
    if (!tf_ready_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "正在等待 TF 就绪，暂不启动任务。");
      return false;
    }

    action_executor_->clearAction();
    for (const auto &action : mission_actions_) {
      action_executor_->addAction(action);
    }

    mission_running_ = true;
    mission_start_requested_ = false;

    publishTaskStatus(false, "queued", 0,
                        static_cast<uint8_t>(mission_actions_.size()));

    RCLCPP_INFO(node_->get_logger(), "任务已启动，动作数量：%zu。",
                mission_actions_.size());
    return true;
  }

  void stopMission() {
    RCLCPP_INFO(node_->get_logger(), "正在停止任务。");
    action_executor_->emergencyStop();
    mission_running_ = false;
      publishTaskStatus(false, "stopped", 0,
                        static_cast<uint8_t>(mission_actions_.size()));
  }

  bool callSetMode(const std::string &mode) {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "飞行模式切换服务不可用。");
      return false;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    auto future = set_mode_client_->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(
        node_, future, std::chrono::seconds(2));
    return result == rclcpp::FutureReturnCode::SUCCESS &&
           future.get()->mode_sent;
  }

  bool callArm(bool arm) {
    if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "解锁服务不可用。");
      return false;
    }
    auto request =
        std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    auto future = arming_client_->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(
        node_, future, std::chrono::seconds(2));
    return result == rclcpp::FutureReturnCode::SUCCESS &&
           future.get()->success;
  }

  void tfStatusCb(const std_msgs::msg::Bool::SharedPtr msg) {
    tf_ready_ = msg->data;
    if (tf_ready_ && !tf_ready_logged_) {
      RCLCPP_INFO(node_->get_logger(), "TF 已就绪。");
      tf_ready_logged_ = true;
    }
  }

  void stateCb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
  }

  void startMissionCb(const std_msgs::msg::Empty::SharedPtr) {
    mission_start_requested_ = true;
    waiting_start_request_logged_ = false;
    if (!mission_running_) {
      publishTaskStatus(false, "waiting_start", 0, static_cast<uint8_t>(mission_actions_.size()));
    }
    RCLCPP_INFO(node_->get_logger(),
                "收到启动任务指令，开始初始化 OFFBOARD、解锁并准备执行任务。");
  }

  void stopMissionCb(const std_msgs::msg::Empty::SharedPtr) {
    RCLCPP_INFO(node_->get_logger(), "收到停止任务指令。");
    stopMission();
  }

  void publishTaskStatus(bool task_running, const std::string &action_name,
                         int32_t action_step, uint8_t action_num)
  {
    if (!task_status_pub_) {
      RCLCPP_WARN(node_->get_logger(), "task_status_pub_ 尚未初始化。");
      return;
    }

    if (has_last_task_status_ && 
        last_task_running_ == task_running &&
        last_action_name_ == action_name &&
        last_action_step_ == action_step &&
        last_action_num_ == action_num) {
          return;
        }

    drone_msgs::msg::TaskStatus msg;
    msg.task_running = task_running;
    msg.action_name = action_name;
    msg.action_step = action_step;
    msg.action_num = action_num;
    task_status_pub_->publish(msg);

    has_last_task_status_ = true;
    last_task_running_ = task_running;
    last_action_name_ = action_name;
    last_action_step_ = action_step;
    last_action_num_ = action_num;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tf_status_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_mission_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  std::unique_ptr<ActionExecutor> action_executor_;

  std::vector<std::shared_ptr<DroneAction>> mission_actions_;
  mavros_msgs::msg::State current_state_;
  std::string mission_config_path_;
  YAML::Node system_config_;

  rclcpp::Publisher<drone_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;

  bool tf_ready_ = false;
  bool tf_ready_logged_ = false;
  bool initialized_ = false;
  bool initial_setpoints_sent_ = false;
  bool auto_start_mission_ = true;
  bool mission_start_requested_ = false;
  bool mission_running_ = false;
  bool waiting_start_request_logged_ = false;
  double takeoff_altitude_ = 1.2;
  rclcpp::Time last_init_attempt_;

  bool has_last_task_status_ = false;
  bool last_task_running_ = false;
  std::string last_action_name_;
  int32_t last_action_step_ = -1;
  uint8_t last_action_num_ = 0;
};

}  // namespace offboard_run

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_controller_node");

  try {
    offboard_run::MissionController controller(node);
    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
      controller.controlLoop();
      rclcpp::spin_some(node);
      rate.sleep();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "任务控制器异常：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

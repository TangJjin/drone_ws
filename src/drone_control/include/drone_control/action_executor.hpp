#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include "drone_msgs/msg/vision_servo_status.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <memory>
#include <queue>
#include <sstream>
#include <string>

#include "drone_action.hpp"

using offboard_run::ActionStatus;
using offboard_run::ActionType;
using offboard_run::DroneAction;
using offboard_run::SpatialPoint;
using offboard_run::VisualServoController;
using offboard_run::VisualServoObservation;
using offboard_run::VisualServoOutput;
using offboard_run::VisualServoState;

struct TaskRuntimeStatus
{
    bool task_running;
    std::string action_name;
    int32_t action_step;
};

struct MoveRuntimeState
{
    bool initialized = false;

    geometry_msgs::msg::PoseStamped start_pose;
    // 进入动作时一次性冻结后的world_enu终点
    geometry_msgs::msg::PoseStamped resolved_target_pose;
    // 上一次真正发出去的平滑 setpoint
    geometry_msgs::msg::PoseStamped last_command_pose;

    rclcpp::Time last_update_time;

    double start_yaw_rad = 0.0;
    // 用于 yaw 平滑推进
    double target_yaw_rad = 0.0;
    double last_command_yaw_rad = 0.0;

    double total_distance_m = 0.0;
    int stable_count = 0;
};

class ActionExecutor
{
public:
    ActionExecutor(const rclcpp::Node::SharedPtr &node,
                   tf2_ros::Buffer &tf_buffer)
        : node_(node),
          tf_buffer_(tf_buffer)
    {
        land_setpoint_quiet_time_s_ =
            node_->declare_parameter<double>("land_setpoint_quiet_time_s", 0.2);

        setpoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());

        step_pub_ = node_->create_publisher<std_msgs::msg::String>("/step", 10);
        mission_status_pub_ =
            node_->create_publisher<std_msgs::msg::String>("/mission_status", 10);

        hover_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/mission/hover_active", rclcpp::QoS(10).reliable().transient_local());
 
        visual_servo_status_pub_ =
            node_->create_publisher<drone_msgs::msg::VisionServoStatus>(
                "/control/vision_servo/status",
                rclcpp::QoS(10).reliable().transient_local());
        
        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&ActionExecutor::state_callback, this, std::placeholders::_1));

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&ActionExecutor::pose_callback, this, std::placeholders::_1));

        visual_servo_target_sub_ =
            node_->create_subscription<geometry_msgs::msg::PointStamped>(
                "/air_ground_servo/target_point", rclcpp::SensorDataQoS(),
                std::bind(
                    &ActionExecutor::visualServoTargetCallback,
                    this, std::placeholders::_1));

        arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        land_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

        last_finish_pose_.header.frame_id = "world_enu";
        last_finish_pose_.pose.orientation.w = 1.0;

        RCLCPP_INFO(node_->get_logger(), "动作执行器初始化完成，已准备执行任务。");
    }

    void addAction(const std::shared_ptr<DroneAction> &action)
    {
        action_queue_.push(action);
        RCLCPP_INFO(node_->get_logger(), "已添加动作，当前队列长度：%zu", action_queue_.size());
    }

    void clearAction()
    {
        publishVisionHoverActive(false);
        while (!action_queue_.empty())
        {
            action_queue_.pop();
        }
        action_id_ = 0;
        current_action_.reset();
        resetActionRuntimeState();
    }

    void emergencyStop()
    {
        clearAction();
        if (current_pose_received_)
        {
            sendPositionSetpoint(current_pose_);
        }
    }

    void sendPositionSetpoint(const geometry_msgs::msg::PoseStamped &pose)
    {
        geometry_msgs::msg::PoseStamped target = pose;
        target.header.stamp = node_->now();
        if (target.header.frame_id.empty())
        {
            target.header.frame_id = "world_enu";
        }
        setpoint_pub_->publish(target);
    }

    void sendDummyPose(double altitude = 0.0)
    {
        if (current_pose_received_)
        {
            sendPositionSetpoint(current_pose_);
            return;
        }

        geometry_msgs::msg::PoseStamped dummy_pose;
        dummy_pose.header.frame_id = "world_enu";
        dummy_pose.header.stamp = node_->now();
        dummy_pose.pose.position.z = altitude;
        dummy_pose.pose.orientation.w = 1.0;
        sendPositionSetpoint(dummy_pose);
    }

    bool isIdle() const
    {
        return !current_action_ && action_queue_.empty();
    }

    TaskRuntimeStatus getTaskRuntimeStatus() const
    {
        TaskRuntimeStatus status;
        status.task_running = static_cast<bool>(current_action_);
        status.action_step = current_action_ ? action_id_ : 0;
        status.action_name = current_action_ ? actionTypeToString(current_action_->getType()) : "idle";
        if (drop_active_)
        {
            status.action_name = "drop";
        }
        return status;
    }

    void controlLoop()
    {
        if (!current_pose_received_)
        {
            sendDummyPose();
            return;
        }

        if (!current_state_.connected)
        {
            sendPositionSetpoint(current_pose_);
            return;
        }

        const bool executing_land =
            current_action_ &&
            current_action_->getType() == ActionType::LAND;
        if (!current_state_.armed && !executing_land)
        {
            sendPositionSetpoint(current_pose_);
            return;
        }

        std_msgs::msg::String step_msg;
        step_msg.data = std::to_string(action_id_);
        step_pub_->publish(step_msg);

        if (!current_action_ && !action_queue_.empty())
        {
            current_action_ = action_queue_.front();
            action_queue_.pop();
            action_id_++;
            current_action_->setStatus(ActionStatus::EXECUTING);
            current_action_->setStartTime(node_->now());
            force_status_publish_ = true;
            RCLCPP_INFO(node_->get_logger(), "开始执行动作，动作类型编号：%d", static_cast<int>(current_action_->getType()));
        }

        if (current_action_)
        {
            executeAction(current_action_);
        }
    }

private:
    Eigen::Vector3d bodyVectorToEnu(const Eigen::Vector3d &body_vec) const
    {
        Eigen::Quaterniond q_current(
            current_pose_.pose.orientation.w,
            current_pose_.pose.orientation.x,
            current_pose_.pose.orientation.y,
            current_pose_.pose.orientation.z);
        return q_current * body_vec;
    }

    void resetActionRuntimeState()
    {
        visual_servo_controller_.reset();
        visual_servo_action_initialized_ = false;
        drop_active_ = false;
        drop_started_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        resetMoveRuntimeState();
        land_mode_request_sent_ = false;
        land_setpoint_quiet_started_ = false;
        land_low_altitude_count_ = 0;
        land_setpoint_quiet_start_time_ =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    }


    void completeCurrentAction(const std::string &message)
    {
        if (current_action_ && current_action_->getType() == ActionType::HOVER)
        {
            publishVisionHoverActive(false);
        }
        if (current_action_)
        {
            current_action_->setStatus(ActionStatus::COMPLETED);
        }
        last_finish_pose_ = current_pose_;
        current_action_.reset();
        resetActionRuntimeState();
        broadcastStatus(message);
    }

    void failCurrentAction(const std::string &message)
    {
        if (current_action_ && current_action_->getType() == ActionType::HOVER)
        {
            publishVisionHoverActive(false);
        }
        if (current_action_)
        {
            current_action_->setStatus(ActionStatus::FAILED);
        }
        current_action_.reset();
        resetActionRuntimeState();
        RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    }

    double getYawDeg(const geometry_msgs::msg::PoseStamped &pose) const
    {
        return getPoseYawRad(pose) * 57.29577951308232;
    }

    double getPoseYawRad(const geometry_msgs::msg::PoseStamped &pose) const
    {
        // move 平滑控制内部统一使用弧度制 yaw，避免角度/弧度混用。
        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::msg::Quaternion makeQuaternionFromYaw(double yaw_rad) const
    {
        // /mavros/setpoint_position/local 仍然发 PoseStamped，因此需要把目标 yaw 重新封装成四元数。
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        q.normalize();
        return tf2::toMsg(q);
    }

    double normalizeAngleRad(double angle_rad) const
    {
        // 把任意角度误差折叠回 [-pi, pi]，保证 yaw 总是走最短角路径。
        return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
    }

    double stepTowardYawRad(
        double current_yaw_rad,
        double target_yaw_rad,
        double max_step_rad) const
    {
        // yaw 不直接跳到终点，而是按本周期允许的最大角步长逼近目标。
        const double yaw_error = normalizeAngleRad(target_yaw_rad - current_yaw_rad);
        if (std::abs(yaw_error) <= max_step_rad)
        {
            return target_yaw_rad;
        }

        return normalizeAngleRad(
            current_yaw_rad + std::copysign(max_step_rad, yaw_error));
    }

    Eigen::Vector3d stepTowardPosition(
        const Eigen::Vector3d &from,
        const Eigen::Vector3d &to,
        double max_xy_step,
        double max_z_step) const
    {
        // 位置平滑推进分成水平和竖直两部分限速：
        // xy 用平面距离限速，z 单独限速，便于室内把升降速度设得更保守。
        Eigen::Vector3d result = from;

        const Eigen::Vector2d xy_delta = to.head<2>() - from.head<2>();
        const double xy_distance = xy_delta.norm();
        if (xy_distance <= max_xy_step || max_xy_step <= 0.0)
        {
            result.x() = to.x();
            result.y() = to.y();
        }
        else
        {
            const Eigen::Vector2d xy_step = xy_delta.normalized() * max_xy_step;
            result.x() += xy_step.x();
            result.y() += xy_step.y();
        }

        const double z_delta = to.z() - from.z();
        if (std::abs(z_delta) <= max_z_step || max_z_step <= 0.0)
        {
            result.z() = to.z();
        }
        else
        {
            result.z() += std::copysign(max_z_step, z_delta);
        }

        return result;
    }

    void resetMoveRuntimeState()
    {
        move_runtime_.initialized = false;
        move_runtime_.start_pose = geometry_msgs::msg::PoseStamped{};
        move_runtime_.resolved_target_pose = geometry_msgs::msg::PoseStamped{};
        move_runtime_.last_command_pose = geometry_msgs::msg::PoseStamped{};
        move_runtime_.last_update_time =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        move_runtime_.start_yaw_rad = 0.0;
        move_runtime_.target_yaw_rad = 0.0;
        move_runtime_.last_command_yaw_rad = 0.0;
        move_runtime_.total_distance_m = 0.0;
        move_runtime_.stable_count = 0;
    }

    bool resolveMoveTargetPoseOnce(
        const std::shared_ptr<DroneAction> &action,
        geometry_msgs::msg::PoseStamped &resolved_target_pose)
    {
        // move 动作一开始就把目标冻结成固定 world_enu 终点。
        // 这样后续即使机体继续转向，BODY / WORLD_BODY 目标也不会在执行过程中漂移。
        resolved_target_pose = action->getTargetPose();

        if (action->getFrame() == DroneAction::Frame::WORLD_BODY)
        {
            try
            {
                resolved_target_pose = tf_buffer_.transform(resolved_target_pose, "world_enu");
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(node_->get_logger(), "坐标变换失败：%s，已保持当前位置。", ex.what());
                return false;
            }
        }
        else if (action->getFrame() == DroneAction::Frame::BODY)
        {
            // BODY 模式下，位置和 yaw 都解释为“相对上一动作结束姿态”的增量。
            const Eigen::Vector3d delta = bodyVectorToEnu(
                Eigen::Vector3d(
                    resolved_target_pose.pose.position.x,
                    resolved_target_pose.pose.position.y,
                    resolved_target_pose.pose.position.z));
            const double target_yaw_delta = getPoseYawRad(resolved_target_pose);

            resolved_target_pose = last_finish_pose_;
            resolved_target_pose.pose.position.x += delta.x();
            resolved_target_pose.pose.position.y += delta.y();
            resolved_target_pose.pose.position.z += delta.z();
            resolved_target_pose.pose.orientation =
                makeQuaternionFromYaw(normalizeAngleRad(getPoseYawRad(last_finish_pose_) + target_yaw_delta));
        }

        if (resolved_target_pose.header.frame_id.empty())
        {
            resolved_target_pose.header.frame_id = "world_enu";
        }

        return true;
    }

    bool initializeMoveRuntime(const std::shared_ptr<DroneAction> &action)
    {
        // 首次进入 move 时初始化运行时状态。
        // 这里缓存的是“平滑轨迹状态”，而不是仅缓存最终目标点。
        geometry_msgs::msg::PoseStamped resolved_target_pose;
        if (!resolveMoveTargetPoseOnce(action, resolved_target_pose))
        {
            return false;
        }

        move_runtime_.initialized = true;
        move_runtime_.start_pose = current_pose_;
        move_runtime_.resolved_target_pose = resolved_target_pose;
        move_runtime_.last_command_pose = current_pose_;
        move_runtime_.last_command_pose.header.frame_id = "world_enu";
        move_runtime_.last_update_time = node_->now();
        move_runtime_.start_yaw_rad = getPoseYawRad(current_pose_);
        move_runtime_.target_yaw_rad = getPoseYawRad(resolved_target_pose);
        move_runtime_.last_command_yaw_rad = move_runtime_.start_yaw_rad;
        move_runtime_.total_distance_m =
            SpatialPoint(current_pose_).distance(SpatialPoint(resolved_target_pose));
        move_runtime_.stable_count = 0;
        return true;
    }

    geometry_msgs::msg::PoseStamped buildNextMoveSetpoint(
        const std::shared_ptr<DroneAction> &action,
        double dt)
    {
        // 参考轨迹生成策略：
        // 从“上一次发出的命令位姿”出发，朝最终目标推进一个受限步长。
        // 这样 setpoint 本身是连续变化的，而不是每周期都跳到最终终点。
        geometry_msgs::msg::PoseStamped next_setpoint = move_runtime_.last_command_pose;

        const Eigen::Vector3d from(
            move_runtime_.last_command_pose.pose.position.x,
            move_runtime_.last_command_pose.pose.position.y,
            move_runtime_.last_command_pose.pose.position.z);
        const Eigen::Vector3d to(
            move_runtime_.resolved_target_pose.pose.position.x,
            move_runtime_.resolved_target_pose.pose.position.y,
            move_runtime_.resolved_target_pose.pose.position.z);

        const Eigen::Vector3d next_position = stepTowardPosition(
            from,
            to,
            action->getMoveMaxXYSpeed() * dt,
            action->getMoveMaxZSpeed() * dt);

        next_setpoint.header.frame_id = "world_enu";
        next_setpoint.pose.position.x = next_position.x();
        next_setpoint.pose.position.y = next_position.y();
        next_setpoint.pose.position.z = next_position.z();

        move_runtime_.last_command_yaw_rad = stepTowardYawRad(
            move_runtime_.last_command_yaw_rad,
            move_runtime_.target_yaw_rad,
            action->getMoveMaxYawRateRadps() * dt);
        next_setpoint.pose.orientation =
            makeQuaternionFromYaw(move_runtime_.last_command_yaw_rad);

        return next_setpoint;
    }

    bool isMoveGoalReached(
        const std::shared_ptr<DroneAction> &action) const
    {
        // move 完成条件同时看位置和 yaw，避免“人到位了但机头还在转”。
        const SpatialPoint current(current_pose_);
        const SpatialPoint target(move_runtime_.resolved_target_pose);
        const double position_error = current.distance(target);
        const double yaw_error = std::abs(normalizeAngleRad(
            move_runtime_.target_yaw_rad - getPoseYawRad(current_pose_)));

        return position_error < action->getPositionTolerance() &&
               yaw_error < action->getYawToleranceRad();
    }

    std::string formatPose(const geometry_msgs::msg::PoseStamped &pose) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "x=" << pose.pose.position.x << " m, "
            << "y=" << pose.pose.position.y << " m, "
            << "z=" << pose.pose.position.z << " m, "
            << "yaw=" << getYawDeg(pose) << " deg";
        return oss.str();
    }

    bool transformToWorldBody(const geometry_msgs::msg::PoseStamped &source,
                              geometry_msgs::msg::PoseStamped &target)
    {
        try
        {
            target = tf_buffer_.transform(source, "world_body");
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "状态播报坐标变换失败：%s，暂时使用 ENU 坐标。",
                                 ex.what());
            target = source;
            return false;
        }
    }

    std::string formatStatusPose(
        const geometry_msgs::msg::PoseStamped &pose,
        const char *fallback_frame_text,
        bool &using_world_body)
    {
        geometry_msgs::msg::PoseStamped world_body_pose;
        if (transformToWorldBody(pose, world_body_pose))
        {
            using_world_body = true;
            return formatPose(world_body_pose);
        }

        using_world_body = false;
        return std::string(fallback_frame_text) + " " + formatPose(world_body_pose);
    }

    std::string formatSeconds(double seconds) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << seconds;
        return oss.str();
    }

    std::string formatMeters(double meters) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << meters;
        return oss.str();
    }

    void publishStatus(const std::string &message)
    {
        std_msgs::msg::String status_msg;
        status_msg.data = message;
        mission_status_pub_->publish(status_msg);
    }

    void broadcastStatus(const std::string &message)
    {
        publishStatus(message);
        RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    }

    void broadcastStatusThrottled(const std::string &message)
    {
        const rclcpp::Time now = node_->now();
        const bool should_publish =
            force_status_publish_ ||
            last_status_publish_time_.nanoseconds() == 0 ||
            (now - last_status_publish_time_).seconds() >= status_publish_period_s_;

        if (!should_publish)
        {
            return;
        }

        last_status_publish_time_ = now;
        force_status_publish_ = false;
        broadcastStatus(message);
    }

    void executeAction(const std::shared_ptr<DroneAction> &action)
    {
        switch (action->getType())
        {
        case ActionType::MOVE_TO_POSITION:
            executeMoveToPosition(action);
            break;
        case ActionType::HOVER:
            executeHover(action);
            break;
        case ActionType::VISUAL_SERVO:
            executeVisualServo(action);
            break;
        case ActionType::LAND:
            executeLand(action);
            break;
        case ActionType::TAKEOFF:
            executeTakeoff(action);
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "未知动作类型，已保持当前位置。");
            sendPositionSetpoint(current_pose_);
            break;
        }
    }

    void executeMoveToPosition(const std::shared_ptr<DroneAction> &action)
    {
        if (!move_runtime_.initialized)
        {
            if (!initializeMoveRuntime(action))
            {
                sendPositionSetpoint(current_pose_);
                return;
            }
        }

        const rclcpp::Time now = node_->now();
        double dt = (now - move_runtime_.last_update_time).seconds();
        move_runtime_.last_update_time = now;
        if (dt <= 0.0)
        {
            // 首帧或时钟异常时给一个保底周期，避免本周期步长为 0。
            dt = 0.02;
        }

        // 根据当前平滑轨迹状态生成下一帧 setpoint。
        const geometry_msgs::msg::PoseStamped next_setpoint =
            buildNextMoveSetpoint(action, dt);
        move_runtime_.last_command_pose = next_setpoint;
        sendPositionSetpoint(next_setpoint);

        if (isMoveGoalReached(action))
        {
            // 连续满足阈值才完成，降低位置估计轻微抖动带来的误判。
            move_runtime_.stable_count++;
            if (move_runtime_.stable_count > 20)
            {
                completeCurrentAction("已平滑到达目标位置与目标朝向。");
                return;
            }
        }
        else
        {
            move_runtime_.stable_count = 0;
        }

        const SpatialPoint current(current_pose_);
        const SpatialPoint target_point(move_runtime_.resolved_target_pose);
        const double distance_to_target = current.distance(target_point);
        const double yaw_error_deg = std::abs(normalizeAngleRad(
            move_runtime_.target_yaw_rad - getPoseYawRad(current_pose_))) * 57.29577951308232;
        bool current_uses_world_body = false;
        bool target_uses_world_body = false;
        const std::string current_text =
            formatStatusPose(current_pose_, "ENU", current_uses_world_body);
        const std::string target_text =
            formatStatusPose(move_runtime_.resolved_target_pose, "ENU", target_uses_world_body);
        const std::string frame_text =
            current_uses_world_body && target_uses_world_body
                ? "初始点坐标系 world_body"
                : "坐标变换未就绪，混合坐标";

        broadcastStatusThrottled(
            "航点飞行中（" + frame_text + "）：当前 " + current_text +
            "，目标 " + target_text +
            "，剩余距离=" + formatMeters(distance_to_target) +
            " m，剩余偏航=" + formatSeconds(yaw_error_deg) + " deg。");
    }

    void executeHover(const std::shared_ptr<DroneAction> &action)
    {
        publishVisionHoverActive(action->shouldNotifyVisionHover());
        sendPositionSetpoint(last_finish_pose_);
        const double elapsed = (node_->now() - action->getStartTime()).seconds();
        const double remaining = std::max(0.0, action->getHoverTime() - elapsed);
        if (elapsed > action->getHoverTime())
        {
            completeCurrentAction("悬停动作已完成。");
            return;
        }

        bool hold_uses_world_body = false;
        const std::string hold_text =
            formatStatusPose(last_finish_pose_, "ENU", hold_uses_world_body);
        const std::string frame_text =
            hold_uses_world_body ? "初始点坐标系 world_body"
                                 : "坐标变换未就绪，ENU";

        broadcastStatusThrottled(
            "悬停中（" + frame_text + "）：保持位置 " + hold_text +
            "，已悬停=" + formatSeconds(elapsed) + " s，剩余=" +
            formatSeconds(remaining) + " s。");
    }

    void publishVisionHoverActive(bool active)
    {
        if (vision_hover_active_published_ && vision_hover_active_ == active)
        {
            return;
        }

        std_msgs::msg::Bool msg;
        msg.data = active;
        hover_active_pub_->publish(msg);
        vision_hover_active_ = active;
        vision_hover_active_published_ = true;
    }

    void executeVisualServo(const std::shared_ptr<DroneAction> &action)
    {
        const rclcpp::Time now = node_->now();

        if (drop_active_)
        {
            sendPositionSetpoint(visual_servo_hold_pose_);
            if ((now - drop_started_time_).seconds() >= kDropHoldDurationS)
            {
                drop_active_ = false;
                replaceRemainingActionsWithReturnHomeAndLand();
                completeCurrentAction("视觉伺服完成后已执行舵机抛投。");
            }
            return;
        }

        if (!visual_servo_action_initialized_)
        {
            visual_servo_controller_.start(action->getVisualServoConfig(), now);
            visual_servo_action_initialized_ = true;
            visual_servo_hold_pose_ = current_pose_;
            last_visual_servo_state_ = VisualServoState::IDLE;
            force_visual_servo_status_publish_ = true;
        }

        const VisualServoObservation *observation =
            visual_servo_target_received_ ? &latest_visual_servo_target_ : nullptr;
        const VisualServoOutput output =
            visual_servo_controller_.update(observation, now);

        if (output.state == VisualServoState::SUCCEEDED)
        {
            publishVisualServoStatus(output, false, true);
            visual_servo_hold_pose_ = current_pose_;
            if (!startDropServo())
            {
                completeCurrentAction("视觉伺服已完成，但舵机抛投启动失败。");
                return;
            }
            drop_active_ = true;
            drop_started_time_ = now;
            return;
        }

        if (output.state == VisualServoState::TIMED_OUT)
        {
            publishVisualServoStatus(output, false, true);
            const std::string message = "视觉伺服超时：" + output.detail;
            if (action->shouldContinueOnVisualServoTimeout())
            {
                completeCurrentAction(message + "，按配置继续后续任务。");
            }
            else
            {
                failCurrentAction(message);
            }
            return;
        }

        if (output.state == VisualServoState::TRACKING)
        {
            const Eigen::Vector3d enu_delta = bodyVectorToEnu(output.body_delta);
            geometry_msgs::msg::PoseStamped target_pose = current_pose_;
            target_pose.pose.position.x += enu_delta.x();
            target_pose.pose.position.y += enu_delta.y();
            target_pose.pose.position.z += enu_delta.z();
            visual_servo_hold_pose_ = target_pose;
            sendPositionSetpoint(target_pose);
        }
        else
        {
            if (output.state == VisualServoState::ALIGNED &&
                last_visual_servo_state_ != VisualServoState::ALIGNED)
            {
                visual_servo_hold_pose_ = current_pose_;
            }
            sendPositionSetpoint(visual_servo_hold_pose_);
        }

        publishVisualServoStatus(output, true, false);
        last_visual_servo_state_ = output.state;
        broadcastStatusThrottled(
            "视觉伺服：state=" +
            std::string(VisualServoController::stateName(output.state)) +
            "，offset_m=(" + formatSeconds(output.filtered_error_x) + ", " +
            formatSeconds(output.filtered_error_y) + ")，" + output.detail + "。");
    }

    bool writePwmValue(const std::string &path, const std::string &value)
    {
        std::ofstream file(path);
        if (!file)
        {
            RCLCPP_ERROR(node_->get_logger(), "无法打开 PWM 文件：%s", path.c_str());
            return false;
        }
        file << value;
        file.flush();
        if (!file)
        {
            RCLCPP_ERROR(node_->get_logger(), "无法写入 PWM 文件：%s", path.c_str());
            return false;
        }
        return true;
    }

    bool startDropServo()
    {
        constexpr const char *kPwmChipPath = "/sys/class/pwm/pwmchip0";
        constexpr const char *kPwmPath = "/sys/class/pwm/pwmchip0/pwm0";
        constexpr const char *kPeriodNs = "20000000";
        constexpr const char *kReleaseDutyNs = "1000000";

        if (!std::filesystem::exists(kPwmPath) &&
            !writePwmValue(std::string(kPwmChipPath) + "/export", "0"))
        {
            return false;
        }

        if (!writePwmValue(std::string(kPwmPath) + "/enable", "0") ||
            !writePwmValue(std::string(kPwmPath) + "/polarity", "normal") ||
            !writePwmValue(std::string(kPwmPath) + "/period", kPeriodNs) ||
            !writePwmValue(std::string(kPwmPath) + "/duty_cycle", kReleaseDutyNs) ||
            !writePwmValue(std::string(kPwmPath) + "/enable", "1"))
        {
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "舵机抛投已触发：duty_cycle=%s ns。", kReleaseDutyNs);
        return true;
    }

    void replaceRemainingActionsWithReturnHomeAndLand()
    {
        while (!action_queue_.empty())
        {
            action_queue_.pop();
        }

        geometry_msgs::msg::PoseStamped return_pose;
        return_pose.header.frame_id = "world_body";
        return_pose.pose.position.z = 1.5;
        return_pose.pose.orientation.w = 1.0;

        constexpr double kDegToRad = M_PI / 180.0;
        action_queue_.push(DroneAction::createMoveToAction(
            return_pose,
            DroneAction::Frame::WORLD_BODY,
            0.10,
            4.0 * kDegToRad,
            0.50,
            0.30,
            40.0 * kDegToRad));
        action_queue_.push(DroneAction::createLandAction());

        RCLCPP_INFO(
            node_->get_logger(),
            "抛投完成，已清空后续航点并切换为返航、恢复初始航向和降落。");
    }

    void publishVisualServoStatus(
        const VisualServoOutput &output, bool active, bool force)
    {
        const rclcpp::Time now = node_->now();
        const std::string state_name =
            VisualServoController::stateName(output.state);
        const bool state_changed = state_name != last_visual_servo_status_name_;
        const bool period_elapsed =
            last_visual_servo_status_time_.nanoseconds() == 0 ||
            (now - last_visual_servo_status_time_).seconds() >= 0.1;
        if (!force && !force_visual_servo_status_publish_ &&
            !state_changed && !period_elapsed)
        {
            return;
        }

        drone_msgs::msg::VisionServoStatus msg;
        msg.stamp = now;
        msg.active = active;
        msg.state = state_name;
        if (current_action_ &&
            current_action_->getType() == ActionType::VISUAL_SERVO)
        {
            msg.requested_target_id =
                current_action_->getVisualServoConfig().target_id;
        }
        msg.tracked_target_id = visual_servo_controller_.lockedTargetId();
        msg.target_sequence = latest_visual_servo_target_.sequence;
        msg.target_visible = output.target_visible;
        msg.aligned = output.aligned || output.state == VisualServoState::SUCCEEDED;
        msg.filtered_error_x = output.filtered_error_x;
        msg.filtered_error_y = output.filtered_error_y;
        msg.detail = output.detail;
        visual_servo_status_pub_->publish(msg);

        last_visual_servo_status_name_ = state_name;
        last_visual_servo_status_time_ = now;
        force_visual_servo_status_publish_ = false;
    }

    void executeLand(const std::shared_ptr<DroneAction> &action)
    {
        if (!action->isStartPoseInitialized())
        {
            action->setStartPose(current_pose_);
            force_status_publish_ = true;
        }

        if (!land_setpoint_quiet_started_)
        {
            land_setpoint_quiet_started_ = true;
            land_setpoint_quiet_start_time_ = node_->now();
            force_status_publish_ = true;
            broadcastStatus("降落接管开始：已停止发送位置 setpoint，准备切换 AUTO.LAND。");
            return;
        }

        if (!land_mode_request_sent_)
        {
            const double quiet_elapsed =
                (node_->now() - land_setpoint_quiet_start_time_).seconds();
            if (quiet_elapsed < land_setpoint_quiet_time_s_)
            {
                broadcastStatusThrottled(
                    "降落接管静默中：已停止位置 setpoint，等待 " +
                    formatSeconds(land_setpoint_quiet_time_s_ - quiet_elapsed) +
                    " s 后请求 AUTO.LAND。");
                return;
            }

            if (!callSetMode("AUTO.LAND"))
            {
                broadcastStatusThrottled(
                    "AUTO.LAND 切换失败，保持 setpoint 静默并重试：当前高度（ENU z）=" +
                    formatMeters(current_pose_.pose.position.z) + " m，当前模式=" +
                    current_state_.mode + "。");
                return;
            }
            land_mode_request_sent_ = true;
            force_status_publish_ = true;
            RCLCPP_INFO(node_->get_logger(), "降落模式切换请求已发送。");
        }

        const double current_z = current_pose_.pose.position.z;
        const double start_z = action->getStartPose().pose.position.z;
        const double descended = std::max(0.0, start_z - current_z);
        constexpr double kLandCompleteAltitudeM = 0.15;
        constexpr int kLandCompleteStableCycles = 20;

        if (current_z <= kLandCompleteAltitudeM)
        {
            ++land_low_altitude_count_;
        }
        else
        {
            land_low_altitude_count_ = 0;
        }

        if (!current_state_.armed ||
            land_low_altitude_count_ >= kLandCompleteStableCycles)
        {
            const std::string reason =
                !current_state_.armed ? "飞控已上锁" : "高度达到近地阈值";
            completeCurrentAction(
                "降落动作已完成（" + reason + "）：当前高度（ENU z）=" +
                formatMeters(current_z) + " m，相对降落开始已下降=" +
                formatMeters(descended) + " m。");
            return;
        }

        broadcastStatusThrottled(
            "降落中：当前高度（ENU z）=" + formatMeters(current_z) +
            " m，相对降落开始已下降=" + formatMeters(descended) +
            " m，飞控模式=" + current_state_.mode + "。");
    }

    void executeTakeoff(const std::shared_ptr<DroneAction> &action)
    {
        geometry_msgs::msg::PoseStamped takeoff_pose;
        if (action->isStartPoseInitialized())
        {
            takeoff_pose = action->getStartPose();
        }
        else
        {
            action->setStartPose(current_pose_);
            takeoff_pose = current_pose_;
        }

        takeoff_pose.header.frame_id = "world_enu";
        takeoff_pose.header.stamp = node_->now();
        const double target_z =
            action->getStartPose().pose.position.z + action->getTargetAltitude();
        takeoff_pose.pose.position.z = target_z;

        const SpatialPoint current(current_pose_);
        const SpatialPoint target(takeoff_pose);
        if (std::abs(current.z - target.z) < action->getPositionTolerance())
        {
            completeCurrentAction("起飞动作已完成。");
            return;
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "正在执行起飞：当前 ENU z %.2f m，目标 ENU z %.2f m，相对起飞高度 %.2f m。",
                             current.z, target.z, action->getTargetAltitude());
        sendPositionSetpoint(takeoff_pose);
    }

    bool callSetMode(const std::string &mode)
    {
        if (!set_mode_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "飞行模式切换服务不可用。");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        auto future = set_mode_client_->async_send_request(request);
        const auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2));
        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(node_->get_logger(), "飞行模式切换请求超时，目标模式：%s。", mode.c_str());
            return false;
        }
        const bool mode_sent = future.get()->mode_sent;
        if (!mode_sent)
        {
            RCLCPP_WARN(node_->get_logger(), "飞控拒绝切换飞行模式，目标模式：%s。", mode.c_str());
        }
        return mode_sent;
    }

    void state_callback(mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_.header.frame_id = "world_enu";
        current_pose_received_ = true;
        if (!last_finish_pose_initialized_)
        {
            last_finish_pose_ = current_pose_;
            last_finish_pose_initialized_ = true;
        }
    }

    void visualServoTargetCallback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!msg || !std::isfinite(msg->point.x) ||
            !std::isfinite(msg->point.y) || !std::isfinite(msg->point.z))
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "已忽略包含非法米制坐标的视觉伺服目标消息。");
            return;
        }

        // The vision node publishes camera optical-frame offsets in metres:
        // x points right and y points down. Axis/sign mapping stays configurable.
        ++visual_servo_target_sequence_;
        latest_visual_servo_target_.sequence = visual_servo_target_sequence_;
        latest_visual_servo_target_.target_id.clear();
        latest_visual_servo_target_.valid = true;
        latest_visual_servo_target_.confirmed = true;
        latest_visual_servo_target_.error_x = msg->point.x;
        latest_visual_servo_target_.error_y = msg->point.y;
        latest_visual_servo_target_.received_time = node_->now();
        visual_servo_target_received_ = true;
    }

    std::string actionTypeToString(ActionType type) const
    {
        switch (type)
        {
        case ActionType::MOVE_TO_POSITION:
            return "move";
        case ActionType::HOVER:
            return "hover";
        case ActionType::VISUAL_SERVO:
            return "visual_servo";
        case ActionType::LAND:
            return "land";
        case ActionType::TAKEOFF:
            return "takeoff";
        default:
            return "unknown";
        }
    }

    //- 进入 `move` 第一帧时初始化
    //- 记录起点、解析终点、提取起始 yaw 和目标 yaw
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer &tf_buffer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr step_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hover_active_pub_;
    rclcpp::Publisher<drone_msgs::msg::VisionServoStatus>::SharedPtr visual_servo_status_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr visual_servo_target_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_ = false;
    rclcpp::Time last_status_publish_time_;
    bool force_status_publish_ = true;
    double status_publish_period_s_ = 1.0;
    double land_setpoint_quiet_time_s_ = 0.2;
    bool vision_hover_active_ = false;
    bool vision_hover_active_published_ = false;

    bool land_mode_request_sent_ = false;
    bool land_setpoint_quiet_started_ = false;
    int land_low_altitude_count_ = 0;
    rclcpp::Time land_setpoint_quiet_start_time_;

    std::queue<std::shared_ptr<DroneAction>> action_queue_;
    std::shared_ptr<DroneAction> current_action_;

    geometry_msgs::msg::PoseStamped last_finish_pose_;
    bool last_finish_pose_initialized_ = false;

    int action_id_ = 0;
    MoveRuntimeState move_runtime_;
    VisualServoController visual_servo_controller_;
    VisualServoObservation latest_visual_servo_target_;
    geometry_msgs::msg::PoseStamped visual_servo_hold_pose_;
    VisualServoState last_visual_servo_state_ = VisualServoState::IDLE;
    bool visual_servo_target_received_ = false;
    uint32_t visual_servo_target_sequence_ = 0;
    bool visual_servo_action_initialized_ = false;
    bool drop_active_ = false;
    rclcpp::Time drop_started_time_;
    static constexpr double kDropHoldDurationS = 1.0;
    bool force_visual_servo_status_publish_ = true;
    std::string last_visual_servo_status_name_;
    rclcpp::Time last_visual_servo_status_time_;
};

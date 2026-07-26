#pragma once

// 引入 ROS 2 和几何消息库
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <memory>

#include "drone_control/visual_servo_controller.hpp"

namespace offboard_run {
    
    // 定义一个空间点类，表示无人机在3D空间中的位置
    struct SpatialPoint {
        double x;  // x坐标
        double y;  // y坐标
        double z;  // z坐标

        // 构造函数，默认为原点(0, 0, 0)
        SpatialPoint(double x = 0.0, double y = 0.0, double z = 0.0)
            : x(x), y(y), z(z) {}

        // 从PoseStamped对象构造SpatialPoint
        explicit SpatialPoint(const geometry_msgs::msg::PoseStamped &pose)
            : x(pose.pose.position.x),
              y(pose.pose.position.y),
              z(pose.pose.position.z) {}

        // 从Point对象构造SpatialPoint
        explicit SpatialPoint(const geometry_msgs::msg::Point &point)
            : x(point.x), y(point.y), z(point.z) {}

        // 计算与另一个SpatialPoint的3D距离
        double distance(const SpatialPoint &other) const {
            return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) +
                             std::pow(z - other.z, 2));
        }

        // 计算忽略z轴的2D距离
        double distanceXY(const SpatialPoint &other) const {
            return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
        }
    };

    // 定义动作类型的枚举
    enum class ActionType {
        MOVE_TO_POSITION = 0,  // 移动到目标位置
        HOVER,                 // 悬停
        VISUAL_SERVO,          // 通用单目标视觉伺服
        LAND,                  // 着陆
        TAKEOFF,               // 起飞
    };

    // 定义动作状态的枚举
    enum class ActionStatus {
        PENDING,    // 等待执行
        EXECUTING,  // 执行中
        COMPLETED,  // 执行完成
        FAILED,     // 执行失败
        ABORTED,    // 执行中止
    };

    // DroneAction 类，用于定义和管理无人机动作
    class DroneAction {
    private:
        struct PrivateTag{};  // 私有标签，用于控制 DroneAction 的构造

    public:
        explicit DroneAction(PrivateTag) {}

        // 枚举：参考坐标系
        enum class Frame {
            BODY = 0,    // 无人机体坐标系
            WORLD_BODY,  // 世界坐标系（相对于无人机的当前位置）
            WORLD_ENU,   // 世界坐标系（相对于地理参考系）
        };

        // 工厂方法，创建 "移动到目标位置" 的动作
        static std::shared_ptr<DroneAction> createMoveToAction(
            const geometry_msgs::msg::PoseStamped &target_pose,
            Frame frame = Frame::WORLD_BODY,
            double position_tolerance = 0.1,
            double yaw_tolerance_rad = 5.0 * M_PI / 180.0,
            double move_max_xy_speed_mps = 0.35,
            double move_max_z_speed_mps = 0.20,
            double move_max_yaw_rate_radps = 30.0 * M_PI / 180.0)
        {
            auto action = std::make_shared<DroneAction>(PrivateTag{});
            action->type_ = ActionType::MOVE_TO_POSITION;     // 设置动作类型
            action->target_pose_ = target_pose;               // 设置目标位置
            action->frame_ = frame;                           // 设置参考坐标系
            action->position_tolerance_ = position_tolerance; // 设置位置公差
            action->yaw_tolerance_rad_ = yaw_tolerance_rad;   // 设置 yaw 到达容差
            action->move_max_xy_speed_mps_ = move_max_xy_speed_mps;
            action->move_max_z_speed_mps_ = move_max_z_speed_mps;
            action->move_max_yaw_rate_radps_ = move_max_yaw_rate_radps;
            return action;
        }

        // 工厂方法，创建 "悬停" 的动作
        static std::shared_ptr<DroneAction> createHoverAction(
            double hover_time_s,
            bool notify_vision_hover = false) {
            auto action = std::make_shared<DroneAction>(PrivateTag{});
            action->type_ = ActionType::HOVER;  // 设置动作类型为悬停
            action->hover_time_s_ = hover_time_s;  // 设置悬停时间
            action->notify_vision_hover_ = notify_vision_hover;
            return action;
        }

        static std::shared_ptr<DroneAction> createVisualServoAction(
            const VisualServoConfig &config,
            bool continue_on_timeout = true) {
            auto action = std::make_shared<DroneAction>(PrivateTag{});
            action->type_ = ActionType::VISUAL_SERVO;
            action->visual_servo_config_ = config;
            action->visual_servo_continue_on_timeout_ = continue_on_timeout;
            return action;
        }

        // 工厂方法，创建 "着陆" 的动作
        static std::shared_ptr<DroneAction> createLandAction(){
            auto action = std::make_shared<DroneAction>(PrivateTag{});
            action->type_ = ActionType::LAND;  // 设置动作类型为着陆
            return action;
        }

        // 工厂方法，创建 "起飞" 的动作
        static std::shared_ptr<DroneAction> createTakeoffAction(
            double target_altitude, double tolerance = 0.1){
                auto action = std::make_shared<DroneAction>(PrivateTag());
                action->type_ = ActionType::TAKEOFF;  // 设置动作类型为起飞
                action->target_altitude_ = target_altitude;  // 设置目标高度
                action->position_tolerance_ = tolerance;
                return action;
            }

        // 获取动作类型
        ActionType getType() const { return type_; }

        // 获取当前动作状态
        ActionStatus getStatus() const { return status_; }

        // 获取位置公差
        double getPositionTolerance() const { return position_tolerance_; }

        // 获取目标高度
        double getTargetAltitude() const { return target_altitude_; }

        double getHoverTime() const { return hover_time_s_; }
        bool shouldNotifyVisionHover() const { return notify_vision_hover_; }

        geometry_msgs::msg::PoseStamped getTargetPose() const { return target_pose_; }

        Frame getFrame() const { return frame_; }

       

        // 获取动作开始时间
        rclcpp::Time getStartTime() const {return start_time_; }

        // 检查起始位置是否已初始化
        bool isStartPoseInitialized() const { return init_start_pose_; }

        // 获取起始位姿
        geometry_msgs::msg::PoseStamped getStartPose() const {
            return init_start_pose_ ? start_pose_ : geometry_msgs::msg::PoseStamped();
        }

        // 设置动作状态
        void setStatus(ActionStatus status) { status_ = status; }

        // 设置动作开始时间
        void setStartTime(const rclcpp::Time &time) { start_time_ = time; }

        // 设置起始位姿
        void setStartPose(const geometry_msgs::msg::PoseStamped &pose) {
            start_pose_ = pose;
            init_start_pose_ = true;
        }

        // 判断动作是否完成
        bool isCompleted() const { return status_ == ActionStatus::COMPLETED; }

        // 判断动作是否在执行中
        bool isExecuting() const { return status_ == ActionStatus::EXECUTING; }

        // 判断动作是否失败
        bool isFailed() const { return status_ == ActionStatus::FAILED; }

        // 判断动作是否已中止
        bool isAborted() const { return status_ == ActionStatus::ABORTED; }

        double getMoveMaxXYSpeed() const { return move_max_xy_speed_mps_; }
        double getMoveMaxZSpeed() const { return move_max_z_speed_mps_; }
        double getMoveMaxYawRateRadps() const { return move_max_yaw_rate_radps_; }
        double getYawToleranceRad() const { return yaw_tolerance_rad_; }
        const VisualServoConfig &getVisualServoConfig() const {
            return visual_servo_config_;
        }
        bool shouldContinueOnVisualServoTimeout() const {
            return visual_servo_continue_on_timeout_;
        }

    private:
        // 动作类型（如悬停、移动等）
        ActionType type_ = ActionType::HOVER;

        // 动作状态（执行中的状态）
        ActionStatus status_ = ActionStatus::PENDING;

        // 目标位置
        geometry_msgs::msg::PoseStamped target_pose_;
        Frame frame_ = Frame::WORLD_BODY;  // 默认参考系是世界坐标系

        // 悬停时间（秒）
        double hover_time_s_ = 0.0;

        // 该悬停动作是否向视觉端发布任务窗口状态
        bool notify_vision_hover_ = false;

        // 位置公差，用于判断是否到达目标位置
        double position_tolerance_ = 0.1;

        // 默认目标高度
        double target_altitude_ = 1.2;

        // 起始位置
        geometry_msgs::msg::PoseStamped start_pose_;
        bool init_start_pose_ = false;

        // 动作开始时间
        rclcpp::Time start_time_;

        // move 动作在水平面 (x/y) 上的最大平滑推进速度，单位 m/s
        double move_max_xy_speed_mps_ = 0.35;

        // move 动作在竖直方向 (z) 上的最大平滑推进速度，单位 m/s
        double move_max_z_speed_mps_ = 0.20;

        // move 动作的最大偏航角速度，单位 rad/s。
        // 这里用 30.0 * M_PI / 180.0 是把更直观的 30 deg/s 转成弧度制，
        // 因为 C++/ROS/tf2 里的 yaw、三角函数和角度差计算通常都统一使用弧度。
        double move_max_yaw_rate_radps_ = 30.0 * M_PI / 180.0;

        // move 动作的偏航到达容差，单位 rad。
        // 这里用 5.0 * M_PI / 180.0 是把 5 deg 转成弧度制，
        // 表示当前 yaw 与目标 yaw 的误差小于约 5 度时，可认为朝向已经到位。
        double yaw_tolerance_rad_ = 5.0 * M_PI / 180.0;

        VisualServoConfig visual_servo_config_;
        bool visual_servo_continue_on_timeout_ = true;
    };
}

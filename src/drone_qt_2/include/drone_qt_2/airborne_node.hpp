#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <QProcess>

#include "rclcpp/rclcpp.hpp"
#include <mavros_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "drone_qt_2/mission_yaml_builder.hpp"

//自定义消息头文件
#include "drone_msgs/msg/drone_status.hpp"
#include "drone_msgs/srv/start_task.hpp"
#include "drone_msgs/srv/start_offboard.hpp"
#include "drone_msgs/srv/upload_mission_yaml.hpp"
#include "drone_msgs/msg/barcode_capture.hpp"
#include "drone_msgs/msg/ready_status.hpp"
#include "drone_msgs/msg/task_status.hpp"
#include "drone_msgs/msg/world_group.hpp"
#include "drone_msgs/srv/upload_mission_summary.hpp"

class AirborneNode : public rclcpp::Node
{
public:
    AirborneNode();
    ~AirborneNode() override;

private:
    //负责创建发布器、服务和定时器
    void setupInterfaces();

    //定时器回调函数，用于模拟任务进度更新和状态发布
    void onTimer();

    //发布当前状态消息
    void publishStatus();

    //杀死残留的offboard节点，防止重复启动失败
    bool killResidualOffboardNodes(std::string &error_message);

    //处理任务启动服务请求的回调函数
    void handleStartTask(
        const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
        std::shared_ptr<drone_msgs::srv::StartTask::Response> response);

    void handleStopPush(
        const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
        std::shared_ptr<drone_msgs::srv::StartTask::Response> response);

    void handleStartOffboard(
        const std::shared_ptr<drone_msgs::srv::StartOffboard::Request> request,
        std::shared_ptr<drone_msgs::srv::StartOffboard::Response> response);

    void handleUploadMissionSummary(
        const std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Request> request,
        std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Response> response);

    std::string buildStatusText() const;

    //处理条形码捕获消息的回调函数
    void handleBarcodeCapture(
        const drone_msgs::msg::BarcodeCapture::SharedPtr msg);

    //处理shell命令操作
    bool startOffboardCommand();
    bool startTaskCommand();
    bool stopTaskCommand(std::string &error_message);
    bool stopOffboardProcess(std::string &error_message);

    //保存yaml字符串到文件，并返回保存路径和错误信息
    bool saveMissionYamlToFile(const std::string &yaml_text, std::string &saved_path, std::string &error_message);

    rclcpp::Publisher<drone_msgs::msg::DroneStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr barcode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_pub_;
    rclcpp::Publisher<drone_msgs::msg::TaskStatus>::SharedPtr return_status_pub_;
    rclcpp::Publisher<drone_msgs::msg::ReadyStatus>::SharedPtr return_path_ready_pub_;
    rclcpp::Publisher<drone_msgs::msg::WorldGroup>::SharedPtr return_world_group_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr return_delta_pub_;
    rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr vision_barcode_pub_;
    rclcpp::Subscription<drone_msgs::msg::BarcodeCapture>::SharedPtr vision_barcode_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<drone_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
    rclcpp::Subscription<drone_msgs::msg::ReadyStatus>::SharedPtr ready_status_sub_;
    rclcpp::Subscription<drone_msgs::msg::WorldGroup>::SharedPtr return_world_group_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr return_delta_sub_;
    rclcpp::Service<drone_msgs::srv::StartTask>::SharedPtr start_task_srv_;
    rclcpp::Service<drone_msgs::srv::StartTask>::SharedPtr stop_push_srv_;
    rclcpp::Service<drone_msgs::srv::StartOffboard>::SharedPtr start_offboard_srv_;
    rclcpp::Service<drone_msgs::srv::UploadMissionSummary>::SharedPtr upload_mission_summary_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool task_running_{false};//任务是否正在运行
    int task_progress_{0};//任务进度百分比
    std::string current_task_name_;//当前任务名称字符串
    std::string action_name_{""};//当前动作名称字符串

    bool connected{false};//连接状态字符串
    bool armed{false};//解锁状态布尔值
    int8_t flight_mode{0};//飞行模式数值
    float battery_voltage{0.0};//电压数值
    float battery_percent{0.0};//电量百分比数值

    double position_x{0};//无人机位置x坐标
    double position_y{0};//无人机位置y坐标
    double position_z{0};//无人机位置z坐标
    double position_qx{0};//无人机姿态四元数qx
    double position_qy{0};//无人机姿态四元数qy
    double position_qz{0};//无人机姿态四元数qz
    double position_qw{0};//无人机姿态四元数qw

    bool offboard_started_{false};//防止任务重复启动
    bool task_started_{false};
    bool task_stoped_{false};

    QProcess *offboard_process_{nullptr};

    std::string current_mission_path_;//当前任务yaml保存路径字符串
    bool mission_uploaded_{false};//是否已上传任务yaml的标志

    bool unlock_flag_{false};//是否是从开锁到解锁的状态
    bool auto_stop_flag_{false};//判断是否自动执行过stop
    int disarm_stable_count_{0};    // 连续收到 armed == false 的次数

    bool waypoint_or_button_{false};//判断是否是航点上传
};
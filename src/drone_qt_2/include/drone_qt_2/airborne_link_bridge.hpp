#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <deque>

#include <QObject>
#include <QByteArray>
#include <QSerialPort>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include "drone_msgs/msg/drone_status.hpp"
#include "drone_msgs/msg/task_status.hpp"
#include "drone_msgs/msg/ready_status.hpp"
#include "drone_msgs/msg/world_group.hpp"
#include "drone_msgs/msg/barcode_capture.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include "drone_msgs/srv/upload_mission_summary.hpp"
#include "drone_msgs/srv/start_offboard.hpp"
#include "drone_msgs/srv/start_task.hpp"

#include "drone_qt_2/link_protocol.hpp"

class AirborneLinkBridge : public rclcpp::Node
{
    
public:
    AirborneLinkBridge();
    ~AirborneLinkBridge() override = default;

private:
    struct CachedResponse
    {
        uint8_t response_type{0};
        QByteArray payload;
    };

    struct Packet
    {
        uint8_t type{0};
        uint8_t flags{0};
        uint16_t seq{0};
        QByteArray payload;
    };

    //设置ROS接口，包括创建发布者、服务端和订阅者等
    void setupRosInterfaces();
    //设置串口通信参数，并连接相关信号槽
    void setupSerial();


    //串口数据读取回调函数，用于处理接收到的数据
    void onSerialReadyRead();
    //协议帧处理函数，包括尝试解析一帧数据、验证帧的合法性、生成新的序列号等
    bool tryParseOnePacket(Packet &packet);
    //协议帧编码函数，用于将消息类型、标志位、序列号和载荷编码成完整的帧数据
    QByteArray encodeFrame(uint8_t type, uint8_t flags, uint16_t seq, const QByteArray &payload) const;
    //验证函数，用于检查接收到的帧数据是否符合协议格式和校验要求，确保数据的正确性和完整性
    bool validateFrame(const QByteArray &frame) const;

    //协议帧处理函数，根据不同的消息类型调用不同的处理函数
    void handlePacket(const Packet &packet);
    void handleUploadMissionSummaryRequest(uint16_t seq, const QByteArray &payload);
    void handleStartOffboardRequest(uint16_t seq, const QByteArray &payload);
    void handleStartTaskRequest(uint16_t seq, const QByteArray &payload);
    void handleStopPushRequest(uint16_t seq, const QByteArray &payload);

    //发布状态消息到ROS话题
    void publishDroneStatus(const drone_msgs::msg::DroneStatus::SharedPtr msg);
    void publishPathReady(const drone_msgs::msg::ReadyStatus::SharedPtr msg);
    void publishTaskStatus(const drone_msgs::msg::TaskStatus::SharedPtr msg);
    void publishReturnWorldGroup(const drone_msgs::msg::WorldGroup::SharedPtr msg);
    void publishVisionBarcode(const drone_msgs::msg::BarcodeCapture::SharedPtr msg);
    void publishDelta(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void publishLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    //发送ACK帧，包含被确认的序列号
    void sendAck(uint16_t seq);
    //发送响应帧，包含消息类型、序列号和载荷数据
    void sendResponse(uint8_t type, uint16_t seq, const QByteArray &payload);
    //服务任务去重
    void cacheAndSendResponse(uint8_t type, uint16_t seq, const QByteArray &payload);

    rclcpp::Client<drone_msgs::srv::UploadMissionSummary>::SharedPtr upload_mission_summary_client_;
    rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedPtr start_offboard_client_;
    rclcpp::Client<drone_msgs::srv::StartTask>::SharedPtr start_task_client_;
    rclcpp::Client<drone_msgs::srv::StartTask>::SharedPtr stop_push_client_;

    rclcpp::Subscription<drone_msgs::msg::DroneStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<drone_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
    rclcpp::Subscription<drone_msgs::msg::ReadyStatus>::SharedPtr path_ready_sub_;
    rclcpp::Subscription<drone_msgs::msg::WorldGroup>::SharedPtr return_world_group_sub_;
    rclcpp::Subscription<drone_msgs::msg::BarcodeCapture>::SharedPtr vision_barcode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr delta_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;

    QSerialPort serial_;
    QByteArray rx_buffer_;

    std::unordered_map<uint16_t, CachedResponse> completed_requests_;
    std::deque<uint16_t> completed_request_order_;
};
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

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

#include "drone_warehouse/link_protocol.hpp"

class GroundLinkBridge : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    GroundLinkBridge();
    ~GroundLinkBridge() override = default;

private:
    //解码后的协议帧
    struct Packet
    {
        uint8_t type{0};//消息类型
        uint8_t flags{0};//标志位，保留备用
        uint16_t seq{0};//序列号，用于匹配请求和响应
        QByteArray payload;//消息载荷，包含具体的数据内容
    };
    //待发送的请求信息，用于重试机制
    struct PendingRequest
    {
        uint8_t request_type{0};//请求类型
        uint16_t seq{0};//请求的序列号
        int retry_count{0};//重试次数，初始为0，每次重试加1
        QByteArray encoded_frame;//已经编码好的完整帧数据，包含头部、载荷和校验等信息
        rclcpp::Time deadline;//请求的截止时间，超过这个时间还没有收到响应就认为请求失败，需要重试
    };

    struct PendingUploadMissionSummaryCall
    {
        bool done{false};
        bool success{false};
        std::string message;
        std::string saved_path;
        uint32_t action_count{0};
    };

    struct PendingStartOffboardCall
    {
        bool done{false};
        bool success{false};
        std::string message;
    };

    struct PendingStartTaskCall
    {
        bool done{false};
        bool success{false};
        std::string message;
    };

    struct PendingStopPushCall
    {
        bool done{false};
        bool success{false};
        std::string message;
    };

    //设置ROS接口，包括创建发布者、服务端和订阅者等
    void setupRosInterfaces();
    //设置串口通信参数，并连接相关信号槽
    void setupSerial();

    //串口数据读取回调函数，用于处理接收到的数据
    void onSerialReadyRead();
    //定时器回调函数，用于检查待重试的请求是否超时，并进行重试
    void onRetryTimer();

    //协议帧处理函数，包括尝试解析一帧数据、验证帧的合法性、生成新的序列号等
    bool tryParseOnePacket(Packet &packet);
    //编码函数，用于将消息类型、标志位、序列号和载荷数据编码成一个完整的帧数据，准备发送
    QByteArray encodeFrame(uint8_t type, uint8_t flags, uint16_t seq, const QByteArray &payload) const;
    //验证函数，用于检查接收到的帧数据是否符合协议格式和校验要求，确保数据的正确性和完整性
    bool validateFrame(const QByteArray &frame) const;
    //生成函数，用于生成一个新的序列号，确保每个请求都有一个唯一的标识符，便于匹配响应和处理重试逻辑
    uint16_t nextSeq();

    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray encodeUploadMissionSummaryRequest(
        const drone_msgs::srv::UploadMissionSummary::Request &request,
        uint16_t msg_id) const;
    QByteArray encodeStartOffboardRequest(
        const drone_msgs::srv::StartOffboard::Request &request) const;
    QByteArray encodeStartTaskRequest(
        const drone_msgs::srv::StartTask::Request &request) const;
    QByteArray encodeStopPushRequest(
        const drone_msgs::srv::StartTask::Request &request) const;

    //处理函数，用于处理接收到的协议帧，根据帧的类型调用不同的处理逻辑，并发送相应的ACK或响应数据
    void handlePacket(const Packet &packet);
    void handleAck(uint16_t seq);

    void handleDroneStatusReport(const QByteArray &payload);
    void handlePathReadyReport(const QByteArray &payload);
    void handleTaskStatusReport(const QByteArray &payload);
    void handleReturnWorldGroupReport(const QByteArray &payload);
    void handleVisionBarcodeReport(const QByteArray &payload);
    void handleDeltaReport(const QByteArray &payload);
    void handleLocalPositionReport(const QByteArray &payload);
    void handleUploadMissionSummaryResponse(uint16_t seq, const QByteArray &payload);
    void handleStartOffboardResponse(uint16_t seq, const QByteArray &payload);
    void handleStartTaskResponse(uint16_t seq, const QByteArray &payload);
    void handleStopPushResponse(uint16_t seq, const QByteArray &payload);

    //发送函数，用于发送协议帧到串口
    uint16_t sendPacket(uint8_t type, uint8_t flags, const QByteArray &payload, bool need_ack);
    void sendAck(uint16_t seq);

    rclcpp::Service<drone_msgs::srv::UploadMissionSummary>::SharedPtr upload_mission_summary_srv_;
    rclcpp::Service<drone_msgs::srv::StartOffboard>::SharedPtr start_offboard_srv_;
    rclcpp::Service<drone_msgs::srv::StartTask>::SharedPtr start_task_srv_;
    rclcpp::Service<drone_msgs::srv::StartTask>::SharedPtr stop_push_srv_;

    rclcpp::Publisher<drone_msgs::msg::DroneStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<drone_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;
    rclcpp::Publisher<drone_msgs::msg::ReadyStatus>::SharedPtr path_ready_pub_;
    rclcpp::Publisher<drone_msgs::msg::WorldGroup>::SharedPtr return_world_group_pub_;
    rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr vision_barcode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr delta_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_pub_;

    std::unordered_map<uint16_t, PendingUploadMissionSummaryCall> pending_upload_calls_;
    std::unordered_map<uint16_t, PendingStartOffboardCall> pending_start_offboard_calls_;
    std::unordered_map<uint16_t, PendingStartTaskCall> pending_start_task_calls_;
    std::unordered_map<uint16_t, PendingStopPushCall> pending_stop_push_calls_;

    QSerialPort serial_;
    QByteArray rx_buffer_;
    uint16_t next_seq_{1};
    uint16_t next_msg_id_{1};
    std::unordered_map<uint16_t, PendingRequest> pending_requests_;
};
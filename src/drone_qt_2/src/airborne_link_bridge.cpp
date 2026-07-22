#include "drone_qt_2/airborne_link_bridge.hpp"

#include <QDataStream>

namespace lp = drone_msgs::link_protocol;

namespace
{
    uint16_t crc16_ccitt(const uint8_t *data, int length)
    {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
                } else {
                    crc = static_cast<uint16_t>(crc << 1);
                }
            }
        }
        return crc;
    }
}

AirborneLinkBridge::AirborneLinkBridge()
    : rclcpp::Node("airborne_link_bridge")
{
    setupRosInterfaces();
    setupSerial();
}

void AirborneLinkBridge::setupRosInterfaces()
{
    status_sub_ = this->create_subscription<drone_msgs::msg::DroneStatus>(
        "/serial/drone/status",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const drone_msgs::msg::DroneStatus::SharedPtr msg) {
            publishDroneStatus(msg);
        });

    path_ready_sub_ = this->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/drone/control/path_ready",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg) {
            publishPathReady(msg);
        });

    task_status_sub_ = this->create_subscription<drone_msgs::msg::TaskStatus>(
        "/serial/drone/task/status",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg) {
            publishTaskStatus(msg);
        });

    return_world_group_sub_ = this->create_subscription<drone_msgs::msg::WorldGroup>(
        "/drone/return/world_group",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::WorldGroup::SharedPtr msg) {
            publishReturnWorldGroup(msg);
        });

    vision_barcode_sub_ = this->create_subscription<drone_msgs::msg::BarcodeCapture>(
        "/serial/drone/vision/barcode",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::BarcodeCapture::SharedPtr msg) {
            publishVisionBarcode(msg);
        });

    delta_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/serial/drone/pose_yaw_compare/delta",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
            publishDelta(msg);
        });

    upload_mission_summary_client_ = this->create_client<drone_msgs::srv::UploadMissionSummary>(
        "/drone/upload_mission_summary");

    start_offboard_client_ = this->create_client<drone_msgs::srv::StartOffboard>(
        "/drone/start_offboard");

    start_task_client_ = this->create_client<drone_msgs::srv::StartTask>(
        "/drone/start_task");

    stop_push_client_ = this->create_client<drone_msgs::srv::StartTask>(
        "/drone/stop_push");
}

/************************ 用户层 *************************/

void AirborneLinkBridge::setupSerial()
{
    serial_.setPortName("/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    serial_.setBaudRate(QSerialPort::Baud115200);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    QObject::connect(&serial_, &QSerialPort::readyRead, [this]() {
        onSerialReadyRead();
    });

    if (!serial_.open(QIODevice::ReadWrite)) {
        RCLCPP_ERROR(this->get_logger(), "failed to open serial port");
    }
}

void AirborneLinkBridge::onSerialReadyRead()
{
    rx_buffer_.append(serial_.readAll());
    //尝试从接收缓冲区中解析出完整的协议帧，直到无法再解析出新的帧为止
    Packet packet;
    while (tryParseOnePacket(packet)) {
        handlePacket(packet);
    }
}

void AirborneLinkBridge::handlePacket(const Packet &packet)
{
    //如果收到的是ACK帧，则调用handleAck函数处理，并返回
    if (packet.type == lp::kTypeAck) {
        return;
    }

    //如果收到的帧需要ACK，则先发送ACK帧，通知对方已经收到消息，避免对方继续重试
    if (packet.flags & lp::kFlagNeedAck) {
        sendAck(packet.seq);
        auto it = completed_requests_.find(packet.seq);//发现重复调用
        if (it != completed_requests_.end()) {
            sendResponse(it->second.response_type, packet.seq, it->second.payload);
            return;
        }
    }

    //根据帧的类型调用不同的处理逻辑，并发送相应的ACK或响应数据
    switch (packet.type) {
    case lp::kTypeUploadMissionSummaryReq:
        handleUploadMissionSummaryRequest(packet.seq, packet.payload);
        break;
    case lp::kTypeStartOffboardReq:
        handleStartOffboardRequest(packet.seq, packet.payload);
        break;
    case lp::kTypeStartTaskReq:
        handleStartTaskRequest(packet.seq, packet.payload);
        break;  
    case lp::kTypeStopPushReq:
        handleStopPushRequest(packet.seq, packet.payload);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "unknown packet type: 0x%02X", packet.type);
        break;
    }
}

void AirborneLinkBridge::sendAck(uint16_t seq)
{
    const QByteArray empty_payload;
    const QByteArray frame = encodeFrame(lp::kTypeAck, lp::kFlagAck, seq, empty_payload);
    serial_.write(frame);
}

void AirborneLinkBridge::sendResponse(uint8_t type, uint16_t seq, const QByteArray &payload)
{
    const QByteArray frame = encodeFrame(type, 0, seq, payload);
    serial_.write(frame);
}





/******************* 数据处理层(发送层) ********************/

void AirborneLinkBridge::publishDroneStatus(const drone_msgs::msg::DroneStatus::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    /*************************消息打包**************************/

    stream << static_cast<quint8>(msg->connected ? 1 : 0);
    stream << static_cast<float>(msg->battery_percent);
    stream << static_cast<quint8>(msg->flight_mode);
    stream << static_cast<quint8>(msg->armed ? 1 : 0);

    const QByteArray state = QByteArray::fromStdString(msg->state);
    stream << static_cast<quint16>(state.size());
    payload.append(state);
    stream.writeRawData(state.constData(), state.size());

    /**********************************************************/

    const QByteArray frame = encodeFrame(lp::kTypeDroneStatus, 0, 0, payload);
    serial_.write(frame);
}

void AirborneLinkBridge::publishTaskStatus(const drone_msgs::msg::TaskStatus::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);

    /*************************消息打包**************************/

    stream << static_cast<quint8>(msg->task_running ? 1 : 0);

    const QByteArray action_name = QByteArray::fromStdString(msg->action_name);
    stream << static_cast<quint16>(action_name.size());
    stream.writeRawData(action_name.constData(), action_name.size());

    stream << static_cast<qint32>(msg->action_step);
    stream << static_cast<quint8>(msg->action_num);

    /**********************************************************/

    const QByteArray frame = encodeFrame(lp::kTypeTaskStatus, 0, 0, payload);
    serial_.write(frame);
}

void AirborneLinkBridge::publishPathReady(const drone_msgs::msg::ReadyStatus::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    QByteArray payload;
    //只有一个字节就不用stream了
    payload.append(static_cast<char>(msg->air_route_ready ? 1 : 0));
    const QByteArray frame = encodeFrame(lp::kTypePathReady, 0, 0, payload);
    serial_.write(frame);
}

void AirborneLinkBridge::publishReturnWorldGroup(const drone_msgs::msg::WorldGroup::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    /*************************消息打包**************************/

    const auto &points = msg->points;
    stream << static_cast<quint16>(points.size());

    for (const auto &point : points) {
        stream << static_cast<float>(point.x);
        stream << static_cast<float>(point.y);
    }

    /**********************************************************/

    const QByteArray frame = encodeFrame(lp::kTypeReturnWorldGroup, 0, 0, payload);
    serial_.write(frame);
}

void AirborneLinkBridge::publishVisionBarcode(const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    /*************************消息打包**************************/

    stream << static_cast<qint32>(msg->stamp.sec);
    stream << static_cast<quint32>(msg->stamp.nanosec);

    const QByteArray barcode = QByteArray::fromStdString(msg->barcode);
    stream << static_cast<quint16>(barcode.size());
    stream.writeRawData(barcode.constData(), barcode.size());
    const QByteArray image_format = QByteArray::fromStdString(msg->image_format);
    stream << static_cast<quint16>(image_format.size());
    stream.writeRawData(image_format.constData(), image_format.size());

    /**********************************************************/

    const QByteArray frame = encodeFrame(lp::kTypeVisionBarcode, 0, 0, payload);
    serial_.write(frame);
}

void AirborneLinkBridge::publishDelta(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    /*************************消息打包**************************/

    stream << static_cast<float>(msg->x);
    stream << static_cast<float>(msg->y);
    stream << static_cast<float>(msg->z);

    /**********************************************************/    

    const QByteArray frame = encodeFrame(lp::kTypeDelta, 0, 0, payload);
    serial_.write(frame);
}









/******************* 数据处理层(接收层) ********************/

void AirborneLinkBridge::handleUploadMissionSummaryRequest(uint16_t seq, const QByteArray &payload)
{
    if (!upload_mission_summary_client_ || !upload_mission_summary_client_->service_is_ready()) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("service /drone/upload_mission_summary not ready");
        const QByteArray saved_path;
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_stream.writeRawData(msg.constData(), msg.size());
        resp_stream << static_cast<quint16>(saved_path.size());
        resp_stream.writeRawData(saved_path.constData(), saved_path.size());
        resp_stream << static_cast<quint32>(0);

        cacheAndSendResponse(lp::kTypeUploadMissionSummaryResp, seq, resp_payload);
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint16 point_count = 0;
    stream >> point_count;

    const int fixed_summary_size =
        8 * 8 +   // 8个float64
        6 * 1 +   // 6个bool按uint8发
        10 * 8 +  // 10个新增float64
        2 +       // camera_aim_stable_cycles(uint16)
        2;        // frame_len

    if (payload.size() < 2 + static_cast<int>(point_count) * 8 + fixed_summary_size) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("invalid payload for UploadMissionSummaryReq");
        const QByteArray saved_path;
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_stream.writeRawData(msg.constData(), msg.size());
        resp_stream << static_cast<quint16>(saved_path.size());
        resp_stream.writeRawData(saved_path.constData(), saved_path.size());
        resp_stream << static_cast<quint32>(0);

        cacheAndSendResponse(lp::kTypeUploadMissionSummaryResp, seq, resp_payload);
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::UploadMissionSummary::Request>();
    request->points.reserve(point_count);

    for (quint16 i = 0; i < point_count; ++i) {
        float x = 0.0f;
        float y = 0.0f;
        stream >> x;
        stream >> y;

        drone_msgs::msg::WorldPoint point;
        point.x = x;
        point.y = y;
        request->points.push_back(point);
    }

    auto &summary = request->summary;
    stream >> summary.takeoff_altitude;
    stream >> summary.move_altitude;
    stream >> summary.start_altitude;
    stream >> summary.yaw;
    stream >> summary.tolerance;
    stream >> summary.takeoff_hover_duration;
    stream >> summary.landing_hover_duration;
    stream >> summary.move_hover_duration;

    quint8 add_hover_between_takeoff = 0;
    quint8 add_hover_between_landing = 0;
    quint8 add_hover_between_moves = 0;
    quint8 use_camera_aim = 0;
    quint8 auto_start_mission = 0;
    quint8 compress_straight_segments = 0;

    stream >> add_hover_between_takeoff;
    stream >> add_hover_between_landing;
    stream >> add_hover_between_moves;
    stream >> use_camera_aim;
    stream >> auto_start_mission;
    stream >> compress_straight_segments;

    summary.add_hover_between_takeoff = (add_hover_between_takeoff != 0);
    summary.add_hover_between_landing = (add_hover_between_landing != 0);
    summary.add_hover_between_moves = (add_hover_between_moves != 0);
    summary.use_camera_aim = (use_camera_aim != 0);
    summary.auto_start_mission = (auto_start_mission != 0);
    summary.compress_straight_segments = (compress_straight_segments != 0);

    stream >> summary.cam_tolerance;
    stream >> summary.camera_aim_pid_p;
    stream >> summary.camera_aim_pid_i;
    stream >> summary.camera_aim_pid_d;
    stream >> summary.camera_aim_target_timeout_s;
    stream >> summary.camera_aim_stable_cycles;
    stream >> summary.camera_aim_max_step;
    stream >> summary.camera_aim_wait_first_targets_timeout_s;
    stream >> summary.camera_aim_no_target_confirm_s;
    stream >> summary.camera_aim_record_result_timeout_s;
    stream >> summary.camera_aim_scan_point_timeout_s;

    quint16 frame_len = 0;
    stream >> frame_len;

    const qint64 frame_offset = stream.device() ? stream.device()->pos() : -1;
    if (frame_offset < 0 || payload.size() < frame_offset + frame_len) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("invalid frame in UploadMissionSummaryReq");
        const QByteArray saved_path;
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_stream.writeRawData(msg.constData(), msg.size());
        resp_stream << static_cast<quint16>(saved_path.size());
        resp_stream.writeRawData(saved_path.constData(), saved_path.size());
        resp_stream << static_cast<quint32>(0);

        cacheAndSendResponse(lp::kTypeUploadMissionSummaryResp, seq, resp_payload);
        return;
    }

    QByteArray frame_bytes(frame_len, Qt::Uninitialized);
    if (frame_len > 0) {
        const int read_size = stream.readRawData(frame_bytes.data(), frame_len);
        if (read_size != frame_len) {
            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray("invalid frame in UploadMissionSummaryReq");
            const QByteArray saved_path;
            resp_stream << static_cast<quint8>(0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_stream.writeRawData(msg.constData(), msg.size());
            resp_stream << static_cast<quint16>(saved_path.size());
            resp_stream.writeRawData(saved_path.constData(), saved_path.size());
            resp_stream << static_cast<quint32>(0);

            cacheAndSendResponse(lp::kTypeUploadMissionSummaryResp, seq, resp_payload);
            return;
        }
    }
    summary.frame = frame_bytes.toStdString();

    upload_mission_summary_client_->async_send_request(
        request,
        [this, seq](rclcpp::Client<drone_msgs::srv::UploadMissionSummary>::SharedFuture future)
        {
            const auto response = future.get();

            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray::fromStdString(response->message);
            const QByteArray saved_path = QByteArray::fromStdString(response->saved_path);

            resp_stream << static_cast<quint8>(response->success ? 1 : 0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_stream.writeRawData(msg.constData(), msg.size());
            resp_stream << static_cast<quint16>(saved_path.size());
            resp_stream.writeRawData(saved_path.constData(), saved_path.size());
            resp_stream << static_cast<quint32>(response->action_count);

            cacheAndSendResponse(lp::kTypeUploadMissionSummaryResp, seq, resp_payload);
        });
}

void AirborneLinkBridge::handleStartOffboardRequest(uint16_t seq, const QByteArray &payload)
{
    if (!start_offboard_client_ || !start_offboard_client_->service_is_ready()) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("service /drone/start_offboard not ready");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        cacheAndSendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint16 source_len = 0;
    stream >> source_len;

    if (payload.size() < 2 + source_len) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("invalid payload for StartOffboardReq");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        cacheAndSendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
        return;
    }

    QByteArray source_bytes(source_len, Qt::Uninitialized);
    if (source_len > 0) {
        const int read_size = stream.readRawData(source_bytes.data(), source_len);
        if (read_size != source_len) {
            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray("invalid payload for StartOffboardReq");
            resp_stream << static_cast<quint8>(0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_stream.writeRawData(msg.constData(), msg.size());

            cacheAndSendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
            return;
        }
    }
    const std::string request_source = source_bytes.toStdString();

    auto request = std::make_shared<drone_msgs::srv::StartOffboard::Request>();
    request->request_source = request_source;

    start_offboard_client_->async_send_request(
        request,
        [this, seq](rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedFuture future)
        {
            const auto response = future.get();

            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray::fromStdString(response->message);
            resp_stream << static_cast<quint8>(response->success ? 1 : 0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_payload.append(msg);
  
            cacheAndSendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
        });
}

void AirborneLinkBridge::handleStartTaskRequest(uint16_t seq, const QByteArray &payload)
{
    if (!start_task_client_ || !start_task_client_->service_is_ready()) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("service /drone/start_task not ready");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        cacheAndSendResponse(lp::kTypeStartTaskResp, seq, resp_payload);
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint16 source_len = 0;
    stream >> source_len;

    if (payload.size() < 2 + source_len) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("invalid payload for StartTaskReq");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        cacheAndSendResponse(lp::kTypeStartTaskResp, seq, resp_payload);
        return;
    }

    QByteArray task_name_bytes(source_len, Qt::Uninitialized);
    if (source_len > 0) {
        const int read_size = stream.readRawData(task_name_bytes.data(), source_len);
        if (read_size != source_len) {
            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray("invalid payload for StartTaskReq");
            resp_stream << static_cast<quint8>(0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_stream.writeRawData(msg.constData(), msg.size());

            cacheAndSendResponse(lp::kTypeStartTaskResp, seq, resp_payload);
            return;
        }
    }
    const std::string task_name = task_name_bytes.toStdString();

    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = task_name;

    start_task_client_->async_send_request(
        request,
        [this, seq](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            const auto response = future.get();

            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray::fromStdString(response->message);
            resp_stream << static_cast<quint8>(response->success ? 1 : 0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_payload.append(msg);

            cacheAndSendResponse(lp::kTypeStartTaskResp, seq, resp_payload);
        });
}

void AirborneLinkBridge::handleStopPushRequest(uint16_t seq, const QByteArray &payload)
{
    if (!stop_push_client_ || !stop_push_client_->service_is_ready()) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("service /drone/stop_push not ready");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        cacheAndSendResponse(lp::kTypeStopPushResp, seq, resp_payload);
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint16 source_len = 0;
    stream >> source_len;

    if (payload.size() < 2 + source_len) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("invalid payload for StopPushReq");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        cacheAndSendResponse(lp::kTypeStopPushResp, seq, resp_payload);
        return;
    }

    QByteArray task_name_bytes(source_len, Qt::Uninitialized);
    if (source_len > 0) {
        const int read_size = stream.readRawData(task_name_bytes.data(), source_len);
        if (read_size != source_len) {
            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray("invalid payload for StopPushReq");
            resp_stream << static_cast<quint8>(0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_stream.writeRawData(msg.constData(), msg.size());

            cacheAndSendResponse(lp::kTypeStopPushResp, seq, resp_payload);
            return;
        }
    }
    const std::string task_name = task_name_bytes.toStdString();

    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = task_name;

    stop_push_client_->async_send_request(
        request,
        [this, seq](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            const auto response = future.get();

            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray::fromStdString(response->message);
            resp_stream << static_cast<quint8>(response->success ? 1 : 0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_payload.append(msg);

            cacheAndSendResponse(lp::kTypeStopPushResp, seq, resp_payload);
        });
}








/************************ 底层 *************************/

bool AirborneLinkBridge::tryParseOnePacket(Packet &packet)
{
    while(true){
        //尝试从接收缓冲区中解析出完整的协议帧，直到无法再解析出新的帧为止
        while (rx_buffer_.size() >= 2) {
            const uint8_t b0 = static_cast<uint8_t>(rx_buffer_[0]);
            const uint8_t b1 = static_cast<uint8_t>(rx_buffer_[1]);
            if (b0 == lp::kSof1 && b1 == lp::kSof2) {
                break;
            }
            rx_buffer_.remove(0, 1);
        }

        if (rx_buffer_.size() < 11) {
            return false;
        }

        //从帧数据中提取载荷长度字段，计算出预期的帧长度，并与实际接收的帧长度进行比较，如果不匹配则认为帧不合法，继续尝试解析下一个帧
        const auto *data = reinterpret_cast<const uint8_t *>(rx_buffer_.constData());
        const uint16_t payload_len =
            static_cast<uint16_t>(data[7]) |
            (static_cast<uint16_t>(data[8]) << 8);

        //协议帧格式：2字节帧头 + 1字节版本 + 1字节类型 + 1字节标志 + 2字节序列号 + 2字节载荷长度 + N字节载荷 + 2字节CRC16校验
        const int frame_len = 9 + static_cast<int>(payload_len) + 2;
        if (rx_buffer_.size() < frame_len) {
            return false;
        }

        const QByteArray frame = rx_buffer_.left(frame_len);
        rx_buffer_.remove(0, frame_len);

        if (!validateFrame(frame)) {
            continue;
        }

        //从帧数据中提取消息类型、标志位、序列号和载荷数据，填充到Packet结构体中，供后续处理函数使用
        const auto *frame_data = reinterpret_cast<const uint8_t *>(frame.constData());
        packet.type = frame_data[3];
        packet.flags = frame_data[4];
        packet.seq = static_cast<uint16_t>(frame_data[5]) |
                    (static_cast<uint16_t>(frame_data[6]) << 8);
        packet.payload = frame.mid(9, payload_len);
        return true;
    }
}

QByteArray AirborneLinkBridge::encodeFrame(
    uint8_t type,
    uint8_t flags,
    uint16_t seq,
    const QByteArray &payload) const
{
    //编码函数，用于将消息类型、标志位、序列号和载荷数据编码成一个完整的帧数据，准备发送
    QByteArray frame;
    frame.reserve(9 + payload.size() + 2);

    //帧格式：2字节帧头 + 1字节版本 + 1字节类型 + 1字节标志 + 2字节序列号 + 2字节载荷长度 + N字节载荷 + 2字节CRC16校验
    frame.append(static_cast<char>(lp::kSof1));
    frame.append(static_cast<char>(lp::kSof2));
    frame.append(static_cast<char>(lp::kVersion));
    frame.append(static_cast<char>(type));
    frame.append(static_cast<char>(flags));

    frame.append(static_cast<char>(seq & 0xFF));
    frame.append(static_cast<char>((seq >> 8) & 0xFF));

    const uint16_t payload_len = static_cast<uint16_t>(payload.size());
    frame.append(static_cast<char>(payload_len & 0xFF));
    frame.append(static_cast<char>((payload_len >> 8) & 0xFF));

    frame.append(payload);

    const uint8_t *crc_begin = reinterpret_cast<const uint8_t *>(frame.constData() + 2);
    const int crc_len = frame.size() - 2;
    const uint16_t crc = crc16_ccitt(crc_begin, crc_len);

    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));

    return frame;
}

bool AirborneLinkBridge::validateFrame(const QByteArray &frame) const
{
    //验证函数，用于检查接收到的帧数据是否符合协议格式和校验要求，确保数据的正确性和完整性
    if (frame.size() < 11) {
        return false;
    }

    const auto *data = reinterpret_cast<const uint8_t *>(frame.constData());

    //检查帧头、版本号、载荷长度和CRC校验等字段，确保帧的合法性和完整性
    if (data[0] != lp::kSof1 || data[1] != lp::kSof2) {
        return false;
    }

    if (data[2] != lp::kVersion) {
        return false;
    }

    //从帧数据中提取载荷长度字段，计算出预期的帧长度，并与实际接收的帧长度进行比较，如果不匹配则认为帧不合法
    const uint16_t payload_len =
        static_cast<uint16_t>(data[7]) |
        (static_cast<uint16_t>(data[8]) << 8);

    const int expected_size = 9 + static_cast<int>(payload_len) + 2;
    if (frame.size() != expected_size) {
        return false;
    }

    //从帧数据中提取CRC16校验字段，计算出接收到的帧数据的CRC16校验值，并与提取的CRC16值进行比较，如果不匹配则认为帧数据有误，需要丢弃
    const uint16_t recv_crc =
        static_cast<uint16_t>(data[frame.size() - 2]) |
        (static_cast<uint16_t>(data[frame.size() - 1]) << 8);

    //计算CRC16校验值时，需要从帧数据的第3个字节开始（即版本号字段），一直计算到载荷数据的最后一个字节，才能得到正确的CRC16值
    const uint16_t calc_crc = crc16_ccitt(data + 2, frame.size() - 4);

    //如果帧数据的CRC16校验通过，则认为帧数据合法，可以继续进行后续处理；否则认为帧数据有误，需要丢弃
    return recv_crc == calc_crc;
}

void AirborneLinkBridge::cacheAndSendResponse(uint8_t type, uint16_t seq, const QByteArray &payload)
{
    //保存对应seq的type和payload
    completed_requests_[seq] = CachedResponse{type, payload};
    completed_request_order_.push_back(seq);

    while (completed_request_order_.size() > 100) {
        const uint16_t old_seq = completed_request_order_.front();
        completed_request_order_.pop_front();
        completed_requests_.erase(old_seq);
    }

    sendResponse(type, seq, payload);
}
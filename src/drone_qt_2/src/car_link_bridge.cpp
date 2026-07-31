#include "drone_qt_2/car_link_bridge.hpp"

#include <limits>

#include <QDataStream>

#include "drone_qt_2/link_protocol.hpp"

namespace lp = drone_msgs::link_protocol;

namespace
{
constexpr int kMaxPayloadSize = 4096;
void configureStream(QDataStream &stream)
{
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
}

uint16_t crc16Ccitt(const uint8_t *data, int length)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000)
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
}

CarLinkBridge::CarLinkBridge()
    : rclcpp::Node("airborne_car_link_bridge")
{
    serial_port_ = this->declare_parameter<std::string>(
        "serial_port", "/dev/warehouse_car_serial");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);

    setupSerial();
    setupRosInterfaces();
}

void CarLinkBridge::setupRosInterfaces()
{
    car_local_position_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/car/local_position",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            [this](
                const geometry_msgs::msg::PoseStamped::SharedPtr message) {
                sendCarLocalPosition(message);
            });

    car_keypad_s4_pressed_sub_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/keypad/s4_pressed",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr message) {
                sendCarKeypadS4Pressed(message);
            });

    car_route_state_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/route/state",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const std_msgs::msg::String::SharedPtr message) {
                sendCarRouteState(message);
            });

    car_control_mode_pub_ =
        this->create_publisher<std_msgs::msg::String>(
            "/control/mode",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
}

void CarLinkBridge::setupSerial()
{
    serial_.setPortName(QString::fromStdString(serial_port_));
    serial_.setBaudRate(baud_rate_);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    QObject::connect(&serial_, &QSerialPort::readyRead, [this]() {
        onSerialReadyRead();
    });

    if (!serial_.open(QIODevice::ReadWrite)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to open car serial port %s: %s",
            serial_port_.c_str(),
            serial_.errorString().toStdString().c_str());
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "car serial port opened: %s, baud=%d",
        serial_port_.c_str(),
        baud_rate_);
}

void CarLinkBridge::sendCarLocalPosition(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
    if (!message) {
        return;
    }
    if (!serial_.isOpen()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "car serial port is not open");
        return;
    }

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    configureStream(stream);

    stream << static_cast<double>(message->pose.position.x);
    stream << static_cast<double>(message->pose.position.y);
    stream << static_cast<double>(message->pose.position.z);
    stream << static_cast<double>(message->pose.orientation.x);
    stream << static_cast<double>(message->pose.orientation.y);
    stream << static_cast<double>(message->pose.orientation.z);
    stream << static_cast<double>(message->pose.orientation.w);

    if (stream.status() != QDataStream::Ok) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to encode car local position payload");
        return;
    }

    const QByteArray frame =
        encodeFrame(lp::kTypecarLocalPosition, 0, tx_sequence_++, payload);
    if (frame.isEmpty() || serial_.write(frame) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to write car local position frame: %s",
            serial_.errorString().toStdString().c_str());
    }
}

void CarLinkBridge::sendCarKeypadS4Pressed(
    const std_msgs::msg::Bool::SharedPtr message)
{
    if (!message || !serial_.isOpen()) {
        return;
    }

    QByteArray payload;
    payload.append(static_cast<char>(message->data ? 1 : 0));

    const QByteArray frame = encodeFrame(
        lp::kTypeCarKeypadS4Pressed,
        0,
        tx_sequence_++,
        payload);
    if (frame.isEmpty() || serial_.write(frame) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to write car S4 keypad frame: %s",
            serial_.errorString().toStdString().c_str());
    }
}

void CarLinkBridge::sendCarRouteState(
    const std_msgs::msg::String::SharedPtr message)
{
    if (!message || !serial_.isOpen()) {
        return;
    }

    // 统一成大写后传输，地面端可以直接按文档中的状态名进行映射。
    const QString state =
        QString::fromStdString(message->data).trimmed().toUpper();
    if (state.isEmpty()) {
        return;
    }

    const QByteArray frame = encodeFrame(
        lp::kTypeCarRouteState,
        0,
        tx_sequence_++,
        state.toUtf8());
    if (frame.isEmpty() || serial_.write(frame) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to write car route state frame: %s",
            serial_.errorString().toStdString().c_str());
    }
}

QByteArray CarLinkBridge::encodeFrame(
    uint8_t type,
    uint8_t flags,
    uint16_t sequence,
    const QByteArray &payload) const
{
    if (payload.size() > std::numeric_limits<quint16>::max()) {
        return {};
    }

    QByteArray frame;
    frame.reserve(9 + payload.size() + 2);
    frame.append(static_cast<char>(lp::kSof1));
    frame.append(static_cast<char>(lp::kSof2));
    frame.append(static_cast<char>(lp::kVersion));
    frame.append(static_cast<char>(type));
    frame.append(static_cast<char>(flags));
    frame.append(static_cast<char>(sequence & 0xFF));
    frame.append(static_cast<char>((sequence >> 8) & 0xFF));

    const auto payload_length =
        static_cast<uint16_t>(payload.size());
    frame.append(static_cast<char>(payload_length & 0xFF));
    frame.append(static_cast<char>((payload_length >> 8) & 0xFF));
    frame.append(payload);

    const auto *crc_begin =
        reinterpret_cast<const uint8_t *>(frame.constData() + 2);
    const uint16_t crc =
        crc16Ccitt(crc_begin, frame.size() - 2);
    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));
    return frame;
}

void CarLinkBridge::onSerialReadyRead()
{
    rx_buffer_.append(serial_.readAll());

    uint8_t type = 0;
    QByteArray payload;
    while (tryParseCarFrame(type, payload)) {
        if (type == lp::kTypeCarControlMode) {
            publishCarControlMode(payload);
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "ignored unexpected car serial frame type: 0x%02X",
                type);
        }
    }
}

bool CarLinkBridge::tryParseCarFrame(
    uint8_t &type,
    QByteArray &payload)
{
    while (true) {
        while (rx_buffer_.size() >= 2) {
            const auto first = static_cast<uint8_t>(rx_buffer_[0]);
            const auto second = static_cast<uint8_t>(rx_buffer_[1]);
            if (first == lp::kSof1 && second == lp::kSof2) {
                break;
            }
            rx_buffer_.remove(0, 1);
        }

        if (rx_buffer_.size() < 11) {
            return false;
        }

        const auto *data =
            reinterpret_cast<const uint8_t *>(rx_buffer_.constData());
        const uint16_t payload_length =
            static_cast<uint16_t>(data[7]) |
            (static_cast<uint16_t>(data[8]) << 8);

        if (payload_length > kMaxPayloadSize) {
            rx_buffer_.remove(0, 1);
            continue;
        }

        const int frame_length = 9 + static_cast<int>(payload_length) + 2;
        if (rx_buffer_.size() < frame_length) {
            return false;
        }

        const QByteArray frame = rx_buffer_.left(frame_length);
        rx_buffer_.remove(0, frame_length);

        if (!validateFrame(frame)) {
            RCLCPP_WARN(this->get_logger(), "discarded invalid car serial frame");
            continue;
        }

        const auto *frame_data =
            reinterpret_cast<const uint8_t *>(frame.constData());
        type = frame_data[3];
        payload = frame.mid(9, payload_length);
        return true;
    }
}

bool CarLinkBridge::validateFrame(const QByteArray &frame) const
{
    if (frame.size() < 11) {
        return false;
    }

    const auto *data =
        reinterpret_cast<const uint8_t *>(frame.constData());
    if (data[0] != lp::kSof1 ||
        data[1] != lp::kSof2 ||
        data[2] != lp::kVersion) {
        return false;
    }

    const uint16_t payload_length =
        static_cast<uint16_t>(data[7]) |
        (static_cast<uint16_t>(data[8]) << 8);
    if (frame.size() != 9 + static_cast<int>(payload_length) + 2) {
        return false;
    }

    const uint16_t received_crc =
        static_cast<uint16_t>(data[frame.size() - 2]) |
        (static_cast<uint16_t>(data[frame.size() - 1]) << 8);
    const uint16_t calculated_crc =
        crc16Ccitt(data + 2, frame.size() - 4);
    return received_crc == calculated_crc;
}

void CarLinkBridge::publishCarControlMode(const QByteArray &payload)
{
    const QString mode =
        QString::fromUtf8(payload).trimmed().toUpper();
    if (mode != "DISABLED" && mode != "AUTO") {
        RCLCPP_WARN(
            this->get_logger(),
            "ignored unsupported car control mode payload");
        return;
    }

    std_msgs::msg::String message;
    message.data = mode.toStdString();
    car_control_mode_pub_->publish(message);
}

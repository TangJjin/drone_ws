#include "drone_qt_2/car_link_bridge.hpp"

#include <limits>

#include <QDataStream>

#include "drone_qt_2/link_protocol.hpp"

namespace lp = drone_msgs::link_protocol;

namespace
{
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
        "serial_port", "/dev/fishbot_lidar");
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
}

void CarLinkBridge::setupSerial()
{
    serial_.setPortName(QString::fromStdString(serial_port_));
    serial_.setBaudRate(baud_rate_);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_.open(QIODevice::WriteOnly)) {
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
        encodeFrame(lp::kTypecarLocalPosition, 0, 0, payload);
    if (frame.isEmpty() || serial_.write(frame) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to write car local position frame: %s",
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
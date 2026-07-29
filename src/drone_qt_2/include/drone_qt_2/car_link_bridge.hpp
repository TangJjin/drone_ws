#pragma once

#include <cstdint>
#include <string>

#include <QByteArray>
#include <QSerialPort>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class CarLinkBridge : public rclcpp::Node
{
public:
    CarLinkBridge();
    ~CarLinkBridge() override = default;

private:
    void setupRosInterfaces();
    void setupSerial();
    void sendCarLocalPosition(
        const geometry_msgs::msg::PoseStamped::SharedPtr message);

    QByteArray encodeFrame(
        uint8_t type,
        uint8_t flags,
        uint16_t sequence,
        const QByteArray &payload) const;

    std::string serial_port_;
    int baud_rate_{115200};

    QSerialPort serial_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        car_local_position_sub_;
};
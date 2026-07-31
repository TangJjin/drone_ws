#pragma once

#include <cstdint>
#include <string>

#include <QByteArray>
#include <QSerialPort>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class CarLinkBridge : public rclcpp::Node
{
public:
    CarLinkBridge();
    ~CarLinkBridge() override = default;

private:
    void setupRosInterfaces();
    void setupSerial();
    void onSerialReadyRead();
    void sendCarLocalPosition(
        const geometry_msgs::msg::PoseStamped::SharedPtr message);
    void sendCarKeypadS4Pressed(
        const std_msgs::msg::Bool::SharedPtr message);
    void sendCarRouteState(
        const std_msgs::msg::String::SharedPtr message);
    bool tryParseCarFrame(uint8_t &type, QByteArray &payload);
    bool validateFrame(const QByteArray &frame) const;
    void publishCarControlMode(const QByteArray &payload);

    QByteArray encodeFrame(
        uint8_t type,
        uint8_t flags,
        uint16_t sequence,
        const QByteArray &payload) const;

    std::string serial_port_;
    int baud_rate_{115200};

    QSerialPort serial_;
    QByteArray rx_buffer_;
    uint16_t tx_sequence_{0};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        car_local_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        car_keypad_s4_pressed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        car_route_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        car_control_mode_pub_;
};

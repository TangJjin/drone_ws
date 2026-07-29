#include <memory>

#include <QCoreApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

#include "drone_qt_2/car_link_bridge.hpp"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CarLinkBridge>();

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    const int result = app.exec();
    rclcpp::shutdown();
    return result;
}
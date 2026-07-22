#include <QCoreApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "drone_warehouse/ground_link_bridge.hpp"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GroundLinkBridge>();

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    const int rc = app.exec();
    rclcpp::shutdown();
    return rc;
}
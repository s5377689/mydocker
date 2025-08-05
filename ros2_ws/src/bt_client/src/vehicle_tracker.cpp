#include "vehicle_tracker.hpp"

VehicleTracker::VehicleTracker(QObject * parent)
    : QObject(parent)
{
    node_ = rclcpp::Node::make_shared("vehicle_tracker_node");

    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    navsat_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ap/navsat",
        qos_profile,
        [this] (const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            navSatCallback(msg);
        }
    );

    std::thread([this]() {
        rclcpp::spin(node_);
    }).detach();
}

void VehicleTracker::navSatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    altitude_ = msg->altitude;

    emit positionChanged();
}
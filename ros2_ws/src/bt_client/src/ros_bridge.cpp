#include "ros_bridge.h"

RosBridge::RosBridge(QObject* parent):
    QObject(parent)
{
    node_ = rclcpp::Node::make_shared("qt_ros_bridge");
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/bt_status", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit btXmlReceived(QString::fromStdString(msg->data));
        }
    );

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(); 
    executor_->add_node(node_);

    spinThread_ = std::thread([this]() {
        executor_->spin();
    });
}

RosBridge::~RosBridge()
{
    executor_->cancel();
    if (spinThread_.joinable()) {
        spinThread_.join();
    }
}

rclcpp::Node::SharedPtr RosBridge::node()
{
    return node_;
}
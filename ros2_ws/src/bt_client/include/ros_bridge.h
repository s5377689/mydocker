#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QTimer>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "parser.h"

class RosBridge : public QObject
{
    Q_OBJECT

public:
    RosBridge(QObject* parent = nullptr);
    ~RosBridge();
    rclcpp::Node::SharedPtr node();

signals:
    void btXmlReceived(const QString& xml);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::shared_ptr<rclcpp::Executor> executor_;
    std::thread spinThread_;
};
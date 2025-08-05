#pragma once
#include <QObject>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <mutex>
#include <thread>
#include <atomic>

class Ros2TopicSubscriber : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
public:
    explicit Ros2TopicSubscriber(QObject* parent = nullptr);
    ~Ros2TopicSubscriber();

    Q_INVOKABLE void start();
    Q_INVOKABLE void stop();
    Q_INVOKABLE bool running() const { return running_; }

    // Called by provider to get the latest image
    QImage getLatestImage() const;

signals:
    void runningChanged();
    void imageUpdated();

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    QImage img_;
    mutable std::mutex img_mutex_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
    std::atomic<bool> running_{false};
};
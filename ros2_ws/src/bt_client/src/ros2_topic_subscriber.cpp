#include "ros2_topic_subscriber.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <QImage>
#include <iostream>

Ros2TopicSubscriber::Ros2TopicSubscriber(QObject* parent)
    : QObject(parent)
{}

Ros2TopicSubscriber::~Ros2TopicSubscriber() {
    stop();
}

void Ros2TopicSubscriber::start() {
    if (running_) return;
    running_ = true;
    emit runningChanged();

    if (!node_)
        node_ = rclcpp::Node::make_shared("ros2_topic_subscriber_node");

    img_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/camera/compressed/image_search", 10,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            this->imageCallback(msg);
        }
    );

    spin_thread_ = std::thread([this]() {
        executor_.add_node(node_);
        rclcpp::WallRate rate(30);
        while (rclcpp::ok() && running_) {
            executor_.spin_some();
            rate.sleep();
        }
        executor_.remove_node(node_);
    });
}

void Ros2TopicSubscriber::stop() {
    running_ = false;
    emit runningChanged();
    if (spin_thread_.joinable())
        spin_thread_.join();
    img_sub_.reset();
    node_.reset();
}

QImage Ros2TopicSubscriber::getLatestImage() const {
    std::lock_guard<std::mutex> lock(img_mutex_);
    return img_;
}

void Ros2TopicSubscriber::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv::Mat cv_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (cv_img.empty()) {
        std::cerr << "[Ros2TopicSubscriber] Failed to decode image." << std::endl;
        return;
    }
    cv::cvtColor(cv_img, cv_img, cv::COLOR_BGR2RGB);
    QImage qimg(cv_img.data, cv_img.cols, cv_img.rows, cv_img.step, QImage::Format_RGB888);
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        img_ = qimg.copy();
    }
    emit imageUpdated();
}
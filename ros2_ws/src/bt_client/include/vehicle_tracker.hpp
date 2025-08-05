#pragma once
#include <QObject>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>


class VehicleTracker : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double latitude READ latitude NOTIFY positionChanged)
    Q_PROPERTY(double longitude READ longitude NOTIFY positionChanged)
    Q_PROPERTY(double altitude READ altitude NOTIFY positionChanged)

public:
    explicit VehicleTracker(QObject * parent = nullptr);

    double latitude() const { return latitude_; }
    double longitude() const { return longitude_; }
    double altitude() const { return altitude_; }

signals:
    void positionChanged();

private slots:
    void navSatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;

    double latitude_ = 0.0;
    double longitude_ = 0.0;
    double altitude_ = 0.0;
};
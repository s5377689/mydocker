#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "bt_action/bt_node.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "sensor_msgs/msg/imu.hpp"

namespace bt_action
{

class Zigzag : public BtNode
{
public:
    Zigzag(
        const std::string & name,
        const BT::NodeConfiguration & config
    );

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("acc_timeout"),  // Time to accelerate to the desired speed
            BT::InputPort<double>("action_time"),  // Total time for the zigzag action (excluding acceleration time)
            BT::InputPort<double>("speed"),
            BT::InputPort<double>("amplitude"),
            BT::InputPort<double>("period"),
            BT::InputPort<double>("damping"),
            BT::InputPort<double>("clamp")
        };
    }

protected:
    void run_action_() override;

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    double y_linear_vel_ {0.0};
    double z_angular_vel_ {0.0};
};

}  // end of namespace bt_action
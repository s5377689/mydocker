#include "bt_action/stop_gimbal_control.hpp"

namespace bt_action
{

void StopGimbalControl::initialize(
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
{
    gimbal_controller_ = gimbal_controller;
}

void StopGimbalControl::run_action_()
{
    auto gimbal_controller = gimbal_controller_.lock();
    if (!gimbal_controller) {
        // Gimbal controller is not initialized or has been destroyed
        RCLCPP_INFO(get_logger(), "Gimbal controller is not available, skipping stop action.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    if (!gimbal_controller->isConnected()) {
        RCLCPP_WARN(get_logger(), "Gimbal controller is not connected, skipping stop action.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    gimbal_controller->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (gimbal_controller->isConnected()) {
        RCLCPP_ERROR(get_logger(), "Gimbal control is still connected after stop command.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    } else {
        RCLCPP_INFO(get_logger(), "Gimbal control stopped successfully.");
        status_ = BT::NodeStatus::SUCCESS;
        return;
    }
}

}  // namespace bt_action
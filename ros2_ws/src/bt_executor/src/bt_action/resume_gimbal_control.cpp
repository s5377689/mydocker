#include "bt_action/resume_gimbal_control.hpp"

namespace bt_action
{

void ResumeGimbalControl::initialize(
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
{
    gimbal_controller_ = gimbal_controller;
}

void ResumeGimbalControl::run_action_()
{
    auto gimbal_controller = gimbal_controller_.lock();
    if (!gimbal_controller) {
        // Gimbal controller is not initialized or has been destroyed
        RCLCPP_INFO(get_logger(), "Gimbal controller is not available, skipping resume action.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    gimbal_controller->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!gimbal_controller->isConnected()) {
        RCLCPP_ERROR(get_logger(), "Gimbal control is still not connected after resume command.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    } else {
        RCLCPP_INFO(get_logger(), "Gimbal control resumed successfully.");
        status_ = BT::NodeStatus::SUCCESS;
        return;
    }
}

}  // namespace bt_action
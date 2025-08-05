#include <chrono>
#include "bt_action/disarm.hpp"
#include "ardupilot_msgs/srv/arm_motors.hpp"


namespace bt_action
{

void Disarm::run_action_()
{
    // Create a client for the arm_motors service
    auto client = this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");

    // Wait for the service to be available
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !client->service_is_ready()) {
        if (halt_requested_) {
            RCLCPP_INFO(this->get_logger(), "Action halted, stopping arm motors service call.");
            status_ = BT::NodeStatus::FAILURE;
            return;
        }
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(timeout_)) {
            std::string result_msg {
                "Arm motors service not available after waiting for " + std::to_string(timeout_) + " seconds."
            };
            RCLCPP_ERROR(this->get_logger(), "%s", result_msg.c_str());
            *result_msg_ = result_msg;
            status_ = BT::NodeStatus::FAILURE;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for arm motors service to become available...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Create a request
    auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
    request->arm = false;

    std::promise<BT::NodeStatus> promise;
    auto future = promise.get_future();

    // Call the service
    client->async_send_request(
        request,
        [this, &promise] (rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture future) {
            auto response = future.get();
            if (response->result) {
                RCLCPP_INFO(this->get_logger(), "Disarm motors successfully.");
                promise.set_value(BT::NodeStatus::SUCCESS);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to disarm motors.");
                promise.set_value(BT::NodeStatus::FAILURE);
            }
        }
    );

    // Wait for the service response
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
        if (halt_requested_) {
            RCLCPP_INFO(this->get_logger(), "Action halted while waiting for arm service response.");
            status_ = BT::NodeStatus::FAILURE;
            return;
        }
        rclcpp::spin_some(this->get_node_base_interface());
    }

    status_ = future.get();
}

}  // namespace bt_action
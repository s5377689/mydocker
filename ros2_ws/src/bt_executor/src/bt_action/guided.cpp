#include <chrono>
#include "bt_action/guided.hpp"
#include "ardupilot/mode_enum.hpp"
#include "ardupilot_msgs/srv/mode_switch.hpp"


namespace bt_action
{

void Guided::run_action_()
{
    // Create a client for the mode_switch service
    auto client = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");

    // Wait for the service to be available
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !client->service_is_ready()) {
        if (halt_requested_) {
            RCLCPP_INFO(this->get_logger(), "Action halted, stopping mode switch service call.");
            status_ = BT::NodeStatus::FAILURE;
            return;
        }
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(timeout_)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting for %d seconds", timeout_);
            status_ = BT::NodeStatus::FAILURE;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for mode switch service to become available...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Create a request
    auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
    request->mode = static_cast<u_int8_t>(ArduPilotMode::GUIDED);

    std::promise<BT::NodeStatus> promise;
    auto future = promise.get_future();

    // Call the service
    client->async_send_request(
        request,
        [this, &promise] (rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture future) {
            auto response = future.get();
            if (response->status) {
                RCLCPP_INFO(this->get_logger(), "Mode switched successfully.");
                promise.set_value(BT::NodeStatus::SUCCESS);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch mode.");
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
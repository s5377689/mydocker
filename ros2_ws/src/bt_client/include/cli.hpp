#include <poll.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <future>
#include <cinttypes>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "custom_msgs/action/behavior_tree.hpp"


class BtClient : public rclcpp::Node
{
public:
    using ActionT = custom_msgs::action::BehaviorTree;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

    explicit BtClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~BtClient();

    void sendGoal(
        const std::string & xml_text,
        const int timeout = 5
    );
    void printHelp();
    void setWorkingDirectory(const std::string & working_directory) { working_directory_ = working_directory; }

private:
    // Action client methods
    void btGoalResponseCb(const std::shared_ptr<GoalHandle> & goal_handle);
    void btFeedbackCb(
        std::shared_ptr<GoalHandle>,
        const std::shared_ptr<const ActionT::Feedback> feedback
    );
    void btResultCb(const GoalHandle::WrappedResult & result);
    void request_cancel();

    // Service client methods
    void srvResponseCb(
        rclcpp::Client<custom_msgs::srv::GetTargetList>::SharedFuture future
    );

    // CLI (Command Line Interface) methods
    void process_command();
    std::string readXmlFile(const std::string & filename, const std::string & prefix = "");

    // Action client data
    std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
    std::shared_ptr<GoalHandle> goal_handle_;
    rclcpp::CallbackGroup::SharedPtr action_client_cb_group_;

    // Service client data
    std::shared_ptr<rclcpp::Client<custom_msgs::srv::GetTargetList>> srv_client_;
    rclcpp::CallbackGroup::SharedPtr srv_client_cb_group_;

    // CLI data
    std::thread command_thread_;
    std::atomic<bool> waiting_command_;
    std::string working_directory_;
};

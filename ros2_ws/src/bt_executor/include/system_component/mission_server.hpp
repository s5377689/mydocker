#pragma once

#include <memory>
#include <thread>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

#include "system_component/target_registry.hpp"
#include "gimbal/usv/gimbal_streamer.hpp"
#include "gimbal/usv/gimbal_controller.hpp"
#include "custom_msgs/action/behavior_tree.hpp"
#include "custom_msgs/srv/get_target_list.hpp"

namespace system_component
{

class MissionServer : public rclcpp::Node
{
public:
    using ActionT = custom_msgs::action::BehaviorTree;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    explicit MissionServer(const rclcpp::NodeOptions & options);
    ~MissionServer();

private:
    // ---------------------------------------
    //   Callbacks for this BT action server
    // ---------------------------------------
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ActionT::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle
    );
    void handle_accepted(
        const std::shared_ptr<GoalHandle> goal_handle
    );

    rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;

    // ---------------------------------------
    //              BT execution
    // ---------------------------------------
    void execute(
        const std::shared_ptr<GoalHandle> goal_handle
    );
    void resetBt();

    // void cancelMission(
    //     const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    //     std::shared_ptr<std_srvs::srv::Trigger::Response>
    // );

    // ---------------------------------------
    //            BT status update
    // ---------------------------------------
    // Timer callback to update and publish the status XML
    void status_timer_callback();
    void update_and_publish_bt_status();

    // BT status
    std::string bt_id_;
    BT::NodeStatus bt_status_;
    std::string status_xml_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    BT::BehaviorTreeFactory factory_;
    std::shared_ptr<BT::Tree> tree_;
    std::shared_ptr<GoalHandle> goal_handle_;

    std::shared_ptr<std::string> result_msg_;

    // ---------------------------------------
    //   Target management (needed by <Search>)
    // ---------------------------------------
    void get_target_list(
        const std::shared_ptr<custom_msgs::srv::GetTargetList::Request>,
        std::shared_ptr<custom_msgs::srv::GetTargetList::Response>
    );
    rclcpp::Service<custom_msgs::srv::GetTargetList>::SharedPtr get_target_list_service_;
    std::shared_ptr<TargetRegistry> registry_;

    // ---------------------------------------
    //              Gimbal control
    // ---------------------------------------
    // void stopGimbalControl();
    // void resumeGimbalControl();

    // Gimbal camera streams
    std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_eo_streamer_;
    std::string eo_rtsp_url_;

    // Gimbal controller
    std::shared_ptr<gimbal::usv::GimbalController> gimbal_controller_; 
    std::string gimbal_control_ip_;
    int gimbal_control_port_;
};

} // namespace system_component
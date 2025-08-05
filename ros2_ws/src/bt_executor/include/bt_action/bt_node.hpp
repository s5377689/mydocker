/*
    BtNode is a ROS2 node to be used as a Behavior Tree (BT) action node.
    It creates a detached thread run_action_() to run the action.
    The thread will exit when action is halted or finished.
*/

#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_msgs/action/behavior_tree.hpp"

namespace bt_action
{

class BtNode :
    public BT::StatefulActionNode, public rclcpp::Node
{
public:
    using ActionT = custom_msgs::action::BehaviorTree;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    // To create a BtNode, you need to implement the following 3 methods:
    // 1. As a BT::StateFulActionNode, the following constructor is required:
    // <Class Constructor>(
    //     const std::string & name,
    //     const BT::NodeConfiguration & config
    // ):
    //     BtNode(name, config) {}

    BtNode(
        const std::string & name,
        const BT::NodeConfiguration & config
    );
    ~BtNode();

    // result_msg will be printed out by the mission server
    // goal_handle is used to publish the feedback message
    void baseInitialize(
        std::shared_ptr<std::string> result_msg,
        std::shared_ptr<GoalHandle> goal_handle
    );

    // 2. As a BT::StatefulActionNode, input ports are required:
    // static BT::PortsList providedPorts() { return {}; }

    virtual BT::NodeStatus onStart() override;
    virtual BT::NodeStatus onRunning() override;
    virtual void onHalted() override;

protected:
    // 3. Override the run_action_() method to implement action execution:
    // - if action is done, set status_ to SUCCESS or FAILURE depending on the result, and return.
    // - if halt_requested_ is true, set status_ to FAILURE and return.
    virtual void run_action_() = 0;

    std::thread action_thread_;
    std::atomic<BT::NodeStatus> status_ {BT::NodeStatus::IDLE};
    std::atomic<bool> halt_requested_ {false};

    std::shared_ptr<std::string> result_msg_;
    std::shared_ptr<GoalHandle> goal_handle_;
};

}  // namespace bt_action
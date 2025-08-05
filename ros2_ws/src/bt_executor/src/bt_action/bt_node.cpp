#include "bt_action/bt_node.hpp"

namespace bt_action
{

BtNode::BtNode(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BT::StatefulActionNode(name, config),
    Node(name + "_node")
{}

BtNode::~BtNode()
{
    if (action_thread_.joinable()) {
        action_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "BT Action thread has exited.");
}

BT::NodeStatus BtNode::onStart()
{
    status_ = BT::NodeStatus::RUNNING;
    halt_requested_ = false;  // To prevent any previous halt from affecting the new action
    action_thread_ = std::thread(std::bind(&BtNode::run_action_, this));
    return status_;
}

BT::NodeStatus BtNode::onRunning()
{
    // Periodically process the callbacks of this ros2 node
    rclcpp::spin_some(this->get_node_base_interface());
    return status_;
}

void BtNode::onHalted()
{
    halt_requested_ = true;
    status_ = BT::NodeStatus::IDLE;
}

}  // namespace bt_action
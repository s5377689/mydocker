#pragma once
#include "bt_action/bt_node.hpp"
#include "ardupilot_msgs/srv/arm_motors.hpp"


namespace bt_action
{

class Disarm : public BtNode
{
public:
    Disarm(
        const std::string & name,
        const BT::NodeConfiguration & config
    ):
        BtNode(name, config)   
    {}

    static BT::PortsList providedPorts() { return {}; }

    void run_action_() override;

private:
    int timeout_ {3}; // Timeout for waiting the arm service in seconds
};

}  // namespace bt_action
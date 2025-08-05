#pragma once
#include "bt_action/bt_node.hpp"


namespace bt_action
{

class Guided : public BtNode
{
public:
    Guided(
        const std::string & name,
        const BT::NodeConfiguration & config
    ):
        BtNode(name, config)   
    {}

    static BT::PortsList providedPorts() { return {}; }

    void run_action_() override;

private:
    int timeout_ {3}; // Timeout for waiting the mode switch service in seconds
};

}  // namespace bt_action
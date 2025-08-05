#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <thread>
#include <chrono>
#include "bt_action/bt_node.hpp"
#include "gimbal/usv/gimbal_controller.hpp"


namespace bt_action
{

class ResumeGimbalControl : public BtNode
{
public:
    ResumeGimbalControl(
        const std::string & name,
        const BT::NodeConfiguration & config
    ):
        BtNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    void initialize(std::shared_ptr<gimbal::usv::GimbalController>);
protected:
    void run_action_() override;
private:
    std::weak_ptr<gimbal::usv::GimbalController> gimbal_controller_;
};

}  // namespace bt_action
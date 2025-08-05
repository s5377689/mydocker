#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <behaviortree_cpp/bt_factory.h>
#include "bt_action/bt_node.hpp"
#include "gimbal/usv/gimbal_streamer.hpp"


namespace bt_action
{

class TakePhoto : public BtNode
{
public:
    TakePhoto(
        const std::string & name,
        const BT::NodeConfiguration & config
    );

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("image_folder")
        };
    }

    void initialize(
        std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer
    );

protected:
    void run_action_() override;

private:
    std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer_;
};

}  // end of namespace bt_action
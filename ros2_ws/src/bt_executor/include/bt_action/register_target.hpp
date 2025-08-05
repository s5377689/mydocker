#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <filesystem>
#include <unordered_map>
#include <mutex>

#include "bt_action/bt_node.hpp"
#include "system_component/target_registry.hpp"


namespace bt_action
{

class RegisterTarget : public BtNode
{
public:
    RegisterTarget(
        const std::string & name,
        const BT::NodeConfiguration & config
    );

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("target_id"),
            BT::InputPort<std::string>("image_folder")
        };
    }

    void initialize(std::shared_ptr<system_component::TargetRegistry> registry);

protected:
    void run_action_() override;

private:
    std::shared_ptr<system_component::TargetRegistry> registry_;
};

}  // namespace bt_action
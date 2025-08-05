#pragma once
#include <atomic>
#include <string>
#include <iostream>
#include "gimbal/helpers.hpp"

namespace gimbal {
    class GimbalController;  // Forward declaration
}

namespace gimbal_action {

class GimbalAction
{
public:
    explicit GimbalAction(
        gimbal::GimbalController & gimbal_controller,
        std::string action_name = "") 
    :
        gimbal_controller_(gimbal_controller),
        action_name_(action_name)
    {}
    virtual ~GimbalAction() = default;

    virtual void run() = 0;  // Run the action
    virtual void halt() {
        halt_requested_ = true;
    }
    bool isHalted() const { return halt_requested_; }
    std::string getName() const { return action_name_; }

protected:
    gimbal::GimbalController & gimbal_controller_; 
    std::atomic<bool> halt_requested_ {false};
    std::string action_name_;  // Name of the action for logging
};

}  // namespace gimbal_action
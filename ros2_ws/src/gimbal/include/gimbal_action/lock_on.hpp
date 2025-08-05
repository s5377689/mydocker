#pragma once
#include "gimbal_action/gimbal_action.hpp"
#include "gimbal/usv/gimbal_controller.hpp"
#include <opencv2/opencv.hpp>

namespace gimbal_action
{

class LockOn : public GimbalAction
{
public:    
    explicit LockOn(
        gimbal::usv::GimbalController & gimbal_controller
    ):
        GimbalAction(gimbal_controller, "LockOn")
    {}

    void run() override;
};

}  // namespace gimbal_action
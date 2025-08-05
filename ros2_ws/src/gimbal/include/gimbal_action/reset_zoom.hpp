#pragma once
#include "gimbal/usv/gimbal_controller.hpp"
#include "gimbal_action/gimbal_action.hpp"

namespace gimbal_action
{

class ResetZoom : public GimbalAction
{
public:    
    explicit ResetZoom(
        gimbal::usv::GimbalController & gimbal_controller
    ):
        GimbalAction(gimbal_controller, "ResetZoom")
    {}

    void run() override;

private:
    double zoom_tolerance_ {0.1};
};

}  // namespace gimbal_action
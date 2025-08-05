#pragma once
#include "gimbal/gimbal_controller.hpp"
#include "gimbal_action/gimbal_action.hpp"

namespace gimbal_action
{

class ResetZoom : public GimbalAction
{
public:    
    explicit ResetZoom(
        gimbal::GimbalController & gimbal_controller
    ):
        GimbalAction(gimbal_controller, "ResetZoom")
    {}

    void run() override;

private:
    double zoom_tolerance_ {0.01};
};

}  // namespace gimbal_action
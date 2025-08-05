#pragma once
#include "gimbal/gimbal_controller.hpp"
#include "gimbal_action/gimbal_action.hpp"

namespace gimbal_action
{

class SetZoom : public GimbalAction
{
public:    
    explicit SetZoom(
        gimbal::GimbalController & gimbal_controller,
        double target_zoom
    ):
        GimbalAction(gimbal_controller, "SetZoom"),
        target_zoom_(target_zoom)
    {}

    void run() override;

private:
    double target_zoom_;
    double zoom_tolerance_ {0.05};
};

}  // namespace gimbal_action
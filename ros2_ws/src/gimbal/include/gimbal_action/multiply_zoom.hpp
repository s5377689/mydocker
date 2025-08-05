#pragma once
#include "gimbal/usv/gimbal_controller.hpp"
#include "gimbal_action/gimbal_action.hpp"

namespace gimbal_action
{

class MultiplyZoom : public GimbalAction
{
public:    
    explicit MultiplyZoom(
        gimbal::usv::GimbalController & gimbal_controller,
        double factor
    ):
        GimbalAction(gimbal_controller, "IncreaseZoom"),
        factor_(factor)
    {}

    void run() override;

private:
    double factor_;
    double zoom_tolerance_ {0.45};
};

}  // namespace gimbal_action
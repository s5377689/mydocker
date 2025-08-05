#pragma once
#include "gimbal_action/gimbal_action.hpp"
#include "gimbal/usv/gimbal_controller.hpp"
#include <opencv2/opencv.hpp>

namespace gimbal_action {

class ResetPitch : public GimbalAction
{
public:    
    explicit ResetPitch(
        gimbal::usv::GimbalController & gimbal_controller,
        double pitch_speed,
        double target_pitch 
    ):
        GimbalAction(gimbal_controller, "ResetPitch"),
        pitch_speed_(pitch_speed),
        target_pitch_(target_pitch)
    {}

    void run() override;

    double degree_tolerance_ {2.5};  // Tolerance in degrees for stopping the action
    double target_pitch_;
    double pitch_speed_ = {15.0};   // Speed of pitch movement in degrees per second
};

}  // namespace gimbal
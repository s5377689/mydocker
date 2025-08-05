#pragma once
#include "gimbal_action/gimbal_action.hpp"
#include "gimbal/gimbal_controller.hpp"
#include <opencv2/opencv.hpp>

namespace gimbal_action {

class ResetPitch : public GimbalAction
{
public:    
    explicit ResetPitch(
        gimbal::GimbalController & gimbal_controller,
        double pitch_speed = 40.0,  // Speed of pitch movement in degrees per second
        double target_pitch = 0.0   // Target pitch angle to reset to (default is 0 degrees, which is horizontal)
    ):
        GimbalAction(gimbal_controller, "ResetPitch"),
        pitch_speed_(pitch_speed),
        target_pitch_(target_pitch)
    {}

    void run() override;

    double degree_tolerance_ = 15;  // Tolerance in degrees for stopping the action
    double target_pitch_;
    double pitch_speed_ = 15.0;     // Speed of pitch movement in degrees per second
};

}  // namespace gimbal
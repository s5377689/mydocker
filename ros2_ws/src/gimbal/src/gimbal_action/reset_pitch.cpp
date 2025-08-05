#include "gimbal_action/reset_pitch.hpp"

namespace gimbal_action
{

void ResetPitch::run()
{
    while (!halt_requested_)
    {
        auto gimbal_status = gimbal_controller_.getGimbalStatus();
        auto & pitch = gimbal_status.pitch;

        // Check if the pitch is within the tolerance
        if (std::abs(target_pitch_ - pitch) < degree_tolerance_)
        {
            // Build the command to stop gimbal rotation
            auto cmd = gimbal::buildYawPitchVelocityCommand(0.0, 0.0);
            gimbal_controller_.enqueueCommand(cmd);
            std::cout << "[ResetPitch] Action completed." << std::endl;
            return;
        }


        // Clamp function to ensure speed in the range [-pitch_speed_, pitch_speed_]
        auto clamp = [] (double val, double max_val) {
            return std::max(std::min(val, max_val), -max_val);
        };

        auto ensure_min_speed = [] (double speed, double min_speed) {
            if (std::abs(speed) < min_speed) {
                return (speed > 0) ? min_speed : -min_speed;
            }
            else
                return speed;
        };

        // Calculate the smoothed pitch speed
        double norm_pitch = pitch / 5.0;
        double square_pitch = std::pow(norm_pitch, 2);
        int sign = (pitch > 0) ? -1 : 1;
        double pitch_speed = sign * clamp(square_pitch * pitch_speed_, pitch_speed_);
        pitch_speed = ensure_min_speed(pitch_speed, 5.0);

        // Build the command to rotate the gimbal
        auto cmd = gimbal::buildYawPitchVelocityCommand(0.0, pitch_speed);
        gimbal_controller_.enqueueCommand(cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto cmd = gimbal::buildYawPitchVelocityCommand(0.0, 0.0);
    gimbal_controller_.enqueueCommand(cmd);
    std::cout << "[ResetPitch] Action halted." << std::endl;
}

}  // namespace gimbal_action
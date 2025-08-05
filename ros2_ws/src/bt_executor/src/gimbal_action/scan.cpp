#include <iostream>
#include <chrono>
#include "gimbal_action/scan.hpp"
#include "gimbal_action/helpers.hpp"


namespace gimbal_action
{

Scan::Scan(
    gimbal::usv::GimbalController & gimbal_controller,
    double yaw_from,
    double yaw_to,
    double speed
):
    GimbalAction(gimbal_controller, "Scan"),
    yaw_from_(yaw_from),
    yaw_to_(yaw_to),
    speed_(speed)
{}

void Scan::run()
{
    while (!halt_requested_)
    {
        auto gimbal_status = gimbal_controller_.getGimbalStatus();
        auto yaw = gimbal_status.yaw;

        // If start angle is outside the scan sector, firstly rotate into the range
        if (!is_in_clockwise_sector(yaw_from_, yaw_to_, yaw)) {
            from2to_ = clockwise_diff(yaw, yaw_from_) < 180;
            auto speed = from2to_ ? speed_ : -speed_;
            auto cmd = gimbal::buildYawPitchVelocityCommand(speed, 0);
            gimbal_controller_.enqueueCommand(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 5 Hz
            continue;
        }

        auto speed = from2to_ ? speed_ : -speed_;
        auto cmd = gimbal::buildYawPitchVelocityCommand(speed, 0);
        gimbal_controller_.enqueueCommand(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 5 Hz
    }

    std::cout << "[Scan] Action halted." << std::endl;

    // Stop the gimbal when the action is halted
    auto stop_cmd = gimbal::buildYawPitchVelocityCommand(0, 0);
    gimbal_controller_.enqueueCommand(stop_cmd);
}

}  // namespace gimbal_action
#pragma once
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "ardupilot_msgs/msg/global_position.hpp"

#include "gimbal_action/gimbal_action.hpp"
#include "gimbal/gimbal_controller.hpp"
#include "gimbal/helpers.hpp"

namespace gimbal_action
{

class Scan : public GimbalAction
{
public:    
    Scan(
        gimbal::GimbalController & gimbal_controller,
        double yaw_from,
        double yaw_to,
        double speed = 20
    );
    void run() override;

private:
    // Assume yaw_from < yaw_to (from-to clockwise convention)
    double yaw_from_;  
    double yaw_to_;    
    double speed_;     
    bool from2to_ {true};  // True if the current scan sector is from "from" to "to"; otherwise, is rotating backwards
};

}  // namespace gimbal_action
#include <iostream>
#include "gimbal_action/look_at.hpp"
#include "gimbal/helpers.hpp"


namespace gimbal_action
{

void LookAt::run()
{
    auto frame_width = GIMBAL_FRAME_WIDTH;
    auto frame_height = GIMBAL_FRAME_HEIGHT;

    cv::Point2i camera_center {
        frame_width / 2,
        frame_height / 2
    };

    auto gimbal_status = gimbal_controller_.getGimbalStatus();
    auto bbox_center = (bbox_.tl() + bbox_.br()) / 2;

    auto [yaw_t, pitch_t] = gimbal::offsetToAbsoluteAngles(
        bbox_center.x - camera_center.x,
        // camera_center.y - bbox_center.y,
        // ===== TEMP =====
        bbox_center.y - camera_center.y,
        // ===== TEMP =====
        gimbal_status.zoom,
        gimbal_status.yaw,
        gimbal_status.pitch,
        frame_width,
        frame_height,
        GIMBAL_FOV_X,
        GIMBAL_FOV_Y
    );

    // ===== TEMP =====
    pitch_t -= 180.0f;
    while (pitch_t < -180.0f) {
        pitch_t += 360.0f;
    } 
    // ===== TEMP =====
    int16_t yaw_int = static_cast<int16_t>(std::round(yaw_t * 10));
    int16_t pitch_int = static_cast<int16_t>(std::round(pitch_t * 10));
    
    auto cmd = gimbal::buildRotateToCommand(yaw_int, pitch_int);
    gimbal_controller_.enqueueCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "[LookAt] Action completed." << std::endl;
}

}  // namespace gimbal_action
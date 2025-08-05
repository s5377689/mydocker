#include <iostream>
#include "gimbal_action/look_at.hpp"
#include "gimbal/helpers.hpp"


namespace gimbal_action
{

void LookAt::run()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto bbox_center = (bbox_.tl() + bbox_.br()) / 2;
    auto frame_width = GIMBAL_FRAME_WIDTH;
    auto frame_height = GIMBAL_FRAME_HEIGHT;

    cv::Point2i camera_center {
        frame_width / 2,
        frame_height / 2
    };

    auto gimbal_status = gimbal_controller_.getGimbalStatus();
    auto & zoom = gimbal_status.zoom;

    auto [yaw_t, pitch_t] = gimbal::offsetToAbsoluteAngles(
        bbox_center.x - camera_center.x,
        // camera_center.y - bbox_center.y,
        // ===== TEMP =====
        bbox_center.y - camera_center.y,
        zoom,
        gimbal_status.yaw,
        gimbal_status.pitch,
        frame_width,
        frame_height
    );

    // ===== TEMP =====
    pitch_t -= 180.0f;
    while (pitch_t < -180.0f) {
        pitch_t += 360.0f;
    } 
    // ===== TEMP =====

    std::cout << "[LookAt] Current gimbal status: "
              << "yaw = " << gimbal_status.yaw
              << ", pitch = " << gimbal_status.pitch
              << ", zoom = " << zoom << std::endl;

    std::cout << "[LookAt] Target angles: yaw = " << static_cast<int16_t>(yaw_t)
              << ", pitch = " << static_cast<int16_t>(pitch_t)
              << std::endl;
    
    auto cmd = gimbal::buildRotateToCommand(
        static_cast<int16_t>(yaw_t),
        static_cast<int16_t>(pitch_t)
    );
    gimbal_controller_.enqueueCommand(cmd);
}

}  // namespace gimbal_action
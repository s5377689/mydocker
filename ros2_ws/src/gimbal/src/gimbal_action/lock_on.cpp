#include <iostream>
#include "gimbal_action/lock_on.hpp"
#include "gimbal/helpers.hpp"
#include "gimbal/usv/gimbal_controller.hpp"  // Include the full definition


namespace gimbal_action
{

void LockOn::run()
{       
    cv::Point2i camera_center {
        GIMBAL_FRAME_WIDTH / 2,
        GIMBAL_FRAME_HEIGHT / 2
    };

    while (!halt_requested_)
    {
        auto gimbal_status = gimbal_controller_.getGimbalStatus();
        auto bbox = gimbal_controller_.get_lockon_bbox();
        auto bbox_center = (bbox.tl() + bbox.br()) / 2;

        auto [yaw_t, pitch_t] = gimbal::offsetToAbsoluteAngles(
            bbox_center.x - camera_center.x,
            // camera_center.y - bbox_center.y,
            // ===== TEMP =====
            bbox_center.y - camera_center.y,
            // ===== TEMP =====
            gimbal_status.zoom,
            gimbal_status.yaw,
            gimbal_status.pitch,
            GIMBAL_FRAME_WIDTH,
            GIMBAL_FRAME_HEIGHT,
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[LockOn] Action halted." << std::endl;
}

}  // namespace gimbal_action
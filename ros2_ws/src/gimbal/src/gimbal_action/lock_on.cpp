#include <iostream>
#include "gimbal_action/lock_on.hpp"
#include "gimbal/helpers.hpp"
#include "gimbal/usv/gimbal_controller.hpp"  // Include the full definition

using std::cout;
using std::cerr;
using std::endl;

namespace gimbal_action
{

void LockOn::run()
{
    while (!halt_requested_)
    {
        auto frame_width = GIMBAL_FRAME_WIDTH;
        auto frame_height = GIMBAL_FRAME_HEIGHT;

        cv::Point2i camera_center {
            frame_width / 2,
            frame_height / 2
        };

        auto bbox = gimbal_controller_.getCurrentBBox();
        auto gimbal_status = gimbal_controller_.getGimbalStatus();
        auto & yaw = gimbal_status.yaw;
        auto & pitch = gimbal_status.pitch;
        auto & zoom = gimbal_status.zoom;

        auto bbox_center = (bbox.tl() + bbox.br()) / 2;
        auto [yaw_t, pitch_t] = gimbal::offsetToAbsoluteAngles(
            bbox_center.x - camera_center.x,
            camera_center.y - bbox_center.y,
            zoom,
            yaw,
            pitch,
            frame_width,
            frame_height
        );

        auto cmd = gimbal::buildRotateToCommand(
            static_cast<int16_t>(yaw_t),
            static_cast<int16_t>(pitch_t)
        );
        gimbal_controller_.enqueueCommand(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 5 Hz
    }
}

}  // namespace gimbal_action
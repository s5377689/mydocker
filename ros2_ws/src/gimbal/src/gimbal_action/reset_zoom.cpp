#include "gimbal_action/reset_zoom.hpp"
#include "gimbal/helpers.hpp"

namespace gimbal_action
{

void ResetZoom::run()
{
    while (!halt_requested_)
    {
        auto gimbal_status = gimbal_controller_.getGimbalStatus();
        auto & zoom = gimbal_status.zoom;

        // Check if the zoom is within the tolerance
        if (zoom < (1 + zoom_tolerance_))
        {
            // Build the command to stop gimbal zooming
            auto cmd = gimbal::buildManualZoomAutoFocusCommand(0);  // Stop zooming command
            gimbal_controller_.enqueueCommand(cmd);
            std::cout << "[ResetZoom] Action completed." << std::endl;
            return;
        }
        else {
            auto cmd = gimbal::buildManualZoomAutoFocusCommand(-1);  // Zoom out command
            gimbal_controller_.enqueueCommand(cmd);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "[ResetZoom] Action halted." << std::endl;
    auto cmd = gimbal::buildManualZoomAutoFocusCommand(0);
    gimbal_controller_.enqueueCommand(cmd);
}

}  // namespace gimbal_action
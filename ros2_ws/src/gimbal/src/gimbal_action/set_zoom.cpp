#include "gimbal_action/set_zoom.hpp"
#include "gimbal/helpers.hpp"

namespace gimbal_action
{

void SetZoom::run()
{
    auto gimbal_status = gimbal_controller_.getGimbalStatus();
    auto & zoom = gimbal_status.zoom;
    auto zoom_dif = target_zoom_ - zoom;

    if (std::abs(zoom_dif) < zoom_tolerance_)
    {
        std::cout << "[SetZoom] Action completed." << std::endl;
        return;
    }
    else if (zoom_dif > 0)
    {
        // Zoom in
        auto cmd = gimbal::buildManualZoomAutoFocusCommand(1);  // Zoom in command
        gimbal_controller_.enqueueCommand(cmd);
    }
    else
    {
        // Zoom out
        auto cmd = gimbal::buildManualZoomAutoFocusCommand(-1);  // Zoom out command
        gimbal_controller_.enqueueCommand(cmd);
    }
    
    while (!halt_requested_)
    {
        auto gimbal_status = gimbal_controller_.getGimbalStatus();
        auto & zoom = gimbal_status.zoom;
        auto zoom_dif = target_zoom_ - zoom;

        // Check if the zoom is within the tolerance
        if (std::abs(zoom_dif) < zoom_tolerance_)
        {
            // Build the command to stop gimbal zooming
            auto cmd = gimbal::buildManualZoomAutoFocusCommand(0);  // Stop zooming command
            gimbal_controller_.enqueueCommand(cmd);
            std::cout << "[ResetZoom] Action completed." << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

}  // namespace gimbal_action
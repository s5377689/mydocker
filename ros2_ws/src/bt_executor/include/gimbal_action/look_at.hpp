#pragma once
#include "gimbal_action/gimbal_action.hpp"
#include "gimbal/gimbal_controller.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

namespace gimbal_action
{

class LookAt : public GimbalAction
{
public:    
    LookAt(
        gimbal::GimbalController & gimbal_controller,
        cv::Rect bbox  // (0, 0) is the top-left corner of the frame
    ):
        GimbalAction(gimbal_controller, "LookAt"),
        bbox_(bbox)
    {}

    LookAt(
        gimbal::GimbalController & gimbal_controller,
        cv::Point2i point  // (0, 0) is the top-left corner of the frame
    ):
        GimbalAction(gimbal_controller, "LookAt"),
        bbox_(cv::Rect(point.x, point.y, 1, 1))  // Create a 1x1 bounding box at the point
    {}

    void run() override;

private:
    cv::Rect bbox_;  // The bounding box to look at, initialized in the constructor
};

}  // namespace gimbal_action
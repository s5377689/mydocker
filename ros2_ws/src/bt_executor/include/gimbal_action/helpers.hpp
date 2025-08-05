#pragma once

namespace gimbal_action
{
    double clockwise_diff(
        double from_deg, 
        double to_deg
    );

    bool is_in_clockwise_sector(
        double from_deg, 
        double to_deg, 
        double test_deg
    );
}
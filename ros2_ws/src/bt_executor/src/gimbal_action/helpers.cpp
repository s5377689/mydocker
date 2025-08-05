#include "gimbal_action/helpers.hpp"


namespace gimbal_action
{
    double clockwise_diff(
        double from_deg, 
        double to_deg)
    {
        double diff = to_deg - from_deg;
        while (diff < 0)
            diff += 360.0;
        while (diff >=360.0)
            diff -= 360.0;
        return diff;
    }

    bool is_in_clockwise_sector(
        double from_deg, 
        double to_deg, 
        double test_deg)
    {
        double full_sector = clockwise_diff(from_deg, to_deg);
        double test_sector = clockwise_diff(from_deg, test_deg);
        return test_sector <= full_sector;
    }
}  // namespace gimbal_action
#pragma once
#include <cstdint>

enum class ArduPilotMode : uint8_t
{
    MANUAL       = 0,
    ACRO         = 1,
    STEERING     = 3,
    HOLD         = 4,
    LOITER       = 5,
    FOLLOW       = 6,
    SIMPLE       = 7,
    DOCK         = 8,
    CIRCLE       = 9,
    AUTO         = 10,
    RTL          = 11,
    SMART_RTL    = 12,
    GUIDED       = 15,
    INITIALISING = 16,
    OFFBOARD     = 30   // Reserved for external/lua control
};
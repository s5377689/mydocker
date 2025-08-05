#pragma once
#include <cstdint>

// Gimbal parameters of SiYi ZR10
#define GIMBAL_FRAME_WIDTH 1920
#define GIMBAL_FRAME_HEIGHT 1080
#define GIMBAL_FOV_X 61.5
#define GIMBAL_FOV_Y 36.8
#define GIMBAL_MIN_YAW -160.
#define GIMBAL_MAX_YAW 160.
#define GIMBAL_MAX_PITCH 25.
#define GIMBAL_MIN_PITCH -90.

namespace gimbal
{

struct GimbalStatus {
    float yaw = 0.0f;               // deg
    float pitch = 0.0f;             // deg
    float roll = 0.0f;              // deg
    float yaw_velocity = 0.0f;      // deg/s
    float pitch_velocity = 0.0f;    // deg/s
    float roll_velocity = 0.0f;     // deg/s
    float zoom = 0.0f;              // x
};

}  // namespace gimbal
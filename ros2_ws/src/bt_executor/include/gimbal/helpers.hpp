#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

#include "gimbal/gimbal_status.hpp"


namespace gimbal
{
// -----------------------------------------------
//             ZR10 CRC16 Calculation
// ----------------------------------------------- 
extern const uint16_t crc16_tab[256];
uint16_t CRC16_cal(
    uint8_t *ptr, uint32_t len, uint16_t crc_init
);
uint8_t crc_check_16bites(
    uint8_t* pbuf, uint32_t len,uint32_t* p_result
);

// -----------------------------------------------
//              ZR10 Gimbal Commands
// -----------------------------------------------
std::vector<uint8_t> buildHeartbeatCommand();
std::vector<uint8_t> buildYawPitchVelocityCommand(int8_t yaw, int8_t pitch);
std::vector<uint8_t> buildRotateToCommand(int16_t yaw, int16_t pitch);
std::vector<uint8_t> buildManualZoomAutoFocusCommand(int8_t zoom_type);
std::vector<uint8_t> buildZoomCommand(uint8_t zoom_int, uint8_t zoom_frac);
std::vector<uint8_t> buildResetPoseCommand();

// ---- buildRequestStatusStreamCommand ----
// data_type: 1: pose, 2: lazer-measured distance
// data_freq:
// 0: close  1: 2 Hz  2: 4 Hz  3: 5 Hz
// 4: 10 Hz  5: 20 Hz 6: 50 Hz 7: 100 Hz
std::vector<uint8_t> buildRequestStatusStreamCommand(uint8_t data_type, uint8_t data_freq);
std::vector<uint8_t> buildRequestPoseCommand();
std::vector<uint8_t> buildRequestZoomLevelCommand();


// -----------------------------------------------
//                  Coordinates
// -----------------------------------------------
std::pair<double, double> offsetToRelativeAngles(
    double dx,
    double dy,
    double zoom_factor,
    double frame_width = GIMBAL_FRAME_WIDTH,
    double frame_height = GIMBAL_FRAME_HEIGHT,
    double fov_x = GIMBAL_FOV_X,
    double fov_y = GIMBAL_FOV_Y
);
std::pair<double, double> offsetToAbsoluteAngles(
    double dx,
    double dy,
    double zoom_factor,
    double cur_pan,
    double cur_pitch,
    double frame_width = GIMBAL_FRAME_WIDTH,
    double frame_height = GIMBAL_FRAME_HEIGHT,
    double fov_x = GIMBAL_FOV_X,
    double fov_y = GIMBAL_FOV_Y
);

// -----------------------------------------------
//                     Image
// -----------------------------------------------
void resize_compress_then_publish(
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & pub,
    const cv::Mat & img,
    float scale = 0.25,
    const std::string & encoding = "jpeg",
    int quality = 75
);

}  // namespace gimbal
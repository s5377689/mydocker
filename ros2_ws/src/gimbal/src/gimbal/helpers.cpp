#include "gimbal/helpers.hpp"

using namespace std;


namespace gimbal
{
// -----------------------------------------------
//             ZR10 CRC16 Calculation
// ----------------------------------------------- 
const uint16_t crc16_tab[256]=
{
    0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
};

//   CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init)
{
    uint16_t crc,
    oldcrc16;
    uint8_t temp;
    crc = crc_init;
    while (len--!=0)
    {
        temp=(crc>>8)&0xff;
        oldcrc16=crc16_tab[*ptr^temp];
        crc=(crc<<8)^oldcrc16;
        ptr++;
    }
    //crc=~crc;
    return(crc);
}

uint8_t crc_check_16bites(uint8_t* pbuf, uint32_t len,uint32_t* p_result)
{
    uint16_t crc_result = 0;
    crc_result= CRC16_cal(pbuf,len, 0);
    *p_result = crc_result;
    return 2;
}

// -----------------------------------------------
//              ZR10 Gimbal Commands
// ----------------------------------------------- 
std::vector<uint8_t> buildHeartbeatCommand()
{
    std::vector<uint8_t> packet = {
        0x55, 0x66,   // STX
        0x01,         // CTRL (need_ack)
        0x01, 0x00,   // DataLen (little-endian, 1 byte)
        0x00, 0x00,   // SEQ = 0
        0x00,         // CMD_ID (Heartbeat)
        0x00,         // DATA
        0x59, 0x8B    // CRC16
    };
    return packet;
}

std::vector<uint8_t> buildYawPitchVelocityCommand(
    int8_t yaw,
    int8_t pitch)
{
    /*
        - yaw: horizontal-axis speed [-100, 100]
        - pitch: vertical-axis speed [-100, 100]
    */
    std::vector<uint8_t> packet = {
        0x55, 0x66,   // STX (little-endian)
        0x00,         // CTRL
        0x02, 0x00,   // DataLen = 2 bytes (little-endian)
        0x00, 0x00,   // SEQ = 0
        0x07          // CMD_ID (Yaw/Pitch command)
    };
    packet.push_back(yaw);    // DATA (yaw value)
    packet.push_back(pitch);  // DATA (pitch value)

    // CRC16-CCITT over all previous bytes
    uint16_t crc = CRC16_cal(packet.data(), packet.size(), 0);
    packet.push_back(crc & 0xFF);  // Append low byte
    packet.push_back((crc >> 8) & 0xFF);  // Append high byte

    return packet;
}

std::vector<uint8_t> buildRotateToCommand(
    int16_t yaw,
    int16_t pitch)
{
    /*
        - yaw: horizontal-axis degree [-135.0, 135.0]
        - pitch: vertical-axis degree [-90.0, 25.0]
    */
    std::vector<uint8_t> packet = {
        0x55, 0x66,   // STX (little-endian)
        0x01,         // CTRL
        0x04, 0x00,   // DataLen = 4 bytes (little-endian)
        0x00, 0x00,   // SEQ = 0
        0x0E          // CMD_ID (Yaw/Pitch command)
    };
    packet.push_back(static_cast<uint8_t>(yaw & 0xFF));             // DATA (yaw low byte)
    packet.push_back(static_cast<uint8_t>((yaw >> 8) & 0xFF));      // DATA (yaw high byte)
    packet.push_back(static_cast<uint8_t>(pitch & 0xFF));           // DATA (pitch low byte)
    packet.push_back(static_cast<uint8_t>((pitch >> 8) & 0xFF));    // DATA (pitch high byte)

    // CRC16-CCITT over all previous bytes
    uint16_t crc = CRC16_cal(packet.data(), packet.size(), 0);
    packet.push_back(crc & 0xFF);  // Append low byte
    packet.push_back((crc >> 8) & 0xFF);  // Append high byte

    return packet;
}

std::vector<uint8_t> buildManualZoomAutoFocusCommand(
    int8_t zoom_type)
{
    std::vector<uint8_t> packet = {
        0x55, 0x66,                 // STX
        0x00,                       // CTRL 
        0x01, 0x00,                 // DataLen = 1 byte
        0x00, 0x00,                 // SEQ = 0
        0x05,                       // CMD_ID (Manual Zoom/Auto Focus)
    };
    packet.push_back(zoom_type);    // DATA (-1: decrese zoom level, 1: increase zoom level, 0: stop zooming)

    uint16_t crc = CRC16_cal(packet.data(), packet.size(), 0);
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    return packet;
}

std::vector<uint8_t> buildZoomCommand(
    uint8_t zoom_int,
    uint8_t zoom_frac)
{
    std::vector<uint8_t> packet = {
        0x55, 0x66,   // STX
        0x00,         // CTRL
        0x02, 0x00,   // DataLen = 2 bytes
        0x00, 0x00,   // SEQ = 0
        0x0F,         // CMD_ID
        zoom_int,
        zoom_frac
    };

    uint16_t crc = CRC16_cal(packet.data(), packet.size(), 0);
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    return packet;
}

std::vector<uint8_t> buildResetPoseCommand()
{
    std::vector<uint8_t> packet = {
        0x55, 0x66,   // STX
        0x00,         // CTRL (set to 0x01 for response)
        0x01, 0x00,   // DataLen = 1 byte
        0x00, 0x00,   // SEQ = 0
        0x08          // CMD_ID
    };
    packet.push_back(0x01);  // DATA

    uint16_t crc = CRC16_cal(packet.data(), packet.size(), 0);
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    return packet;
}

std::vector<uint8_t> buildRequestStatusStreamCommand(
    uint8_t data_type, uint8_t data_freq)
{
    std::vector<uint8_t> packet = {
        0x55, 0x66,   // STX
        0x01,         // CTRL
        0x02, 0x00,   // DataLen = 2 bytes
        0x00, 0x00,   // SEQ = 0
        0x25,         // CMD_ID
        data_type,    // DATA (data type)
        data_freq     // DATA (data frequency)
    };

    uint16_t crc = CRC16_cal(packet.data(), packet.size(), 0);
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);
    return packet;
}

std::vector<uint8_t> buildRequestPoseCommand()
{
    std::vector<uint8_t> packet {
        0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0d, 0xe8, 0x05
    };
    return packet;
}

std::vector<uint8_t> buildRequestZoomLevelCommand()
{
    // 55 66 01 00 00 00 00 18 7C 47
    std::vector<uint8_t> packet {
        0x55, 0x66,   // STX
        0x01,         // CTRL
        0x00, 0x00,   // DataLen = 0 bytes
        0x00, 0x00,   // SEQ = 0
        0x18,         // CMD_ID (Request Zoom Level)
        0x7C, 0x47    // CRC16 (calculated value)
    };
    return packet;
}

// -----------------------------------------------
//                  Coordinates
// -----------------------------------------------
std::pair<double, double> offsetToRelativeAngles(
    double dx,
    double dy,
    double zoom_factor,
    double frame_width,
    double frame_height,
    double fov_x,
    double fov_y)
{
    /*  
        - dx: x offset to the center of camera in pixel
        - dy: y offset to the center of camera in pixel
        - fov_x: field of view
    */
    // Adjust FOV based on zoom
    fov_x /= zoom_factor;
    fov_y /= zoom_factor;

    // Degrees per pixel
    double deg_per_pixel_x = fov_x / frame_width;
    double deg_per_pixel_y = fov_y / frame_height;

    // Compute angular offsets
    double pan_angle = dx * deg_per_pixel_x;
    double pitch_angle = dy * deg_per_pixel_y;

    return {pan_angle, pitch_angle};
}

std::pair<double, double> offsetToAbsoluteAngles(
    double dx,
    double dy,
    double zoom_factor,
    double cur_pan,
    double cur_pitch,
    double frame_width,
    double frame_height,
    double fov_x,
    double fov_y)
{
    // Adjust FOV based on zoom
    fov_x /= zoom_factor;
    fov_y /= zoom_factor;

    // Degrees per pixel
    double deg_per_pixel_x = fov_x / frame_width;
    double deg_per_pixel_y = fov_y / frame_height;

    // Compute angular offsets
    double pan_angle = dx * deg_per_pixel_x;
    double pitch_angle = dy * deg_per_pixel_y;

    return {cur_pan + pan_angle, cur_pitch + pitch_angle};
}

// -----------------------------------------------
//                     Image
// -----------------------------------------------
void resize_compress_then_publish(
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & pub,
    const cv::Mat & img,
    float scale,
    const std::string & encoding,
    int quality)
{
    if (img.empty())
    {
        cerr << "Image is empty, cannot publish" << endl;
        return;
    }

    // Resize the image if scale > 0
    cv::Mat resized;
    if (scale > 0)
        cv::resize(img, resized, cv::Size(), scale, scale);
    else
        resized = img;

    // Encode the image
    std::vector<uchar> buffer; 
    std::vector<int> params;
    if (encoding == "jpeg")
    {
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(quality);
    }
    else if (encoding == "png")
    {
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(3);
    }
    else
    {
        cerr << "Unsupported encoding: " << encoding << endl;
        return;
    }

    std::string ext = (encoding == "jpeg") ? ".jpg" : ".png";

    if (!cv::imencode(ext, resized, buffer, params))
    {
        cerr << "Failed to encode image as " << encoding << endl;
        return;
    }

    // Fill and publish the message
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.format = encoding;
    msg.data = std::move(buffer);
    pub->publish(msg);
}

}  // namespace gimbal

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

    double normalize_angle(
        double angle_deg)
    {
        // Normalize angle to [-180, 180) degrees
        while (angle_deg < -180.0)
            angle_deg += 360.0;
        while (angle_deg >= 180.0)
            angle_deg -= 360.0;
        return angle_deg;
    }
}  // namespace gimbal_action
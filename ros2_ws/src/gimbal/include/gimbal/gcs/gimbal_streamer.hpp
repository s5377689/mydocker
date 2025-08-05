#pragma once

#include <atomic>
#include <thread>
#include <mutex>
#include <memory>
#include <string>
#include <iostream>

#include <QObject>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

extern "C" {
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

#include "gimbal/gcs/gimbal_image_provider.hpp"
#include "gimbal/helpers.hpp"


namespace gimbal::gcs
{

class GimbalStreamer : public QObject
{
    Q_OBJECT

public:
    GimbalStreamer(
        QObject * parent,
        const std::string & rtsp_url,
        const std::string & topic_name = "/camera/compressed/image_raw",
        float resize_factor = 0.25f
    );
    ~GimbalStreamer();

    void setImageProvider(GimbalImageProvider * provider);
    Q_INVOKABLE void start();
    Q_INVOKABLE inline bool isConnected() { return connected_; }
    Q_INVOKABLE inline void enablePublish() { publish_frame_ = true; }
    Q_INVOKABLE inline void disablePublish() { publish_frame_ = false; }
    Q_INVOKABLE void stop();
    bool getLatestFrame(cv::Mat & out_frame);

    std::atomic<bool> publish_frame_ {true};
    std::atomic<int> publish_rate_ {10}; // in Hz

private:
    void captureLoop();
    
    GimbalImageProvider * img_provider_ {nullptr};
    std::string rtsp_url_;
    bool running_, connected_;
    std::thread capture_thread_;
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr img_pub_;
    float resize_factor_;
};

}  // namespace gimbal::gcs
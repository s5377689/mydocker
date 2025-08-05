#pragma once
#include <condition_variable>
#include <memory>
#include <iostream>
#include <utility>
#include <map>
#include <functional>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <QObject>

#include "gimbal/helpers.hpp"
#include "gimbal/gimbal_status.hpp"


namespace gimbal::gcs
{

class GimbalController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float yaw READ yaw NOTIFY gimbalStatusChanged)
    Q_PROPERTY(float pitch READ pitch NOTIFY gimbalStatusChanged)
    Q_PROPERTY(float yaw_velocity READ yaw_velocity NOTIFY gimbalStatusChanged)
    Q_PROPERTY(float pitch_velocity READ pitch_velocity NOTIFY gimbalStatusChanged)
    Q_PROPERTY(float zoom_level READ zoom_level NOTIFY gimbalStatusChanged)
    Q_PROPERTY(bool connected READ isConnected NOTIFY connectionChanged)

signals:
    void gimbalStatusChanged();
    void connectionChanged();

public:
    GimbalController(QObject *parent, const std::string & ip, uint16_t port);
    ~GimbalController();

    // GimbalController lifecycle management
    Q_INVOKABLE void start();
    Q_INVOKABLE void stop();
    inline bool isConnected() const { return connected_; }

    // ------------------------------
    //     QML Control interface
    // ------------------------------
    Q_INVOKABLE void enqueueCommand(const std::vector<uint8_t> & cmd);

    Q_INVOKABLE void onImageLeftClicked(int x, int y, int img_width, int img_height);

    Q_INVOKABLE inline void rotateTo(float yaw, float pitch) {
        int16_t yaw_int = static_cast<int16_t>(std::round(yaw * 10));
        int16_t pitch_int = static_cast<int16_t>(std::round(pitch * 10));
        auto cmd = gimbal::buildRotateToCommand(yaw_int, pitch_int);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void zoomIn() {
        auto cmd = gimbal::buildManualZoomAutoFocusCommand(1);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void zoomOut() {
        auto cmd = gimbal::buildManualZoomAutoFocusCommand(-1);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void stopZoom() {
        auto cmd = gimbal::buildManualZoomAutoFocusCommand(0);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void up(int8_t velocity = 10) {
        auto cmd = gimbal::buildYawPitchVelocityCommand(0, velocity);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void down(int8_t velocity = 10) {
        auto cmd = gimbal::buildYawPitchVelocityCommand(0, -velocity);
        enqueueCommand(cmd); 
    }
    Q_INVOKABLE inline void stopRotation() {
        auto cmd = gimbal::buildYawPitchVelocityCommand(0, 0);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void left(int8_t velocity = 10) {
        auto cmd = gimbal::buildYawPitchVelocityCommand(-velocity, 0);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void right(int8_t velocity = 10) {
        auto cmd = gimbal::buildYawPitchVelocityCommand(velocity, 0);
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void resetPose() {
        auto cmd = gimbal::buildResetPoseCommand();
        enqueueCommand(cmd);
    }
    Q_INVOKABLE inline void setZoom(uint8_t zoom_int, uint8_t zoom_frac) {
        auto cmd = gimbal::buildZoomCommand(zoom_int, zoom_frac);
        enqueueCommand(cmd);
    }

    // Getters
    inline gimbal::GimbalStatus getGimbalStatus() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return gimbal_status_;
    }
    inline cv::Rect getCurrentBBox() {
        std::lock_guard<std::mutex> lock(current_bbox_mutex_);
        return current_bbox_;
    }

    // Setters
    void updateBBox(cv::Rect bbox);  // for lock on a target

private:
    void receiveLoop();
    void sendLoop();
    // void actionLoop();

    bool reconnect(int n_retry);  // called by receive_thread_ if connection is lost
    bool connectToGimbal();
    void parseStreamBuffer();
    void parseAndStorePose(const std::vector<uint8_t> & packet);
    void parseAndStoreZoomLevel(const std::vector<uint8_t> & packet);

    // Socket connection parameters
    std::string ip_;
    int port_;
    int sockfd_;
    std::vector<uint8_t> stream_buffer_;

    // Thread management
    std::atomic<bool> running_;             // Indicates if the client is running
    std::atomic<bool> connected_;           // Indicates if the client is connected to the gimbal
    std::thread receive_thread_;            // Thread for receiving status updates
    std::thread send_thread_;               // Thread for sending commands to gimbal
    // std::thread action_thread_;             // Thread for executing high-level actions

    // Command queue for sending commands to gimbal
    std::mutex send_queue_mutex_;
    std::condition_variable send_cv_;
    std::queue<std::vector<uint8_t>> send_queue_;

    std::mutex current_bbox_mutex_;
    cv::Rect current_bbox_;  // To enable lock on a target

    std::mutex status_mutex_;
    gimbal::GimbalStatus gimbal_status_;

    // -----------------------------------
    //     Heart beat sending thread
    // -----------------------------------
    void heartbeatLoop(uint8_t hertz);  // Send heartbeat command at specified hertz
    std::thread heartbeat_thread_;

    // -----------------------------------
    //    Gimbal status request thread 
    // -----------------------------------
    // Since RequestGimbalStatusStreamCommand somehow doesn't work, we manually send the request command periodically
    void requestStatusLoop(uint8_t hertz);
    std::thread request_status_thread_;

    // -------------------------
    //    Q_PROPERTY getters
    // -------------------------
    float yaw() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return gimbal_status_.yaw;
    }
    float pitch() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        float pitch = gimbal_status_.pitch;
        pitch = 180 - pitch;
        while (pitch > 180.0f) {
            pitch -= 360.0f;  // Normalize pitch to [-180, 180]
        }
        return pitch;
    }
    float yaw_velocity() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return gimbal_status_.yaw_velocity;
    }
    float pitch_velocity() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return -gimbal_status_.pitch_velocity;
    }
    float zoom_level() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return gimbal_status_.zoom;
    }
};

}  // namespace gimbal::gcs
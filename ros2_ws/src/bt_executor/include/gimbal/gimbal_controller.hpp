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

#include "gimbal/helpers.hpp"
#include "gimbal/gimbal_status.hpp"
#include "gimbal_action/gimbal_action.hpp"


namespace gimbal
{

class GimbalController 
{
public:
    GimbalController(const std::string & ip, uint16_t port);
    ~GimbalController();

    // GimbalController lifecycle management
    void start();
    bool isConnected() const { return connected_; }
    void stop();

    // -------------------------
    //     Control interface
    // -------------------------
    void enqueueCommand(const std::vector<uint8_t> & cmd);

    void enqueueAction(std::shared_ptr<gimbal_action::GimbalAction> action);
    void interruptCurrentAction();
    void stopAndClearAllActions();
    void replaceWithAction(std::shared_ptr<gimbal_action::GimbalAction> action);

    void printActionQueue() {
        std::lock_guard<std::mutex> lock(action_mutex_);
        std::cout << "[GimbalController] Current action queue size: " << action_queue_.size() << std::endl;
        for (const auto & action : action_queue_) {
            std::cout << "   - " << action->getName() << std::endl;
        }
    }
    void printCurrentAction() {
        std::lock_guard<std::mutex> lock(action_mutex_);
        if (current_action_) {
            std::cout << "[GimbalController] Current action: " << current_action_->getName() << std::endl;
        } else {
            std::cout << "[GimbalController] No action running." << std::endl;
        }
    }
    // -------------------------

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
    void updateBBox(cv::Rect bbox);  // for locking on a target

private:
    void receiveLoop();
    void sendLoop();
    void actionLoop();

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
    std::thread action_thread_;             // Thread for executing high-level actions

    // Command queue for sending commands to gimbal
    std::mutex send_queue_mutex_;
    std::condition_variable send_cv_;
    std::queue<std::vector<uint8_t>> send_queue_;

    // Action management
    std::mutex action_mutex_;
    std::condition_variable action_cv_;
    std::deque<std::shared_ptr<gimbal_action::GimbalAction>> action_queue_;
    std::shared_ptr<gimbal_action::GimbalAction> current_action_;

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
};

}  // namespace gimbal
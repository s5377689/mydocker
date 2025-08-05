#pragma once

#include <unordered_map>
#include <thread>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/tracking.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "bt_action/bt_node.hpp"
#include "bt_action/common_helpers.hpp"
#include "bt_action/search_helpers.hpp"
#include "system_component/target_registry.hpp"
#include "gimbal/usv/gimbal_streamer.hpp"
#include "gimbal/usv/gimbal_controller.hpp"
#include "gimbal/gimbal_status.hpp"
#include "gimbal/helpers.hpp"

namespace bt_action
{

class Search : public BtNode
{
public:
    Search(
        const std::string & name,
        const BT::NodeConfiguration & config
    );
    ~Search();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("target_id"),
        };
    }

    void initialize(
        std::shared_ptr<system_component::TargetRegistry> target_registry,
        std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer,
        std::shared_ptr<gimbal::usv::GimbalController> gimbal_controller
    );

    void transfer_to(
        SearchState to_state,
        const std::string & transition_name
    );

    // ---------------------------------
    //  Parameters setters and getters
    // ---------------------------------
    int scan_time_ms() const { return scan_time_ms_; }
    void set_scan_time_ms(int time_ms) { scan_time_ms_ = time_ms; }
    float scan_from() const { return scan_from_; }
    void set_scan_from(float from) { scan_from_ = from; }
    float scan_to() const { return scan_to_; }
    void set_scan_to(float to) { scan_to_ = to; }
    int final_vote_round() const { return final_vote_round_; }
    void set_final_vote_round(int rounds) { final_vote_round_ = rounds; }
    int winner_threshold() const { return winner_threshold_; }
    void set_winner_threshold(int threshold) { winner_threshold_ = threshold; }

protected:
    void run_action_() override;

private:
    // Hash function for the state tuple to be used as a key to fetch the transition function 
    struct StateTupleHash
    {
        std::size_t operator() (const std::tuple<SearchState, SearchState, std::string> & t) const
        {
            return std::hash<int>()(static_cast<int>(std::get<0>(t))) ^
                (std::hash<int>()(static_cast<int>(std::get<1>(t))) << 1) ^
                (std::hash<std::string>()(std::get<2>(t)) << 2);
        }
    };

    // ----------------------------
    //      Thread management
    // ----------------------------
    void processLoop();  // Loop for processing the input frame
    void publishLoop();  // Loop for publishing the current searching state
    void stop();         // Stop all the threads and clean up resources
    std::thread process_thread_;
    std::thread publish_thread_;

    // --------------------------------------------------------------
    // Processing loops for different stages of searching the target 
    // Implemented as a finite state machine (FSM) with four states
    // --------------------------------------------------------------
    void process_scanning(
        cv::Mat & img
    );
    void process_detecting(
        cv::Mat & img
    );
    void process_recognizing(
        cv::Mat & img
    );
    void process_tracking(
        cv::Mat & img
    );

    // Transition functions for FSM
    void define_transition_funcs();
    std::unordered_map<
        std::tuple<SearchState, SearchState, std::string>,
        std::function<void(void)>,
        StateTupleHash
    > transition_funcs_;

    SearchState search_state_ {SearchState::Start};
    std::atomic<bool> target_found_ {false};
    std::chrono::duration<double> img_timeout_ {3.0};  // Timeout for obtaining input frame

    // Scanning state parameters
    std::chrono::time_point<std::chrono::steady_clock> scan_start_ms_;
    int scan_time_ms_ {5000};  // Time to stay in the Scanning state before start blob detection
    float scan_from_ {GIMBAL_MIN_YAW};
    float scan_to_ {GIMBAL_MAX_YAW};

    // Detecting state parameters
    int vote_round_ {0};  // Current round of voting
    int final_vote_round_ {10}; // Number of rounds to accumulate votes 
    int winner_threshold_ {8}; // Threshold for considering a blob as a possible target
    
    // Recognizing state parameters
    std::chrono::time_point<std::chrono::steady_clock> lockon_start_ms_;
    int lockon_time_ms_ {2000}; // Time to lock on the target before a second SIFT check

    // Tracking state parameters
    std::chrono::time_point<std::chrono::steady_clock> track_start_ms_;
    int tracking_time_ms_ {5000}; // Time to track and record if the target is found

    // --------------------------------
    //   Gimbal Streaming and Control
    // --------------------------------
    std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer_;
    std::shared_ptr<gimbal::usv::GimbalController> gimbal_controller_;

    // --------------------------------
    //        Target Registry
    // --------------------------------
    std::shared_ptr<system_component::TargetRegistry> target_registry_;
    system_component::TargetData target_data_;  // Data of the target to search for

    // --------------------------------
    //      Searching Algorithms
    // --------------------------------
    // Parameters for sea horizon detection
    int upper_margin_ {50}; // Upper margin for the detected sea horizon
    int lower_margin_ {50}; // Lower margin for the detected sea horizon

    // MSER blob detector & position voting
    cv::Ptr<cv::MSER> mser_;
    int num_x_bins_ {20};   // Number of bins for horizontal position voting

    // SIFT + FLANN matcher for target recognition
    cv::Ptr<cv::SIFT> sift_;
    cv::Ptr<cv::FlannBasedMatcher> flann_;

    // CSRT tracker for tracking the target
    cv::Ptr<cv::Tracker> tracker_;

    // --------------------------------
    //      Search State Publisher 
    // --------------------------------
    void init_blackboard();
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr state_pub_;
    StateBlackBoard blackboard_;
    std::mutex blackboard_mutex_;

    // TEMP=====
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // cv::Mat latest_frame_;
};

}  // namespace bt_action
#pragma once

#include <unordered_map>
#include <thread>
#include <memory>
#include <vector>
#include <string>
#include <fstream>

// Navigate headers
#include "ardupilot_msgs/msg/global_position.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "bt_action/bt_node.hpp"
#include "bt_action/common_helpers.hpp"
#include "bt_action/navigate_helpers.hpp"

// Search headers
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/tracking.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "bt_action/search_helpers.hpp"
#include "system_component/target_registry.hpp"
#include "gimbal/usv/gimbal_streamer.hpp"
#include "gimbal/usv/gimbal_controller.hpp"
#include "gimbal/gimbal_status.hpp"
#include "gimbal/helpers.hpp"

namespace bt_action
{

class NavigateWhileSearch : public BtNode
{
public:
    NavigateWhileSearch(
        const std::string & name,
        const BT::NodeConfiguration & config
    );
    ~NavigateWhileSearch();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("waypoints_file"),
            BT::InputPort<std::string>("nav_list"),
            BT::InputPort<std::string>("target_id")
        };
    }

    void initialize(
        std::shared_ptr<system_component::TargetRegistry> target_registry,
        std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer,
        std::shared_ptr<gimbal::usv::GimbalController> gimbal_controller
    );

    // Search state transition
    void transfer_to(
        SearchState to_state,
        const std::string & transition_name
    );

protected:
    void run_action_() override;

private:
    // ===============================================
    //                    Common
    // ===============================================
    void navigate_loop();
    void search_process_loop();  // Loop for processing the input frame
    void search_publish_loop();  // Loop for publishing the current searching state
    void stop();                // Stop all the threads and clean up resources

    std::atomic<bool> target_found_ {false};
    std::atomic<bool> reach_goal_ {false};
    std::thread navigate_thread_;
    std::thread search_process_thread_;
    std::thread search_publish_thread_;

    // ===============================================
    //                    Navigate
    // ===============================================
    void geopose_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
    void publish_zero_velocity();

    rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr gps_pub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    double latitude_{0.0};
    double longitude_{0.0};
    double altitude_{0.0};
    std::vector<ardupilot_msgs::msg::GlobalPosition> all_waypoints_;
    std::vector<ardupilot_msgs::msg::GlobalPosition> nav_waypoints_;
    size_t current_nav_idx_ {0};
    double torelance_meters_ {5.0};


    // ===============================================
    //                      Search
    // ===============================================
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
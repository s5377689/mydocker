#include "bt_action/search.hpp"
#include "gimbal_action/all.hpp"

#include <cv_bridge/cv_bridge.h>


namespace bt_action
{

Search::Search(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BtNode(name, config),
    mser_(cv::MSER::create()),
    sift_(cv::SIFT::create()),
    flann_(cv::makePtr<cv::FlannBasedMatcher>(
        cv::makePtr<cv::flann::KDTreeIndexParams>(5),
        cv::makePtr<cv::flann::SearchParams>(50)
    ))
{
    define_transition_funcs();
    init_blackboard();

    state_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/compressed/image_search",
        rclcpp::QoS(10)
    );

    // TEMP =======
    // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     "/camera/image_raw",
    //     rclcpp::QoS(10),
    //     [this](const sensor_msgs::msg::Image::SharedPtr msg) {
    //         latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    //     }
    // );
}

Search::~Search()
{
    // In case the destructor is called while the action is running
    halt_requested_ = true;
    if (process_thread_.joinable())
        process_thread_.join();
    if (publish_thread_.joinable())
        publish_thread_.join();
}

void Search::init_blackboard()
{
    std::lock_guard<std::mutex> lock(blackboard_mutex_);
    blackboard_.reset_vis_flags();
    blackboard_.upper_margin = upper_margin_;
    blackboard_.lower_margin = lower_margin_;
    blackboard_.num_x_bins = num_x_bins_;
    blackboard_.blob_hist.resize(blackboard_.num_x_bins, 0);
}

void Search::transfer_to(
    SearchState to_state,
    const std::string & transition_name)
{
    auto key = std::make_tuple(search_state_, to_state, transition_name);
    auto it = transition_funcs_.find(key);
    if (it != transition_funcs_.end()) {
        search_state_ = to_state;   // Update the current state
        gimbal_controller_->stopAndClearAllActions();  // Stop any actions of the previous state
        it->second();               // Call the transition function
    } else {
        RCLCPP_ERROR(get_logger(), "Transition '%s' not defined.", transition_name.c_str());
    }
}

// ---------------------------------------------
//          State Transition Functions
// ---------------------------------------------
void Search::define_transition_funcs()
{
    // -------------------------
    //     Start -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Start, SearchState::Scanning, "start")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };

    // -------------------------
    //   Scanning -> Detecting
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Scanning, SearchState::Detecting, "start_vote")] = [this]() {
        vote_round_ = 1;

        // Set all bins to have zero votes
        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.blob_hist.assign(blackboard_.num_x_bins, 0);
        }

        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                15.0
            )
        );
    };

    // -------------------------
    //   Detecting -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Detecting, SearchState::Scanning, "horizon_not_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.reset_vis_flags();
        }

        // Reset zoom and pitch to default values
        // and start scanning again
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::ResetZoom>(
                *gimbal_controller_
            )
        );
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::ResetPitch>(
                *gimbal_controller_,
                15.0,
                180.0
            )
        );
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };
    transition_funcs_[std::make_tuple(SearchState::Detecting, SearchState::Scanning, "lack_votes")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.reset_vis_flags();
        }
        
        // Reset zoom and pitch to default values
        // and start scanning again
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::ResetZoom>(
                *gimbal_controller_
            )
        );
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::ResetPitch>(
                *gimbal_controller_,
                15.0,
                180.0
            )
        );
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };
    transition_funcs_[std::make_tuple(SearchState::Detecting, SearchState::Scanning, "blob_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        cv::Rect max_bbox;
        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            max_bbox = blackboard_.max_bbox;
            blackboard_.reset_vis_flags();
        }

        // look at max_pos
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::LookAt>(
                *gimbal_controller_,
                max_bbox
            )
        );

        // zoom x2
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::MultiplyZoom>(
                *gimbal_controller_,
                2.0
            )
        );
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };

    // -------------------------
    //  Scanning -> Recognizing
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Scanning, SearchState::Recognizing, "candidate_found")] = [this]() {
        lockon_start_ms_ = std::chrono::steady_clock::now();
        
        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::LockOn>(
                *gimbal_controller_
            )
        );
    };

    // -------------------------
    //  Recognizing -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Recognizing, SearchState::Scanning, "lockon_failed")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.reset_vis_flags();
        }

        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };
    transition_funcs_[std::make_tuple(SearchState::Recognizing, SearchState::Scanning, "target_not_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.reset_vis_flags();
        }

        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };

    // -------------------------
    //  Recognizing -> Tracking
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Recognizing, SearchState::Tracking, "target_found")] = [this]() {
        track_start_ms_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.vis_tracking = true;
            blackboard_.vis_sift = false;
        }

        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::LockOn>(
                *gimbal_controller_
            )
        );
    };

    // -------------------------
    //   Tracking -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Tracking, SearchState::Scanning, "tracking_lost")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.reset_vis_flags();
        }

        gimbal_controller_->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller_,
                -60,
                60,
                25.0
            )
        );
    };
}

// ---------------------------------------------
//          State Processing Functions
// ---------------------------------------------
void Search::process_scanning(
    cv::Mat & img)
{    int img_match_idx = -1;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::DMatch> best_cluster;
    std::vector<cv::Point2f> detected_corners;
    std::vector<char> matches_mask;

    bool found_target = recognize_target(
        // Inputs:
        sift_, flann_, img, target_data_,
        // Outputs:
        img_match_idx, keypoints, best_cluster, detected_corners, matches_mask
    );

    if (found_target)
    {
        cv::Rect lockon_bbox = cv::boundingRect(detected_corners);

        if (ensure_bbox_in_bounds(lockon_bbox, img) &&
            is_valid_bbox_aspect_ratio(lockon_bbox))
        {
            // Found the target (need to be double-checked later on)
            {
                std::lock_guard<std::mutex> lock(blackboard_mutex_);
                blackboard_.vis_sift = found_target;
                blackboard_.img_match_idx = img_match_idx;
                blackboard_.kp1 = keypoints;
                blackboard_.kp2 = target_data_.keypoints[img_match_idx];
                blackboard_.detected_corners = detected_corners;
                blackboard_.matches = best_cluster;
                blackboard_.matches_mask = matches_mask;
                blackboard_.vis_tracking = found_target;
            }

            // Initialize the tracker
            tracker_ = cv::TrackerCSRT::create();
            tracker_->init(img, lockon_bbox);
            gimbal_controller_->update_lockon_bbox(lockon_bbox);

            transfer_to(SearchState::Recognizing, "candidate_found");
            return;
        }
    }

    // Check if the scanning time has exceeded the limit
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - scan_start_ms_);

    if (elapsed.count() > scan_time_ms_) {
        transfer_to(SearchState::Detecting, "start_vote");
    }
    return;
}

void Search::process_detecting(
    cv::Mat & img)
{
    bool found_horizon = false;
    int crop_top_y = 0;
    int crop_bottom_y = 0;
    {
        std::lock_guard<std::mutex> lock(blackboard_mutex_);
        found_horizon = find_sea_horizon(img, blackboard_);
        crop_top_y = blackboard_.crop_top_y;
        crop_bottom_y = blackboard_.crop_bottom_y;
        blackboard_.vis_horizon = found_horizon;
        blackboard_.vis_blobs = found_horizon;
    }

    // Return to Scanning state if horizon is not found
    if (!found_horizon) {
        transfer_to(SearchState::Scanning, "horizon_not_found");
        return;
    }

    std::vector<int> blob_hist;
    bool found_blob = false;
    int num_x_bins = 0;
    {
        std::lock_guard<std::mutex> lock(blackboard_mutex_);
        found_blob = blob_voting(
            mser_,
            img(cv::Rect(0, crop_top_y, img.cols, crop_bottom_y - crop_top_y)),
            blackboard_
        );
        blob_hist = blackboard_.blob_hist;
        num_x_bins = blackboard_.num_x_bins;
        blackboard_.vis_hist = found_blob;
    }

    if (vote_round_ < final_vote_round_)
    {
        ++vote_round_;
        return;
    }

    // Find the most-voted position
    cv::Rect max_bbox;
    bool found_max = find_most_voted_position(
        blob_hist,
        img.cols / num_x_bins,
        crop_top_y,
        crop_bottom_y,
        max_bbox
    );

    // Handle end of voting
    {
        std::lock_guard<std::mutex> lock(blackboard_mutex_);
        if (found_max)
            blackboard_.max_bbox = max_bbox;
        blackboard_.blob_hist.assign(blackboard_.num_x_bins, 0);
        blackboard_.vis_max_vote = found_max;
    }
    vote_round_ = 1;  // Reset the vote round for the next scanning

    if (!found_max)
    {
        transfer_to(SearchState::Scanning, "lack_votes");
        return;
    }
    else
    {
        transfer_to(SearchState::Scanning, "blob_found");
        return;
    }
}

void Search::process_recognizing(
    cv::Mat & img)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lockon_start_ms_);

    // Re-check the target after lockon time
    if (elapsed.count() > lockon_time_ms_) {
        int img_match_idx = -1;
        std::vector<cv::KeyPoint> keypoints;
        std::vector<cv::DMatch> best_cluster;
        std::vector<cv::Point2f> detected_corners;
        std::vector<char> matches_mask;

        bool found_target = recognize_target(
            sift_, flann_, img, target_data_,
            img_match_idx, keypoints, best_cluster, detected_corners, matches_mask
        );

        if (found_target)
        {
            cv::Rect lockon_bbox = cv::boundingRect(detected_corners);

            if (ensure_bbox_in_bounds(lockon_bbox, img) &&
                is_valid_bbox_aspect_ratio(lockon_bbox))
            {
                // Found the target (the 2nd time)
                // Reinitialize the tracker (the 2nd time)
                tracker_ = cv::TrackerCSRT::create();
                tracker_->init(img, lockon_bbox);
                gimbal_controller_->update_lockon_bbox(lockon_bbox);

                transfer_to(SearchState::Tracking, "target_found"); 
                return;
            }
        }

        transfer_to(SearchState::Scanning, "target_not_found");
        return;
    }
    else
    {
        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard_.vis_sift = false;
        }
    }

    // Continue locking on the candidate
    cv::Rect lockon_bbox;
    bool tracking_success = tracker_->update(img, lockon_bbox);
    if (!tracking_success) {
        RCLCPP_WARN(get_logger(), "Tracking lost, returning to scanning state");
        transfer_to(SearchState::Scanning, "lockon_failed");
        return;
    }
    gimbal_controller_->update_lockon_bbox(lockon_bbox);
}

void Search::process_tracking(
    cv::Mat & img)
{
    cv::Rect lockon_bbox;
    bool tracking_success = tracker_->update(img, lockon_bbox);
    if (!tracking_success) {
        transfer_to(SearchState::Scanning, "tracking_lost");
        return;
    }
    gimbal_controller_->update_lockon_bbox(lockon_bbox);
}

// ---------------------------------------------
//       Visualization and Publish Thread
// ---------------------------------------------
void Search::publishLoop()
{
    // To fetch the target image to show
    std::string target_id;
    system_component::TargetData target_data; 

    try {
        // Check target to search
        if (!getInput("target_id", target_id)) {
            RCLCPP_ERROR(get_logger(), "Missing required input 'target_id'");
            stop();
            return;
        }
        if (!target_registry_) {
            RCLCPP_ERROR(get_logger(), "Target registry is not initialized");
            stop();
            return;
        }
        if (!target_registry_->get_target(target_id, target_data)) {
            RCLCPP_ERROR(get_logger(), "Target '%s' not found in registry", target_id.c_str());
            stop();
            return;
        }

        // Initialize gimbal streamer
        if (!gimbal_streamer_) {
            RCLCPP_ERROR(get_logger(), "Gimbal streamer is not initialized");
            stop();
            return;
        }
        gimbal_streamer_->enablePublish();
        gimbal_streamer_->start();
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception during initialization: %s", e.what());
        status_ = BT::NodeStatus::FAILURE;
        stop();
        return;
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown error during initialization");
        status_ = BT::NodeStatus::FAILURE;
        stop();
        return;
    }

    rclcpp::Rate rate(30);  // Publish at 30 FPS
    cv::Mat img;

    while (rclcpp::ok() && !halt_requested_)
    {
        if (halt_requested_ || !rclcpp::ok()) {
            stop();
            return;
        }

        // Get the latest frame
        bool has_frame = gimbal_streamer_->getLatestFrame(img);

        // Copy the current state from the blackboard
        StateBlackBoard blackboard;
        {
            std::lock_guard<std::mutex> lock(blackboard_mutex_);
            blackboard = blackboard_;
        }

        if (!has_frame)
            rate.sleep();  // If no frame is available, skip this iteration

        // Visualize the current state
        if (blackboard.vis_state)
            draw_state_label(img, search_state_);
        if (blackboard.vis_horizon)
            draw_sea_horizon(img, blackboard);
        if (blackboard.vis_blobs)
            draw_blobs(img, blackboard, cv::Scalar(0, 255, 0), 2);
        if (blackboard.vis_hist)
            draw_votes_histogram(img, blackboard.blob_hist, winner_threshold_);
        if (blackboard.vis_max_vote)
            draw_bbox(img, blackboard.max_bbox, "", cv::Scalar(0, 0, 255), 3);
        if (blackboard.vis_sift) {
            cv::Mat target_img = cv::imread(
                target_data.filepaths[blackboard.img_match_idx],
                cv::IMREAD_COLOR
            );
            cv::Mat out_img;
            draw_sift_matches(
                img,
                target_img,
                out_img,
                blackboard.kp1,
                blackboard.kp2,
                blackboard.matches,
                blackboard.matches_mask,
                blackboard.detected_corners
            );
            // cv::imshow("SIFT Matches", out_img);
            // int key = cv::waitKey(0);
            // if (key == 'q' || key == 27) {
            //     cv::destroyAllWindows();
            // }
            img = out_img;
        }
        if (blackboard.vis_tracking) {
            auto lockon_bbox = gimbal_controller_->get_lockon_bbox();
            draw_bbox(img, lockon_bbox, "", cv::Scalar(0, 0, 255), 2);
        }

        // Publish the current state image
        resize_compress_then_publish(
            state_pub_,
            img,
            0.5f
        );
        rate.sleep();
    }
}

// ---------------------------------------------
//            Thread Initialization
// ---------------------------------------------
void Search::initialize(
    std::shared_ptr<system_component::TargetRegistry> target_registry,
    std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer,
    std::shared_ptr<gimbal::usv::GimbalController> gimbal_controller)
{
    target_registry_ = target_registry;
    gimbal_streamer_ = gimbal_streamer;
    gimbal_controller_ = gimbal_controller;
}

void Search::run_action_()
{
    process_thread_ = std::thread(&Search::processLoop, this);
    publish_thread_ = std::thread(&Search::publishLoop, this);

    if (process_thread_.joinable()) {
        process_thread_.join();
        RCLCPP_INFO(get_logger(), "Process thread exited.");
    }
    if (publish_thread_.joinable()) {
        publish_thread_.join();
        RCLCPP_INFO(get_logger(), "Publish thread exited.");
    }

    RCLCPP_INFO(get_logger(), "All threads have exited.");
}

void Search::stop()
{
    halt_requested_ = true;  // To notify each thread to stop processing and exit
    status_ = BT::NodeStatus::FAILURE;
    gimbal_controller_->stopAndClearAllActions();
}

// ---------------------------------------------
//                  Main Loop
// ---------------------------------------------
void Search::processLoop()
{
    // Resources needed for the search action
    std::string target_id;

    // ---------------------------------------------
    //               Check resources
    // ---------------------------------------------
    try {
        // Check target to search
        if (!getInput("target_id", target_id)) {
            RCLCPP_ERROR(get_logger(), "Missing required input 'target_id'");
            stop();
            return;
        }
        if (!target_registry_) {
            RCLCPP_ERROR(get_logger(), "Target registry is not initialized");
            stop();
            return;
        }
        if (!target_registry_->get_target(target_id, target_data_)) {
            RCLCPP_ERROR(get_logger(), "Target '%s' not found in registry", target_id.c_str());
            stop();
            return;
        }

        // Check gimbal streamer
        if (!gimbal_streamer_) {
            RCLCPP_ERROR(get_logger(), "Gimbal streamer is not initialized");
            stop();
            return;
        }

        // Check gimbal controller
        if (!gimbal_controller_) {
            RCLCPP_ERROR(get_logger(), "Gimbal controller is not initialized");
            stop();
            return;
        }
        gimbal_controller_->start();
        if (!gimbal_controller_->isConnected()) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to gimbal controller");
            stop();
            return;
        }
        gimbal_streamer_->start();
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception during initialization: %s", e.what());
        status_ = BT::NodeStatus::FAILURE;
        stop();
        return;
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown error during initialization");
        status_ = BT::NodeStatus::FAILURE;
        stop();
        return;
    }

    try {
        // "start": Start -> Scanning
        transfer_to(SearchState::Scanning, "start");

        while (true)
        {
            if (halt_requested_ || !rclcpp::ok()) {
                stop();
                return;
            }

            // Get the current image frame from the gimbal streamer
            cv::Mat img;

            // TEMP=========================
            bool has_frame = gimbal_streamer_->getLatestFrame(img);
            // bool has_frame = true;
            // latest_frame_.copyTo(img);

            std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
            while (!has_frame)
            {
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = now - start_time;
                if (elapsed > img_timeout_) {
                    RCLCPP_ERROR(get_logger(), "Timeout waiting for camera frame.");
                    stop();
                    return;
                }
                RCLCPP_WARN(get_logger(), "Waiting for camera frame...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // TEMP=========================
                has_frame = gimbal_streamer_->getLatestFrame(img);
                // has_frame = true;
                // latest_frame_.copyTo(img);
            }

            if (img.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            switch (search_state_)
            {
                case SearchState::Scanning:
                    process_scanning(img);
                    break;

                case SearchState::Detecting:
                    process_detecting(img);
                    break;

                case SearchState::Recognizing:
                    process_recognizing(img);
                    break;

                case SearchState::Tracking:
                    process_tracking(img);
                    break;

                default:
                    RCLCPP_ERROR(get_logger(), "Unknown search state: %d", static_cast<int>(search_state_));
                    stop();
                    return;
            }
        }
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception during processing: %s", e.what());
        status_ = BT::NodeStatus::FAILURE;
        stop();
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown error during processing");
        status_ = BT::NodeStatus::FAILURE;
        stop();
    }
}

}  // namespace bt_action
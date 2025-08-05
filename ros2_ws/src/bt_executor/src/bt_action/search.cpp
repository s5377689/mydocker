#include "bt_action/search.hpp"
#include "gimbal_action/all.hpp"


namespace bt_action
{

Search::Search(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BtNode(name, config)
{
    define_transition_funcs();

    sift_ = cv::SIFT::create();
    flann_ = cv::makePtr<cv::FlannBasedMatcher>(
        cv::makePtr<cv::flann::KDTreeIndexParams>(5),
        cv::makePtr<cv::flann::SearchParams>(50)
    );
    search_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/compressed/image_search",
        rclcpp::QoS(10)
    );

    blob_hist_.resize(num_x_bins_, 0);
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

void Search::transit_to(
    SearchState to_state,
    const std::string & transition_name)
{
    auto key = std::make_tuple(search_state_, to_state, transition_name);
    auto it = transition_funcs_.find(key);
    if (it != transition_funcs_.end()) {
        search_state_ = to_state;   // Update the current state
        gimbal_controller_.lock()->stopAndClearAllActions();  // Stop any actions of the previous state
        it->second();               // Call the transition function
    } else {
        RCLCPP_ERROR(get_logger(), "Transition '%s' not defined.", transition_name.c_str());
    }
}

void Search::define_transition_funcs()
{
    // -------------------------
    //     Start -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Start, SearchState::Scanning, "start")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();
        auto gimbal_controller = gimbal_controller_.lock();

        gimbal_controller->enqueueAction(
            std::make_shared<gimbal_action::Scan>(
                *gimbal_controller,
                -60,
                60,
                30.0  // Speed of pan movement in degrees per second
            )
        );
    };

    // -------------------------
    //   Scanning -> Detecting
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Scanning, SearchState::Detecting, "start_vote")] = [this]() {
        vote_round_ = 1;
        auto gimbal_controller = gimbal_controller_.lock();

        gimbal_controller->enqueueAction(
            std::make_shared<gimbal_action::ResetPitch>(
                *gimbal_controller
            )
        );
    };

    // -------------------------
    //   Detecting -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Detecting, SearchState::Scanning, "horizon_not_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();
        // reset pitch
        // reset zoom
    };
    transition_funcs_[std::make_tuple(SearchState::Detecting, SearchState::Scanning, "blob_not_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();
        // reset zoom
    };
    transition_funcs_[std::make_tuple(SearchState::Detecting, SearchState::Scanning, "blob_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();
        // zoom x2
        // look at max_pos
    };

    // -------------------------
    //  Scanning -> Recognizing
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Scanning, SearchState::Recognizing, "candidate_found")] = [this]() {
        lockon_start_ms_ = std::chrono::steady_clock::now();
    };

    // -------------------------
    //  Recognizing -> Scanning
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Recognizing, SearchState::Scanning, "lockon_lost")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();
    };
    transition_funcs_[std::make_tuple(SearchState::Recognizing, SearchState::Scanning, "target_not_found")] = [this]() {
        scan_start_ms_ = std::chrono::steady_clock::now();
    };

    // -------------------------
    //  Recognizing -> Tracking
    // -------------------------
    transition_funcs_[std::make_tuple(SearchState::Recognizing, SearchState::Tracking, "target_found")] = [this]() {
        track_start_ms_ = std::chrono::steady_clock::now();
    };
}

void Search::initialize(
    std::shared_ptr<system_component::TargetRegistry> target_registry,
    std::shared_ptr<gimbal::GimbalStreamer> gimbal_streamer,
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
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
    // if (auto gs = gimbal_streamer_.lock())
    // {
    //     gs->stop();
    // }
    if (auto gc = gimbal_controller_.lock())
    {
        // gc->stop();
        gc->stopAndClearAllActions();
    }
}

// ---------------------------------------------
//                 Process Loop
// ---------------------------------------------
void Search::processLoop()
{
    // Resources needed for the search action
    std::shared_ptr<system_component::TargetRegistry> target_registry;
    std::shared_ptr<gimbal::GimbalStreamer> gimbal_streamer;
    std::shared_ptr<gimbal::GimbalController> gimbal_controller;
    std::string target_id;
    system_component::TargetData target_data; 

    // ---------------------------------------------
    //            Initialize resources
    // ---------------------------------------------
    try {
        // Initialize target to search
        if (!getInput("target_id", target_id)) {
            RCLCPP_ERROR(get_logger(), "Missing required input 'target_id'");
            stop();
            return;
        }
        target_registry = target_registry_.lock();
        if (!target_registry) {
            RCLCPP_ERROR(get_logger(), "Target registry is not initialized");
            stop();
            return;
        }
        if (!target_registry->get_target(target_id, target_data)) {
            RCLCPP_ERROR(get_logger(), "Target '%s' not found in registry", target_id.c_str());
            stop();
            return;
        }

        // Initialize gimbal streamer
        gimbal_streamer = gimbal_streamer_.lock();
        if (!gimbal_streamer) {
            RCLCPP_ERROR(get_logger(), "Gimbal streamer is not initialized");
            stop();
            return;
        }

        // Initialize gimbal controller
        gimbal_controller = gimbal_controller_.lock();
        if (!gimbal_controller) {
            RCLCPP_ERROR(get_logger(), "Gimbal controller is not initialized");
            stop();
            return;
        }
        gimbal_controller->start();
        if (!gimbal_controller->isConnected()) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to gimbal controller");
            stop();
            return;
        }

        gimbal_streamer->start();
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

    // ---------------------------------------------
    //                  Main loop
    // ---------------------------------------------
    try {
        transit_to(SearchState::Scanning, "start");

        while (true)
        {
            if (halt_requested_ || !rclcpp::ok()) {
                stop();
                return;
            }

            // Get the current image frame from the gimbal streamer
            cv::Mat img;
            bool has_frame = gimbal_streamer->getLatestFrame(img);
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
                has_frame = gimbal_streamer->getLatestFrame(img);
            }

            switch (search_state_)
            {
                case SearchState::Scanning:
                    process_scanning(img, gimbal_controller);
                    break;

                case SearchState::Detecting:
                    process_detecting(img, gimbal_controller);
                    break;

                case SearchState::Recognizing:
                    process_recognizing(img, gimbal_controller);
                    break;

                case SearchState::Tracking:
                    process_tracking(img, gimbal_controller);
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

// ---------------------------------------------
//                 Publish Loop
// ---------------------------------------------
void Search::publishLoop()
{
    std::shared_ptr<gimbal::GimbalStreamer> gimbal_streamer;
    try {
        // Initialize gimbal streamer
        gimbal_streamer = gimbal_streamer_.lock();
        if (!gimbal_streamer) {
            RCLCPP_ERROR(get_logger(), "Gimbal streamer is not initialized");
            stop();
            return;
        }
        gimbal_streamer->enablePublish();
        gimbal_streamer->start();
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

        // Get the current image frame from the gimbal streamer
        bool has_frame = gimbal_streamer->getLatestFrame(img);

        if (has_frame) 
        {
            drawStatusLabel(img, search_state_);
            resize_compress_then_publish(
                search_pub_,
                img,
                0.5f
            );
        }

        rate.sleep();
    }
}

// ---------------------------------------------
//          State Processing Functions
// ---------------------------------------------
void Search::process_scanning(
    cv::Mat & img,
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
{
    // Check if the scanning time has exceeded the limit
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - scan_start_ms_);
    if (elapsed.count() > scan_time_ms_) {
        transit_to(SearchState::Detecting, "start_vote");
    }
}

void Search::process_detecting(
    cv::Mat & img,
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
{
    RCLCPP_INFO(get_logger(), "Processing Detecting state");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void Search::process_recognizing(
    cv::Mat & img,
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
{
    RCLCPP_INFO(get_logger(), "Processing Recognizing state");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void Search::process_tracking(
    cv::Mat & img,
    std::shared_ptr<gimbal::GimbalController> gimbal_controller)
{
    RCLCPP_INFO(get_logger(), "Processing Tracking state");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

}  // namespace bt_action
#include "bt_action/take_photo.hpp"

namespace bt_action
{

TakePhoto::TakePhoto(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BtNode(name, config)
{}

void TakePhoto::initialize(
    std::shared_ptr<gimbal::usv::GimbalStreamer> gimbal_streamer)
{
    gimbal_streamer_ = gimbal_streamer;
}

void TakePhoto::run_action_()
{
    std::string folder;
    if (!getInput("image_folder", folder))
    {
        RCLCPP_WARN(get_logger(), "Missing input port <image_folder>");
        folder = "/home/robuff/data/photo";  
        RCLCPP_WARN(get_logger(), "Using default folder: %s", folder.c_str());
        // status_ = BT::NodeStatus::FAILURE;
        // return;
    }

    if (!gimbal_streamer_)
    {
        RCLCPP_ERROR(get_logger(), "Gimbal streamer is not initialized");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    gimbal_streamer_->start();

    cv::Mat latest_frame;
    bool has_image = false;

    // Setup timeout
    const int timeout_ms = 3000; // 3 seconds timeout
    auto start_time = std::chrono::steady_clock::now();

    // Wait for the image to be available
    while (!has_image && !halt_requested_)
    {
        has_image = gimbal_streamer_->getLatestFrame(latest_frame);

        if (!has_image)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
            if (elapsed > timeout_ms)
            {
                RCLCPP_ERROR(get_logger(), "Timeout waiting for image");
                status_ = BT::NodeStatus::FAILURE;
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (!has_image)
    {
        RCLCPP_ERROR(get_logger(), "Failed to get image from gimbal streamer");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }

    // Generate a unique filename based on the current time
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);
    char filename[100];
    std::strftime(filename, sizeof(filename), "image_%Y%m%d_%H%M%S.jpg", &now_tm);
    std::string image_path = folder + "/" + filename;

    // Save the image
    if (!cv::imwrite(image_path, latest_frame))
    {
        RCLCPP_ERROR(get_logger(), "Failed to save image to %s", image_path.c_str());
        status_ = BT::NodeStatus::FAILURE;
        return;
    }

    RCLCPP_INFO(get_logger(), "Image saved to %s", image_path.c_str());
    status_ = BT::NodeStatus::SUCCESS;
}

}  // end of namespace bt_action
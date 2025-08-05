#include <opencv2/imgcodecs.hpp>
#include "bt_action/register_target.hpp"
namespace fs = std::filesystem;

namespace bt_action
{

RegisterTarget::RegisterTarget(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BtNode(name, config)
{}

void RegisterTarget::initialize(
    std::shared_ptr<system_component::TargetRegistry> registry)
{
    registry_ = std::move(registry);
}

void RegisterTarget::run_action_()
{
    try {
        std::string target_id, folder;
        if (!getInput("target_id", target_id))
        {
            RCLCPP_ERROR(get_logger(), "Missing input port <target_id>");
            status_ = BT::NodeStatus::FAILURE;
            return;
        } 
        if (!getInput("image_folder", folder))
        {
            RCLCPP_ERROR(get_logger(), "Missing input port <image_folder>");
            status_ = BT::NodeStatus::FAILURE;
            return;
        }

        RCLCPP_INFO(get_logger(), "Registering target: <%s>", target_id.c_str());

        std::vector<std::string> all_filepaths;
        std::vector<std::vector<cv::KeyPoint>> all_keypoints;
        std::vector<cv::Mat> all_descriptors;

        auto sift = cv::SIFT::create();

        for (const auto & entry : fs::directory_iterator(folder))
        {
            if (halt_requested_)
                return;
        
            if (!entry.is_regular_file())
                continue;
        
            // Check if the file is an image
            const auto & file_path = entry.path();
            if (file_path.extension() != ".png" &&
                file_path.extension() != ".jpg" &&
                file_path.extension() != ".jpeg")
                continue;

            cv::Mat img = cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);

            if (img.empty())
            {
                RCLCPP_ERROR(get_logger(), "Failed to read image: %s", file_path.string().c_str());
                continue;
            }

            all_filepaths.push_back(file_path.string());

            // Parse the image 
            std::vector<cv::KeyPoint> kp;
            cv::Mat desc;
            sift->detectAndCompute(img, cv::noArray(), kp, desc);

            if (kp.empty() || desc.empty())
            {
                RCLCPP_WARN(get_logger(), "Failed to detect keypoints or compute descriptors for image: %s", file_path.string().c_str());
                continue;
            }

            all_keypoints.push_back(kp);
            all_descriptors.push_back(desc);

            // DEBUG: Draw keypoints
            // cv::Mat img_with_kp;
            // cv::drawKeypoints(img, kp, img_with_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
            // cv::imshow("SIFT Keypoints - " + file_path.filename().string(), img_with_kp);
            // int key = cv::waitKey(0);
            // cv::destroyAllWindows();
            // if (key == 27) // ESC key
            // {
            //     RCLCPP_INFO(get_logger(), "User requested to stop processing images.");
            //     halt_requested_ = true;
            //     return;
            // }
        }

        if (all_filepaths.empty())
        {
            RCLCPP_ERROR(get_logger(), "No valid images found in folder: %s", folder.c_str());
            status_ = BT::NodeStatus::FAILURE;
            return;
        }

        // Store the target data in the registry
        registry_->register_target(
            target_id,
            system_component::TargetData{
                all_filepaths,
                all_keypoints,
                all_descriptors
            }
        );
        RCLCPP_INFO(get_logger(), "Target <%s> is registered with %zu images", target_id.c_str(), all_keypoints.size());
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception while registering target: %s", e.what());
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Unknown error occurred while registering target.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }

    status_ = BT::NodeStatus::SUCCESS;
}

} // namespace bt_action
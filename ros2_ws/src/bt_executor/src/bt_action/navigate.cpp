#include "bt_action/navigate.hpp"
#include <memory>


using namespace std::chrono_literals;
using ardupilot_msgs::msg::GlobalPosition;
using geographic_msgs::msg::GeoPoseStamped;

namespace bt_action
{

Navigate::Navigate(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BtNode(name, config),
    latitude_(0.0),
    longitude_(0.0),
    altitude_(0.0)
{
    gps_pub_ = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(
        "/ap/cmd_gps_pose",
        10
    );

    rclcpp::QoS qos(10);
    qos.best_effort();
    qos.durability_volatile();

    gps_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        "/ap/geopose/filtered",
        qos,
        std::bind(&Navigate::geopose_callback, this, std::placeholders::_1)
    );
}

void Navigate::run_action_()
{
    try {
        // Load waypoints from the specified file
        std::string filename;
        if (!getInput("waypoints_file", filename))
        {
            RCLCPP_ERROR(get_logger(), "Missing required input <waypoints_file>");
            halt_requested_ = true;
            status_ = BT::NodeStatus::FAILURE;
            return;
        }
        loadWaypoints(filename, all_waypoints_);

        std::string nav_list;

        if (getInput("nav_list", nav_list))
        {
            loadSubsetWaypoints(
                all_waypoints_,
                nav_list,
                nav_waypoints_
            );
        }
        else
        {
            // If no list is provided, assume all waypoints are to be used
            nav_waypoints_ = all_waypoints_;
            RCLCPP_INFO(get_logger(), "No nav_list provided, using all waypoints.");
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Error loading waypoints: %s", e.what());
        halt_requested_ = true;
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Unknown error occurred while loading waypoints.");
        halt_requested_ = true;
        status_ = BT::NodeStatus::FAILURE;
        return;
    }

    try {
        rclcpp::WallRate rate(1.0);
        auto & dest = nav_waypoints_[current_nav_idx_];

        // Navigate through each waypoint
        while (!halt_requested_ && rclcpp::ok())
        {
            gps_pub_->publish(dest);
        
            if (reach_waypoint())
            {
                RCLCPP_INFO(get_logger(), "Reach waypoint %zu", current_nav_idx_ + 1);
                ++current_nav_idx_;

                // if all waypoints have been passed and target still not found, stop navigation
                if (current_nav_idx_ >= nav_waypoints_.size())
                {
                    RCLCPP_INFO(get_logger(), "All waypoints have been passed.");
                    status_ = BT::NodeStatus::SUCCESS;
                    break;
                }
                else
                    dest = nav_waypoints_[current_nav_idx_];  // Set destination to the next waypoint
            }

            rate.sleep();
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Exception during navigation: %s", e.what());
        halt_requested_ = true;
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Unknown error occurred during navigation.");
        halt_requested_ = true;
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
}

void Navigate::geopose_callback(
    const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
    latitude_ = msg->pose.position.latitude;  
    longitude_ = msg->pose.position.longitude;
    altitude_ = msg->pose.position.altitude;
}

bool Navigate::reach_waypoint(double tolerance_meters)
{
    if (current_nav_idx_ >= nav_waypoints_.size())
        return true;

    const auto & wp = nav_waypoints_[current_nav_idx_];

    // Compute horizontal distance (in meters)
    double horizon_dist = haversine(latitude_, longitude_, wp.latitude, wp.longitude);

    return horizon_dist < tolerance_meters;
}

}  // namespace bt_action
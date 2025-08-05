#pragma once

#include <vector>
#include <string>
#include <fstream>

#include "ardupilot_msgs/msg/global_position.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"

#include "bt_action/bt_node.hpp"
#include "bt_action/helpers.hpp"

namespace bt_action
{

class Navigate : public BtNode
{
public:
    Navigate(
        const std::string & name,
        const BT::NodeConfiguration & config
    );
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("waypoints_file"),
            BT::InputPort<std::string>("nav_list")
        };
    }

protected:    
    virtual void run_action_() override;

private:
    void geopose_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
    bool reach_waypoint(double tolerance_meters = 6.2);

    rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr gps_pub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr gps_sub_;

    double latitude_, longitude_, altitude_;
    std::vector<ardupilot_msgs::msg::GlobalPosition> all_waypoints_;
    std::vector<ardupilot_msgs::msg::GlobalPosition> nav_waypoints_;
    size_t current_nav_idx_ = 0;
};

}  // namespace bt_action
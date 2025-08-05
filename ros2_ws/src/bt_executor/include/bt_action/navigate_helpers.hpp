#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ardupilot_msgs/msg/global_position.hpp"

#include "bt_action/common_helpers.hpp"


namespace bt_action
{

void loadWaypoints(
    const std::string & filename,
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_waypoints
);

const double kEarthRadiusMeters = 6371000.0; // Radius of the Earth in meters
void loadSubsetWaypoints(
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & all_waypoints,
    int from, int to, 
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_waypoints
);

void loadSubsetWaypoints(
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & all_waypoints,
    const std::string & list_waypoints,  // 1-based indices as a comma-separated string
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_waypoints
);

double haversine(double lat1, double lon1, double lat2, double lon2);

// - latitude, longitude: current position
// - wp: target waypoint to check
// - tolerance_meters: distance tolerance to consider the waypoint reached
bool reach_waypoint(
    double latitude,
    double longitude,
    ardupilot_msgs::msg::GlobalPosition & wp,
    double tolerance_meters
);

}  // namespace bt_action
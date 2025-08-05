#include "bt_action/navigate_helpers.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
using namespace std;


namespace bt_action
{

void loadWaypoints(
    const std::string & filename,
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_waypoints)
{
    out_waypoints.clear();

    // Load and parse the .plan file
    std::ifstream ifs(filename);
    nlohmann::json plan_json;
    ifs >> plan_json;

    for (const auto & item : plan_json["mission"]["items"])
    {
        if (item["command"] == 16 || item["command"] == 21) // MAV_CMD_NAV_WAYPOINT or ROI
        {
            // Add each waypoint to the list to navigate
            ardupilot_msgs::msg::GlobalPosition wp;
            wp.latitude = item["params"][4];
            wp.longitude = item["params"][5];
            wp.altitude = item["params"][6];
            wp.coordinate_frame = 5;
            wp.header.frame_id = "map";
            out_waypoints.push_back(wp);
        }
    }

    std::cout << "Loaded " << out_waypoints.size()
              <<  " waypoints from file: " << filename << std::endl;
}

void loadSubsetWaypoints(
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & all_waypoints,
    int from, int to, 
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_waypoints)
{
    out_waypoints.clear();

    // Handle special cases (0 or negative indices)
    if (from < 1)
        from = 1;
    if (to < 1 || to > all_waypoints.size())
        to = all_waypoints.size();

    // Convert to 0-based indices
    from -= 1;
    to -= 1;

    // Ensure indices are within bounds
    if (from < 0 || from >= all_waypoints.size() || to < from || to >= all_waypoints.size())
    {
        cerr << "Invalid waypoint indices: from=" << from + 1 
             << ", to=" << to + 1 
             << ", total=" << all_waypoints.size() << endl;
        return;
    }

    // Copy the specified range of waypoints
    for (int i = from; i <= to; ++i)
    {
        out_waypoints.push_back(all_waypoints[i]);
    }
    cout << "Loaded " << out_waypoints.size() 
         << " waypoints from index " << from + 1 
         << " to " << to + 1 << endl;
}

void loadSubsetWaypoints(
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & all_waypoints,
    const std::string & list_waypoints,  // 1-based indices as a comma-separated string
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_waypoints)
{
    out_waypoints.clear();

    // Split the input string by commas and trim whitespace
    std::istringstream stream(list_waypoints);
    std::string token;
    std::vector<int> indices;
    int n_waypoints = static_cast<int>(all_waypoints.size());

    while (std::getline(stream, token, ','))
    {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \n\r\t"));
        token.erase(token.find_last_not_of(" \n\r\t") + 1);
        if (token.empty())
            continue;

        // Convert to integer and add to indices
        try {
            int idx = std::stoi(token);
            if (idx == -1)
                idx = n_waypoints;
            if (idx >= 1 && idx <= n_waypoints)
                indices.push_back(idx - 1);  // Convert to 0-based index
            else
                cerr << "Waypoint index out of bounds: " << token << endl;
        }
        catch (const std::exception & e)
        {
            cerr << "Invalid waypoint index '" << token << "': " << e.what() << endl;
            return;
        }
    }

    // Copy the specified waypoints
    std::cout << "Selected waypoint indices: "; 
    for (int idx : indices)
    {
        out_waypoints.push_back(all_waypoints[idx]);
        std::cout << idx + 1 << " ";  // Convert to 1-based index for display
    }
    std::cout << std::endl;
    
    cout << "Loaded " << out_waypoints.size() 
         << " waypoints from the list: " << list_waypoints << endl;
}

double haversine(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);

    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) *
               std::sin(dlon / 2) * std::sin(dlon / 2);

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return kEarthRadiusMeters * c;
}

bool reach_waypoint(
    double latitude,
    double longitude,
    ardupilot_msgs::msg::GlobalPosition & wp,
    double tolerance_meters)
{
    // Compute horizontal distance (in meters)
    double distance = haversine(latitude, longitude, wp.latitude, wp.longitude);

    return distance < tolerance_meters;
}

}  // namespace bt_action
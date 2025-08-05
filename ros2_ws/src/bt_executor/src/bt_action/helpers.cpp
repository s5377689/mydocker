#include "bt_action/helpers.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
using namespace std;
using namespace cv;

namespace bt_action
{

// =============================================
//               Navigation Helpers
// =============================================
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

bool reachedTargetWaypoint(
    double latitude,
    double longitude,
    ardupilot_msgs::msg::GlobalPosition & wp,
    double tolerance_meters)
{
    // Compute horizontal distance (in meters)
    double horiz_dist = haversine(latitude, longitude, wp.latitude, wp.longitude);

    // Vertical difference (altitude is already in meters)
    // double vert_diff = std::abs(altitude - wp.altitude);

    // Total 3D distance
    // double total_dist = std::sqrt(horiz_dist * horiz_dist + vert_diff * vert_diff);
    double total_dist = horiz_dist; // For USVs, only horizontal distance is considered

    // RCLCPP_INFO(get_logger(),
    //     "Distance to waypoint %zu: horiz = %.2f m, vert = %.2f m, total = %.2f m",
    //     current_waypoint_idx_, horiz_dist, vert_diff, total_dist);

    return total_dist < tolerance_meters;
}

// =============================================
//                 Search Helpers
// =============================================
bool is_inside_rect(
    const ardupilot_msgs::msg::GlobalPosition & point,
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & rect)
{
    double min_lat = std::min(rect[0].latitude, rect[2].latitude);
    double max_lat = std::max(rect[0].latitude, rect[2].latitude);
    double min_lon = std::min(rect[0].longitude, rect[2].longitude);
    double max_lon = std::max(rect[0].longitude, rect[2].longitude);

    return (point.latitude >= min_lat && point.latitude <= max_lat &&
            point.longitude >= min_lon && point.longitude <= max_lon);
}

bool isWithinBounds(float pan, float from, float to, bool rotating_right)
{
    if (rotating_right) {
        if (from < to)
            return pan < to;
        else
            return pan < to || pan > from;
    } else {
        if (from < to)
            return pan > from;
        else
            return pan > from || pan < to;
    }
}

// Normalize angle to the range [-180, 180)
double normalize_angle(
    double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}

// Bearing from point A to B
double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double φ1 = deg2rad(lat1);
    double φ2 = deg2rad(lat2);
    double Δλ = deg2rad(lon2 - lon1);

    double y = sin(Δλ) * cos(φ2);
    double x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ);

    return normalize_angle(rad2deg(atan2(y, x)));
}

// Yaw from quaternion
double get_yaw_from_quat(double x, double y, double z, double w) {
    return atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
}

// Compute pan_from_degree and pan_to_degree
std::pair<double, double> compute_scan_sector(
    double usv_lat, double usv_lon,
    double qx, double qy, double qz, double qw,
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & scan_corners)
{
    double yaw_rad_enu = get_yaw_from_quat(qx, qy, qz, qw);
    double yaw_deg_enu = rad2deg(yaw_rad_enu);
    double yaw_deg = normalize_angle(90.0 - yaw_deg_enu); // Convert ENU to NED

    std::vector<double> relative_bearings;
    for (const auto& corner : scan_corners) {
        double abs_bearing = bearing_deg(
            usv_lat, usv_lon, corner.latitude, corner.longitude
        );
        double rel_bearing = normalize_angle(abs_bearing - yaw_deg);
        relative_bearings.push_back(rel_bearing);
    }

    // Find smallest sector containing all relative bearings
    double min_sector = 360.0;
    double pan_from = 0.0, pan_to = 0.0;
    for (double start : relative_bearings) {
        double max_offset = 0.0;
        for (double b : relative_bearings) {
            double offset = normalize_angle(b - start);
            if (offset < 0) offset += 360.0;
            max_offset = std::max(max_offset, offset);
        }
        if (max_offset < min_sector) {
            min_sector = max_offset;
            pan_from = start;
            pan_to = normalize_angle(start + max_offset);
        }
    }

    return {pan_from, pan_to};  // in degrees, relative to USV heading
}

double normalizeLon(double lon)
{
    // Normalize longitude to the range [0, 360)
	while (lon < 0) lon += 360.0;
	while (lon >= 360.0) lon -= 360.0;
	return lon;
}

double toStdLon(double lon)
{
    // Covert longitude to the standard range [-180, 180)
    while (lon < -180.0) lon += 360.0;
    while (lon > 180.0) lon -= 360.0;
    return lon;
}

void calculateMinCoverRect(
    const std::vector<ardupilot_msgs::msg::GlobalPosition>& waypoints,
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_rect,
    double margin)
{
    out_rect.clear();
    double min_lat = std::numeric_limits<double>::max();
    double max_lat = std::numeric_limits<double>::lowest();

    if (waypoints.empty())
    {
        cerr << "[calculateMinCoverRect] No waypoints provided." << endl;
        return; // No waypoints to process
    }

    // Iterate through the waypoints while:
    // 1. Find the min and max latitude
    // 2. Normalize longitudes to [0, 360)
    vector<double> norm_lons;
    for (auto & wp : waypoints)
    {
        double lat = wp.latitude;
        double lon = normalizeLon(wp.longitude);

        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);

        norm_lons.push_back(normalizeLon(lon));
    }

    // Sort longitues and find the largest gap
    std::sort(norm_lons.begin(), norm_lons.end());
    double max_gap = 0.0;
    int gap_idx = -1;

    for (size_t i = 0; i < norm_lons.size(); ++i)
    {
        double next_lon = norm_lons[(i + 1) % norm_lons.size()];
        double current_lon = norm_lons[i];
        double gap = next_lon - current_lon;

        if (gap < 0)
            gap += 360.0; // Handle wrap-around case

        if (gap > max_gap)
        {
            max_gap = gap;
            gap_idx = i;
        }
    }

    // The bounding width is the complement of the largest gap
    double lon_start = norm_lons[(gap_idx + 1) % norm_lons.size()];
    double lon_end = norm_lons[gap_idx];

    // Convert back to standard longitude range [-180, 180)
    double min_lon = toStdLon(lon_start);
    double max_lon = toStdLon(lon_end);

    // Add some margin around the bounding rectangle
    min_lat = max(min_lat - margin, -90.0);    
    max_lat = min(max_lat + margin, 90.0);

    min_lon -= margin;
    max_lon += margin;
    if (min_lon < -180.0)
        min_lon += 360.0;
    if (max_lon > 180.0)
        max_lon -= 360.0;

    auto add_point = [&] (double lat, double lon) {
        ardupilot_msgs::msg::GlobalPosition point;
        point.latitude = lat;
        point.longitude = lon;
        out_rect.push_back(point);
    };

    // Add the four corners of the rectangle
    add_point(min_lat, min_lon);
    add_point(min_lat, max_lon);
    add_point(max_lat, max_lon);
    add_point(max_lat, min_lon);
}

void calculateMinCoverRect(
    const std::vector<ardupilot_msgs::msg::GlobalPosition>& waypoints,
    int wp_from,  // 0-based index
    int wp_to,
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_rect,
    double margin)
{
    out_rect.clear();
    double min_lat = std::numeric_limits<double>::max();
    double max_lat = std::numeric_limits<double>::lowest();

    if (waypoints.empty())
    {
        cerr << "[calculateMinCoverRect] No waypoints provided." << endl;
        return; // No waypoints to process
    }

    if (wp_from < 0)
        wp_from = 0;  // from the first waypoint
    if (wp_to < 0 || wp_to >= waypoints.size())
        wp_to = waypoints.size() - 1; // to the last waypoint

    // Iterate through the specified subset of the waypoints while:
    // 1. Find the min and max latitude
    // 2. Normalize longitudes to [0, 360)
    vector<double> norm_lons;
    for (int i = wp_from; i <= wp_to; ++i)
    {
        double lat = waypoints[i].latitude;
        double lon = normalizeLon(waypoints[i].longitude);

        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);

        norm_lons.push_back(normalizeLon(lon));
    }

    // Sort longitues and find the largest gap
    std::sort(norm_lons.begin(), norm_lons.end());
    double max_gap = 0.0;
    int gap_idx = -1;

    for (size_t i = 0; i < norm_lons.size(); ++i)
    {
        double next_lon = norm_lons[(i + 1) % norm_lons.size()];
        double current_lon = norm_lons[i];
        double gap = next_lon - current_lon;

        if (gap < 0)
            gap += 360.0; // Handle wrap-around case

        if (gap > max_gap)
        {
            max_gap = gap;
            gap_idx = i;
        }
    }

    // The bounding width is the complement of the largest gap
    double lon_start = norm_lons[(gap_idx + 1) % norm_lons.size()];
    double lon_end = norm_lons[gap_idx];

    // Convert back to standard longitude range [-180, 180)
    double min_lon = toStdLon(lon_start);
    double max_lon = toStdLon(lon_end);

    // Add some margin around the bounding rectangle
    min_lat = max(min_lat - margin, -90.0);    
    max_lat = min(max_lat + margin, 90.0);

    min_lon -= margin;
    max_lon += margin;
    if (min_lon < -180.0)
        min_lon += 360.0;
    if (max_lon > 180.0)
        max_lon -= 360.0;

    out_rect.clear();
    auto add_point = [&] (double lat, double lon) {
        ardupilot_msgs::msg::GlobalPosition point;
        point.latitude = lat;
        point.longitude = lon;
        out_rect.push_back(point);
    };

    // Add the four corners of the rectangle
    add_point(min_lat, min_lon);
    add_point(min_lat, max_lon);
    add_point(max_lat, max_lon);
    add_point(max_lat, min_lon);
}

double clockwise_diff(
    double from_deg, 
    double to_deg)
{
    double diff = to_deg - from_deg;
    while (diff < 0)
        diff += 360.0;
    while (diff >=360.0)
        diff -= 360.0;
    return diff;
}

bool is_in_clockwise_sector(
    double from_deg, 
    double to_deg, 
    double test_deg)
{
    double full_sector = clockwise_diff(from_deg, to_deg);
    double test_sector = clockwise_diff(from_deg, test_deg);
    return test_sector <= full_sector;
}

void overlayHistogram(
    cv::Mat & img,
    const std::vector<int> & votes,
    float alpha,
    int highlight_threshold,
    int bin_unit_height,
    int bin_margin)
{
    cv::Mat overlay = img.clone();

    int bins = votes.size();
    int img_width = img.cols;
    int img_height = img.rows;

    int bin_width = (img_width / bins) - 2 * bin_margin;

    auto max_elem = std::max_element(votes.begin(), votes.end());

    if (*max_elem < highlight_threshold)
    {
        // No need to highlight any bin
        for (int i = 0; i < bins; ++i)
        {
            int bin_height = votes[i] * bin_unit_height; 

            cv::Point p1(
                i * (bin_width + 2 * bin_margin) + bin_margin,
                img_height - bin_height
            );
            cv::Point p2(
                p1.x + bin_width,
                img_height
            );
            cv::rectangle(overlay, p1, p2, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
        }
    }
    else {
        // Highlight bin with maximum votes
        int max_idx = max_elem - votes.begin();

        for (int i = 0; i < bins; ++i)
        {
            int bin_height = votes[i] * bin_unit_height; 

            cv::Point p1(
                i * (bin_width + 2 * bin_margin) + bin_margin,
                img_height - bin_height
            );
            cv::Point p2(
                p1.x + bin_width,
                img_height
            );

            if (i == max_idx)
                cv::rectangle(overlay, p1, p2, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
            else
                cv::rectangle(overlay, p1, p2, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
        }
    }

    cv::addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img);
}

void drawStatusLabel(
    cv::Mat & img,
    const SearchState state,
    float glow_period)
{
    static const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    static const double font_scale = 1.4;
    static const int thickness = 3;
    static const int padding = 20;
    // static const double font_scale = 0.5;
    // static const int thickness = 1;
    // static const int padding = 10;

    std::string text;
    cv::Scalar state_color;
    if (state == SearchState::Scanning)
    {
        text = "Scanning";
        state_color = cv::Scalar(0, 255, 0); // Green
    }
    else if (state == SearchState::Detecting)
    {
        text = "Detecting";
        state_color = cv::Scalar(255, 255, 0); // Cyan
    }
    else if (state == SearchState::Recognizing)
    {
        text = "Recognizing";
        state_color = cv::Scalar(0, 255, 255); // Yellow
    }
    else
    {
        text = "Tracking";
        state_color = cv::Scalar(0, 0, 255); // Red
    }

    // Animate glow based on time
    double t = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
    float glow = 0.5f + 0.5f * std::sin((2 * CV_PI / glow_period) * t);
    cv::Scalar glow_color = glow * state_color;

    // Compute text size
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(
        text, font_face, font_scale, thickness, &baseline
    );
    int box_width = text_size.width + 3 * padding;
    int box_height = text_size.height + 2 * padding;

    // Top-left semi-transparent rounded background
    cv::Point boxPos(10, 10);
    cv::Rect bgRect(boxPos, cv::Size(box_width, box_height));

    cv::Mat overlay;
    img.copyTo(overlay);
    cv::rectangle(
        overlay, bgRect, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA
    );
    double alpha = 0.4;
    cv::addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img);

    // Draw status indicator blob
    cv::circle(
        img,
        cv::Point(boxPos.x + padding - 2, boxPos.y + box_height/2),
        10,
        state_color,
        cv::FILLED,
        cv::LINE_AA
    );

    // Draw glowing text
    cv::Point text_pos(
        boxPos.x + 2*padding,
        boxPos.y + text_size.height + padding
    );
    cv::putText(
        img, text, text_pos, font_face, font_scale, glow_color, thickness, cv::LINE_AA
    );
}

void resize_compress_then_publish(
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & pub,
    const cv::Mat & img,
    float scale,
    const std::string & encoding,
    int quality)
{
    if (img.empty())
    {
        cerr << "Image is empty, cannot publish" << endl;
        return;
    }

    // Resize the image if scale > 0
    cv::Mat resized;
    if (scale > 0)
        cv::resize(img, resized, cv::Size(), scale, scale);
    else
        resized = img;

    // Encode the image
    std::vector<uchar> buffer; 
    std::vector<int> params;
    if (encoding == "jpeg")
    {
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(quality);
    }
    else if (encoding == "png")
    {
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(3);
    }
    else
    {
        cerr << "Unsupported encoding: " << encoding << endl;
        return;
    }

    std::string ext = (encoding == "jpeg") ? ".jpg" : ".png";

    if (!cv::imencode(ext, resized, buffer, params))
    {
        cerr << "Failed to encode image as " << encoding << endl;
        return;
    }

    // Fill and publish the message
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.format = encoding;
    msg.data = std::move(buffer);
    pub->publish(msg);
}

VoteBin quantizeVote(
    float dx,
    float dy,
    float dscale,
    float dorient,
    cv::Size img_size)
{
    int max_dim = std::max(img_size.width, img_size.height);
    int qx = static_cast<int>(dx / (0.25 * max_dim));
    int qy = static_cast<int>(dy / (0.25 * max_dim));
    int qscale = static_cast<int>(std::log2(dscale + 1e-6));
    int qorient = static_cast<int>(dorient / 30) % 12;

    return {qx, qy, qscale, qorient};
}

bool cropSeaHorizon(
    const cv::Mat & img,
    cv::Mat & img_show,  // for visualization
    cv::Mat & out_img,
    int & out_crop_top_y,  // the y-coordinate of the top of the cropped image
    // Additional arguments to control the cropping area 
    const int upper_bandwidth,
    const int lower_bandwidth,
    bool visualize)
{
    cv::Mat gray, edges;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 50, 150, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 100, 100, 10);

    // Find the most hieghtest horizontal line
    int horizon_y = -1;
    cv::Vec4i best_line;
    int min_y = img.rows;

    for (const auto & line : lines)
    {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        double angle = atan2(y2 - y1, x2 - x1) * 180 / CV_PI;

        if (abs(angle) < 10.0)
        {
            int y_avg = (y1 + y2) / 2;
            if (y_avg < min_y)
            {
                min_y = y_avg;
                best_line = line;
                horizon_y = y_avg;
            }
        }
    }

    if (horizon_y != -1)
    {
        int roi_top_y = std::max(
            0,
            horizon_y - upper_bandwidth
        );
        out_crop_top_y = roi_top_y;

        int roi_height = std::min(
            img.rows - roi_top_y,
            upper_bandwidth + lower_bandwidth 
        );
        cv::Rect roi(0, roi_top_y, img.cols, roi_height);
        out_img = img(roi).clone();

        if (visualize)
        {
            cv::line(img_show, 
                     cv::Point(best_line[0], best_line[1]), 
                     cv::Point(best_line[2], best_line[3]), 
                     cv::Scalar(0, 255, 0), 3);
        }
        return true;
    }
    else
        return false;
}


bool blobVoting(
    cv::Ptr<cv::MSER> mser,
    cv::Mat & img_show,
    const cv::Mat & cropped_img,
    std::vector<int> & out_blob_hist,
    bool visualize,
    // Additional arguments for visualization
    // crop_top_y is the y-coordinate of the top of cropped_img
    int crop_top_y)
{
    int num_x_bins = out_blob_hist.size();
    int x_bin_size = cropped_img.cols / num_x_bins;
    std::vector<cv::Rect> rois;
    cv::Mat cropped_gray;
    cv::cvtColor(cropped_img, cropped_gray, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point>> blobs;
    std::vector<cv::Rect> blob_bboxes;

    mser->detectRegions(cropped_gray, blobs, blob_bboxes);
    bool found_blob = false;

    for (const auto & bbox : blob_bboxes)
    {
        if (!isSizeValid(bbox) ||
            !isAspectRatioValid(bbox) ||
            !isColorValid(cropped_img(bbox)) ||
            !isHighTexture(cropped_img(bbox)))
            continue;
        else
            found_blob = true;

        float center_x = bbox.x + bbox.width / 2;
        float center_y = bbox.y + bbox.height / 2;

        int bin_i = center_x / x_bin_size;
        if (bin_i >= num_x_bins)
            continue;

        out_blob_hist[bin_i] += 1;

        if (!visualize)
            continue;

        // Visualize the detected blobs
        int org_bbox_x = bbox.x;
        int org_bbox_y = crop_top_y + bbox.y;
        cv::Rect org_bbox(
            org_bbox_x,
            org_bbox_y,
            bbox.width,
            bbox.height
        );
        cv::rectangle(cropped_img, bbox, cv::Scalar(0, 0, 255), 2);
    }

    return found_blob;
}

bool isSizeValid(
    const cv::Rect & roi,
    int min_size,
    int max_size)
{
    return roi.width > min_size && roi.height > min_size &&
           roi.width < max_size && roi.height < max_size;
}

bool isAspectRatioValid(
    const cv::Rect & roi,
    double min_aspect_ratio,
    double max_aspect_ratio)
{
    double aspect_ratio = static_cast<double>(roi.width) / roi.height;
    return aspect_ratio >= min_aspect_ratio && aspect_ratio <= max_aspect_ratio;
}

bool isColorValid(
    const cv::Mat & roi,
    double max_brightness)
{
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    cv::Mat V = hsv_channels[2];
    cv::Scalar mean_val = cv::mean(V);
    double mean_brightness = mean_val[0];

    return mean_brightness < max_brightness;
}

bool isHighTexture(
    const cv::Mat & roi,
    double threshold)
{
    cv::Mat lap;
    cv::Laplacian(roi, lap, CV_64F);
    cv::Scalar mu, sigma;
    cv::meanStdDev(lap, mu, sigma);
    double lap_var = sigma[0] * sigma[0];

    return lap_var > threshold;
}

int findMostVotedPosition(
    std::vector<int> & blob_hist,
    int bin_width,
    int crop_top_y,
    cv::Rect & out_bbox)
{
    int max_val = 0;
    int max_loc = -1;

    // Find the most voted bin
    for (int i = 0; i < blob_hist.size(); ++i)
    {
        if (blob_hist[i] > max_val)
        {
            max_val = blob_hist[i];
            max_loc = i;
        }
    }

    out_bbox.x = max_loc * bin_width;
    out_bbox.y = crop_top_y;
    out_bbox.width = bin_width;
    out_bbox.height = 20;

    return max_val;
}

void zoomIn(
    const cv::Mat & img,
    const cv::Rect & bbox,
    cv::Mat & out_img)
{
    cv::resize(img(bbox), out_img, cv::Size(), 30.0, 30.0, cv::INTER_LINEAR);
}

bool recognizeTarget(
    cv::Ptr<cv::SIFT> sift,
    cv::Ptr<cv::FlannBasedMatcher> flann,
    const cv::Mat & img,
    const system_component::TargetData & target_data,
    // For detect result visualization
    int & out_img_match_idx,
    std::vector<cv::KeyPoint> & out_kp_frame,
    std::vector<cv::DMatch> & out_best_cluster,
    vector<Point2f> & out_detect_corners)
{
    cv::Mat desc_frame;

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    sift->detectAndCompute(img_gray, cv::noArray(), out_kp_frame, desc_frame);

    if (desc_frame.empty() || target_data.descriptors.empty()) {
        return false;
    }

    // Find the best match from the batch of target images through FLANN
    std::size_t num_good_matches = 0;
    int best_match_idx = -1;
    vector<cv::DMatch> best_matches;

    for (std::size_t i = 0; i < target_data.filepaths.size(); ++i)
    {
        const cv::Mat & desc_target = target_data.descriptors[i];

        std::vector<std::vector<cv::DMatch>> knn_matches;

        try {
            flann->knnMatch(desc_frame, desc_target, knn_matches, 2);
        } catch (const cv::Exception & e) {
            return false;
        }

        std::vector<cv::DMatch> matches;
        for (const auto & m : knn_matches)
        {
            if (m.size() < 2)
                continue;
            if (m[0].distance < 0.7 * m[1].distance)
                matches.push_back(m[0]);
        }

        if (matches.size() > 3 &&
            matches.size() > num_good_matches)
        {
            num_good_matches = matches.size();
            best_match_idx = i;
            best_matches = matches;
        }
    }

    // std::cout << "Number of good matches: " << num_good_matches << std::endl;

    if (best_match_idx == -1)
    {
        return false;
    }
    else
        out_img_match_idx = best_match_idx;

    // Use Hough transform to find the best cluster from the best-matched keypoints
    map<VoteBin, vector<DMatch>> houghBins;
    
    for (const DMatch & m : best_matches)
    {
        const KeyPoint & kp_query = out_kp_frame[m.queryIdx];
        const KeyPoint & kp_train = target_data.keypoints[out_img_match_idx][m.trainIdx];

        float scale_ratio = kp_train.size / kp_query.size;
        float orient_diff = fmod(kp_train.angle - kp_query.angle + 360.0f, 360.0f);
        float dx = kp_train.pt.x - kp_query.pt.x;
        float dy = kp_train.pt.y - kp_query.pt.y;

        VoteBin bin = quantizeVote(dx, dy, scale_ratio, orient_diff, img.size());
        houghBins[bin].push_back(m);
    }

    size_t max_votes = 0;
    for (auto & [bin, matches] : houghBins)
    {
        if (matches.size() > max_votes)
        {
            max_votes = matches.size();
            out_best_cluster = matches;
        }
    }

    if (out_best_cluster.size() < 4)
    {
        // std::cout << "Only " << out_best_cluster.size() << " matches. Not enough matches in best cluster." << std::endl;
        return false;
    }
    // std::cout << "Best cluster size: " << out_best_cluster.size() << std::endl;

    // Estimate homography through RANSAC and compute bounding box
    std::vector<cv::Point2f> src_pts;
    std::vector<cv::Point2f> dst_pts;

    for (const auto & m : out_best_cluster)
    {
        src_pts.push_back(target_data.keypoints[out_img_match_idx][m.trainIdx].pt);
        dst_pts.push_back(out_kp_frame[m.queryIdx].pt);
    }

    if (src_pts.size() < 4)
        return false;

    cv::Mat H;
    std::vector<unsigned char> inliers_mask;
    H = cv::findHomography(src_pts, dst_pts, cv::RANSAC, 3.0, inliers_mask);

    if (H.empty())
    {
        // std::cerr << "findHomography failed: Homography matrix is empty." << std::endl;
        return false;
    }

    cv::Mat target_img = cv::imread(
        target_data.filepaths[out_img_match_idx]
    );
    vector<Point2f> obj_corners = {
        Point2f(0,0), Point2f(target_img.cols, 0),
        Point2f(target_img.cols, target_img.rows), Point2f(0, target_img.rows)
    };
    out_detect_corners.clear();
    perspectiveTransform(obj_corners, out_detect_corners, H);

    return true;
}

bool ensureBBoxInBounds(
        cv::Rect & bbox,
        const cv::Mat & img)
{
    // Clamp top-left corner
    if (bbox.x < 0)
        bbox.x = 0;
    else if (bbox.x >= img.cols)
        return false;

    if (bbox.y < 0)
        bbox.y = 0;
    else if (bbox.y >= img.rows)
        return false;

    // Compute bottom-right corner
    int x2 = bbox.x + bbox.width;
    int y2 = bbox.y + bbox.height;

    // Clamp bottom-right corner
    if (x2 <= 0 || y2 <= 0)
        return false;

    if (x2 > img.cols)
        bbox.width = img.cols - bbox.x;
    if (y2 > img.rows)
        bbox.height = img.rows - bbox.y;

    // Ensure bbox is still valid
    if (bbox.width <= 0 || bbox.height <= 0)
        return false;

    return true;
}

bool isValidBBoxAspectRatio(
    const cv::Rect & bbox,
    double max_aspect_ratio)
{
    if (bbox.width <= 1 || bbox.height <= 1)
        return false;

    double aspect_ratio1 = static_cast<double>(bbox.width) / bbox.height;  
    double aspect_ratio2 = static_cast<double>(bbox.height) / bbox.width;
    if (aspect_ratio1 > max_aspect_ratio ||
        aspect_ratio2 > max_aspect_ratio)
        return false;
    else
        return true;
}


}  // namespace bt_action
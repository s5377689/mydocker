#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "ardupilot_msgs/msg/global_position.hpp"

#include "system_component/target_registry.hpp"

namespace bt_action
{

// =============================================
//               Navigation Helpers
// =============================================
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

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

double haversine(double lat1, double lon1, double lat2, double lon2);

bool reachedTargetWaypoint(
    double latitude,
    double longitude,
    ardupilot_msgs::msg::GlobalPosition & wp,
    double tolerance_meters
);

// =============================================
//                 Search Helpers
// =============================================
enum class SearchState : char
{
    Start,  // Placeholder for the first state transition (state from)
    Scanning,
    Detecting,
    Recognizing,
    Tracking
};

bool is_inside_rect(
    const ardupilot_msgs::msg::GlobalPosition & point,
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & rect
);
bool isWithinBounds(float pan, float from, float to, bool rotating_right);
double normalize_angle(double angle);
double bearing_deg(double lat1, double lon1, double lat2, double lon2);
double get_yaw_from_quat(double x, double y, double z, double w);

std::pair<double, double> compute_scan_sector(
    double usv_lat, double usv_lon,
    double qx, double qy, double qz, double qw,
    const std::vector<ardupilot_msgs::msg::GlobalPosition> & scan_corners 
);

double normalizeLon(double lon);
double toStdLon(double lon);

// out_rect: axis-aligned rectangle covering the waypoints
// corners order: lower-left, lower-right, upper-right, upper-left
void calculateMinCoverRect(
    const std::vector<ardupilot_msgs::msg::GlobalPosition>& waypoints,
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_rect,
    double margin
);
void calculateMinCoverRect(
    const std::vector<ardupilot_msgs::msg::GlobalPosition>& waypoints,
    int wp_from,
    int wp_to,
    std::vector<ardupilot_msgs::msg::GlobalPosition> & out_rect,
    double margin=0.001
);

double clockwise_diff(
    double from_deg, 
    double to_deg
);

bool is_in_clockwise_sector(
    double from_deg, 
    double to_deg, 
    double test_deg
);

void overlayHistogram(
    cv::Mat & img,
    const std::vector<int> & votes,
    float alpha = 0.4f,
    int highlight_threshold = 15,
    int bin_unit_height = 5,
    int bin_margin = 2
);

void drawStatusLabel(
    cv::Mat & img,
    const SearchState state,
    float glow_period = 1.f  // in seconds
);

void resize_compress_then_publish(
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & pub,
    const cv::Mat & img,
    float scale = 0.25,
    const std::string & encoding = "jpeg",
    int quality = 75
);

struct VoteBin {
    int qx, qy, qscale, qorient;

    bool operator<(const VoteBin & other) const
    {
        return std::tie(qx, qy, qscale, qorient) <
               std::tie(other.qx, other.qy, other.qscale, other.qorient);
    }
};

VoteBin quantizeVote(
    float dx,
    float dy,
    float dscale,
    float dorient,
    cv::Size img_size);

void processQueryImage(
    const cv::Mat& img_train,
    const cv::Mat& img_query);

bool cropSeaHorizon(
    const cv::Mat & img,
    cv::Mat & img_show,
    cv::Mat & out_img,
    int & out_crop_top_y,
    const int upper_bandwidth = 20,
    const int lower_bandwidth = 20,
    bool visualize = false
);

// Accumulate the voting results over the position of the blobs
// to find the most likely position of the target before zooming in
bool blobVoting(
    cv::Ptr<cv::MSER> mser,
    cv::Mat & img_show,
    const cv::Mat & cropped_img,
    std::vector<int> & out_blob_hist,
    bool visualize = false,
    // Additional arguments for visualization
    // crop_top_y is the y-coordinate of the top of cropped_img
    int crop_top_y = 0
);
bool isSizeValid(
    const cv::Rect & roi,
    int min_size = 1,
    int max_size = 50
);
bool isAspectRatioValid(
    const cv::Rect & roi,
    double min_aspect_ratio = 0.9,
    double max_aspect_ratio = 5.5
);
bool isColorValid(
    const cv::Mat & roi,
    double max_brightness = 180.0
);
bool isHighTexture(
    const cv::Mat & roi,
    double threshold = 100.0
);

int findMostVotedPosition(
    std::vector<int> & blob_hist,
    int bin_width,
    int crop_top_y,
    cv::Rect & out_bbox
);

// TEMP for gazebo
void zoomIn(
    const cv::Mat & img,
    const cv::Rect & bbox,
    cv::Mat & out_img
);

bool recognizeTarget(
    cv::Ptr<cv::SIFT> sift,
    cv::Ptr<cv::FlannBasedMatcher> flann,
    const cv::Mat & img,
    const system_component::TargetData & target_data,
    int & out_img_match_idx,
    std::vector<cv::KeyPoint> & out_kp_frame,
    std::vector<cv::DMatch> & out_best_cluster,
    std::vector<cv::Point2f> & out_detect_corners
);
    
// Retrun true if the bbox is within or can be truncated to be within the image boundarys
// Return false if the bbox is completely out of image boundarys
bool ensureBBoxInBounds(
    cv::Rect & bbox,
    const cv::Mat & img
);

bool isValidBBoxAspectRatio(
    const cv::Rect & bbox,
    double max_aspect_ratio = 40.0
);

}  // namespace bt_action
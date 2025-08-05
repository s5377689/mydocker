#pragma once
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "ardupilot_msgs/msg/global_position.hpp"

#include "bt_action/common_helpers.hpp"
#include "system_component/target_registry.hpp"

namespace bt_action
{

enum class SearchState : char
{
    Start,  // Placeholder for the first state transition (state from)
    Scanning,
    Detecting,
    Recognizing,
    Tracking
};

// Black board to read and write current search state
struct StateBlackBoard
{
    bool vis_state {true};  // Whether to visualize the current state

    // Sea horizon detection and cropping
    bool vis_horizon {false};   // Whether to visualize the cropped sea horizon
    int crop_top_y {0};         // The y-coordinate of the top of the cropped region
    int crop_bottom_y {0};
    int upper_margin {20};      // Upper margin for the cropped sea horizon region
    int lower_margin {20};      // Lower margin for the cropped sea horizon region
    cv::Vec4i best_line;        // Best detected line for the sea horizon

    // Blob voting for possible target positions to zoom in
    bool vis_blobs {false};     // Whether to visualize all detected blobs
    bool vis_hist {false};      // Whether to visualize the voting histogram
    bool vis_max_vote {false};  // Whether to visualize the maximum voted position
    int num_x_bins {20};
    std::vector<int> blob_hist;
    std::vector<cv::Rect> blob_bboxes;  // Only for visualization
    cv::Rect max_bbox;          // The most voted position for zooming in

    // SIFT + FLANN
    bool vis_sift {false};      // Whether to visualize the SIFT keypoints
    int img_match_idx {0};      // Index of the matching image in the target data
    std::vector<cv::KeyPoint> kp1, kp2;  // Keypoints to match
    std::vector<cv::Point2f> detected_corners;
    std::vector<cv::DMatch> matches;  // Matches between keypoints
    std::vector<char> matches_mask;

    // CSRT tracking
    bool vis_tracking {false};  // Whether to visualize the tracking results

    void reset_vis_flags()
    {
        vis_horizon = false;
        vis_blobs = false;
        vis_hist = false;
        vis_max_vote = false;
        vis_sift = false;
        vis_tracking = false;
    }
};

void draw_sea_horizon(
    cv::Mat & img,
    StateBlackBoard & blackboard
);

void draw_votes_histogram(
    cv::Mat & img,
    const std::vector<int> & votes,
    int highlight_threshold = 15,   // hightlight the bin if votes greater than this value
    float alpha = 0.4f,             // transparency of the overlaid histogram
    int bin_unit_height = 5,
    int bin_margin = 2
);

void draw_bbox(
    cv::Mat & img,
    const cv::Rect & bbox,
    const std::string & label = "",
    const cv::Scalar & color = cv::Scalar(0, 0, 255),
    int thickness = 2
);

void draw_circles(
    cv::Mat & img,
    const std::vector<cv::Point2f> & points,
    const cv::Scalar & color = cv::Scalar(0, 0, 255),
    int radius = 6,
    int thickness = -1
);

void draw_blobs(
    cv::Mat & img,
    const StateBlackBoard & blackboard,
    const cv::Scalar & color = cv::Scalar(0, 255, 0),
    int thickness = 2
);

void draw_state_label(
    cv::Mat & img,
    const SearchState state,
    float glow_period = 1.f  // in seconds
);

void draw_sift_matches(
    cv::Mat & img1,
    cv::Mat & img2,
    cv::Mat & out_img,
    const std::vector<cv::KeyPoint> & kp1,
    const std::vector<cv::KeyPoint> & kp2,
    const std::vector<cv::DMatch> & matches,
    const std::vector<char> & matches_mask,
    const std::vector<cv::Point2f> & corners = {},
    bool draw_inliers = true
);

bool find_sea_horizon(
    const cv::Mat & img,
    StateBlackBoard & blackboard
);

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

// Accumulate the voting results over the position of the blobs
// to find the most likely position of the target before zooming in
bool blob_voting(
    cv::Ptr<cv::MSER> mser,
    const cv::Mat & cropped_img,
    StateBlackBoard & blackboard
);
bool is_size_valid(
    const cv::Rect & roi,
    int min_size = 1,
    int max_size = 50
);
bool is_aspect_ratio_valid(
    const cv::Rect & roi,
    double min_aspect_ratio = 0.9,
    double max_aspect_ratio = 5.5
);
bool is_color_valid(
    const cv::Mat & roi,
    double max_brightness = 180.0
);
bool is_high_texture(
    const cv::Mat & roi,
    double threshold = 100.0
);

bool find_most_voted_position(
    std::vector<int> & blob_hist,
    int bin_width,
    int crop_top_y,
    int crop_bottom_y,
    cv::Rect & out_bbox,
    int winner_threshold = 0
);

bool recognize_target(
    cv::Ptr<cv::SIFT> sift,
    cv::Ptr<cv::FlannBasedMatcher> flann,
    const cv::Mat & img,
    const system_component::TargetData & target_data,
    int & out_img_match_idx,
    std::vector<cv::KeyPoint> & out_kp_frame,
    std::vector<cv::DMatch> & out_best_cluster,
    std::vector<cv::Point2f> & out_detected_corners,
    std::vector<char> & out_matches_mask
);
    
// Retrun true if the bbox is within or can be truncated to be within the image boundarys
// Return false if the bbox is completely out of image boundarys
bool ensure_bbox_in_bounds(
    cv::Rect & bbox,
    const cv::Mat & img
);

bool is_valid_bbox_aspect_ratio(
    const cv::Rect & bbox,
    double max_aspect_ratio = 40.0
);

}  // namespace bt_action
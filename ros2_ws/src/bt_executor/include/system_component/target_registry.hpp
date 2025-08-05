#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace system_component
{

struct TargetData
{
    std::vector<std::string> filepaths;
    std::vector<std::vector<cv::KeyPoint>> keypoints;
    std::vector<cv::Mat> descriptors;
};

class TargetRegistry
{
public:
    void register_target(
        const std::string & id,
        const TargetData & data
    );

    bool get_target(
        const std::string & id,
        TargetData & out_data
    );

    std::vector<std::string> get_target_list();

    bool has_target(
        const std::string & id
    );

    void remove_target(
        const std::string & id
    );

    void clear_all_targets();

private:
    std::unordered_map<std::string, TargetData> registry_;
    std::mutex mutex_;
};

} // namespace system_component
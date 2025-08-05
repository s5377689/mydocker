// Example of how to use gimbal package components in another package

#include <rclcpp/rclcpp.hpp>

// Include gimbal headers
#include "gimbal/helpers.hpp"                    // Helper functions
#include "gimbal/gimbal_status.hpp"             // GimbalStatus struct and constants
#include "gcs_side/gimbal_controller.hpp"       // GimbalController class
#include "gcs_side/gimbal_streamer.hpp"         // GimbalStreamer class

class MyGimbalApp : public rclcpp::Node
{
public:
    MyGimbalApp() : Node("my_gimbal_app")
    {
        // Use helper functions
        auto heartbeat_cmd = gimbal::buildHeartbeatCommand();
        auto rotate_cmd = gimbal::buildRotateToCommand(45, -30);
        
        // Use GimbalStatus struct and constants
        gimbal::GimbalStatus status;
        status.yaw = 45.0f;
        status.pitch = -30.0f;
        
        RCLCPP_INFO(this->get_logger(), "Frame size: %dx%d", 
                   GIMBAL_FRAME_WIDTH, GIMBAL_FRAME_HEIGHT);
        
        // Create gimbal controller (requires Qt parent, so this is just an example)
        // gimbal_controller_ = std::make_unique<gimbal::usv::GimbalController>(
        //     nullptr, "192.168.1.100", 37260);
        
        // Create gimbal streamer (requires Qt parent, so this is just an example)  
        // gimbal_streamer_ = std::make_unique<gimbal::GimbalStreamer>(
        //     nullptr, "rtsp://192.168.1.100:8554/stream");
    }

private:
    // std::unique_ptr<gimbal::usv::GimbalController> gimbal_controller_;
    // std::unique_ptr<gimbal::GimbalStreamer> gimbal_streamer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyGimbalApp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

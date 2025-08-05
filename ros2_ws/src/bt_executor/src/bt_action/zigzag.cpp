#include "bt_action/zigzag.hpp"

namespace bt_action
{

Zigzag::Zigzag(
    const std::string & name,
    const BT::NodeConfiguration & config
):
    BtNode(name, config)
{
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ap/cmd_vel",
        10
    );

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.durability_volatile();
    qos.best_effort();
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/ap/imu/experimental/data",
    //     qos,
    //     [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {
    //         y_linear_vel_ = msg->linear_acceleration.x;
    //         z_angular_vel_ = msg->angular_velocity.z;
    //     }
    // );
    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/ap/twist/filtered",
        qos,
        [this] (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            y_linear_vel_ = msg->twist.linear.y;
            z_angular_vel_ = msg->twist.angular.z;
        }
    );
}

void Zigzag::run_action_()
{
    double acc_timeout = 5.0;
    double action_time = 5.0;
    double speed = 1.0;
    double amplitude = 1.0;
    double period = 5.0;
    double damping = 0.5;
    double clamp = 1.0;

    try {
        if (!this->getInput<double>("acc_timeout", acc_timeout)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'acc_timeout' input provided for Zigzag action. Assume 5 sec."
            );
        }
        if (!this->getInput<double>("action_time", action_time)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'action_time' input provided for Zigzag action. Assume 5 sec."
            );
        }
        if (!this->getInput<double>("speed", speed)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'speed' input provided for Zigzag action. Assume 1 m/s."
            );
        }
        if (!this->getInput<double>("amplitude", amplitude)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'amplitude' input provided for Zigzag action. Assume 1 m."
            );
        }
        if (!this->getInput<double>("period", period)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'period' input provided for Zigzag action. Assume 5 sec."
            );
        }
        if (!this->getInput<double>("damping", damping)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'damping' input provided for Zigzag action. Assume 0.6."
            );
        }
        if (!this->getInput<double>("clamp", clamp)) {
            RCLCPP_WARN(
                this->get_logger(),
                "No 'clamp' input provided for Zigzag action. Assume 1.0."
            );
        }
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception during Zigzag action initialization: %s", e.what());
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown exception during Zigzag action initialization.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }

    try {
        auto start_time = this->now();
        rclcpp::Rate rate(10); // 10Hz

        // Accelerate the USV to the desired speed
        geometry_msgs::msg::TwistStamped start_twist;
        start_twist.header.stamp = this->now();
        start_twist.header.frame_id = "base_link";
        start_twist.twist.linear.x = speed;
        start_twist.twist.angular.z = 0.0;
        vel_pub_->publish(start_twist);

        while (rclcpp::ok() && (this->now() - start_time).seconds() < acc_timeout)
        {
            if (halt_requested_) {
                status_ = BT::NodeStatus::FAILURE;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Zigzag action halted during acceleration."
                );
                return;
            }

            if (std::abs(y_linear_vel_) - speed > -0.1)
                break; // Stop accelerating if reaching the desired speed

            vel_pub_->publish(start_twist);
            rate.sleep();
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Finished acceleration, zigzag action started with speed: %.2f m/s, amplitude: %.2f m, period: %.2f s, damping: %.2f, clamp: %.2f",
            y_linear_vel_, amplitude, period, damping, clamp
        );

        const double frequency = 2.0*M_PI / period;
        start_time = this->now();

        while (rclcpp::ok() && (this->now() - start_time).seconds() < action_time)
        {
            if (halt_requested_) {
                status_ = BT::NodeStatus::FAILURE;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Zigzag action halted."
                );
                return;
            }

            double t = (this->now() - start_time).seconds();
            double desired_z = amplitude * std::cos(frequency * t);

            // Clamp output if current z angular velocity is too high
            if (std::abs(z_angular_vel_) > clamp) {
                desired_z = 0.0;
            }
            // desired_z = damping * (desired_z - z_angular_vel_);

            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "base_link";
            msg.twist.linear.x = speed;
            msg.twist.angular.z = desired_z;

            vel_pub_->publish(msg);
            rate.sleep();
        }

        // Stop the USV after motion completes
        geometry_msgs::msg::TwistStamped stop_msg;
        stop_msg.header.stamp = this->now();
        stop_msg.header.frame_id = "base_link";
        stop_msg.twist.linear.x = 0.0;
        stop_msg.twist.linear.y = 0.0;
        stop_msg.twist.linear.z = 0.0;
        stop_msg.twist.angular.y = 0.0;
        stop_msg.twist.angular.x = 0.0;
        stop_msg.twist.angular.z = 0.0;
        vel_pub_->publish(stop_msg);
        RCLCPP_INFO(get_logger(), "Published zero velocity command to stop the vehicle.");
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception during Zigzag action execution: %s", e.what());
        status_ = BT::NodeStatus::FAILURE;
        return;
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown exception during Zigzag action execution.");
        status_ = BT::NodeStatus::FAILURE;
        return;
    }

    status_ = BT::NodeStatus::SUCCESS;    
}

}  // end of namespace bt_action
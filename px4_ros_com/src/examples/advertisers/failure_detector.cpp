#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "px4_msgs/msg/failure_detector_status.hpp"

class FailureDetector : public rclcpp::Node {
public:
    FailureDetector() : Node("motor_failure_node") {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/ad/out/reconstruction_error", 10,
            std::bind(&FailureDetector::error_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<px4_msgs::msg::FailureDetectorStatus>(
            "/fmu/in/failure_detector_status", 10);
    }

private:
    void error_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (msg->data >= 0.04) {
            auto failsafe_msg = px4_msgs::msg::FailureDetectorStatus();
            failsafe_msg.timestamp = this->now().nanoseconds()/1000; 
            failsafe_msg.fd_motor = true;
            publisher_->publish(failsafe_msg);
            RCLCPP_WARN(this->get_logger(), "Reconstruction error %.2f exceeds threshold, motor failure flag set!", msg->data);
        } else {
            auto failsafe_msg = px4_msgs::msg::FailureDetectorStatus();
            failsafe_msg.fd_motor = false;
            publisher_->publish(failsafe_msg);
            RCLCPP_INFO(this->get_logger(), "Reconstruction error: %.2f", msg->data);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::FailureDetectorStatus>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FailureDetector>());
    rclcpp::shutdown();
    return 0;
}
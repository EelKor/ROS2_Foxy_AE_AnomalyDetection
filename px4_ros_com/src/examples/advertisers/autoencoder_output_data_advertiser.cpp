#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "px4_msgs/msg/failure_detector_status.hpp"
#include "px4_msgs/msg/autoencoder_outputs.hpp"



class AutoencoderOutputDataAdvertiser : public rclcpp::Node {
public:
    AutoencoderOutputDataAdvertiser() : Node("motor_failure_node") {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/ae/out/autoencoder_input_output_data", 10,
            std::bind(&AutoencoderOutputDataAdvertiser::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<px4_msgs::msg::AutoencoderOutputs>(
            "/fmu/in/autoencoder_outputs", 10);

        // Initialize vectors
        x.resize(12);
        x_hat.resize(12);
        for (int i = 0; i < 12; ++i) {
            x[i] = 0.0;
            x_hat[i] = 0.0;
        }
    }

private:
    void callback(const std_msgs::msg::Float64::SharedPtr msg) {
        
        size_t size = msg->data.size();
        if (size != 24) {
            RCLCPP_ERROR(this->get_logger(), "Received data size is not 24");
            return;
        }
        else {
            for (size_t i = 0; i < 12; ++i) {
                x[i] = msg->data[i];
                x_hat[i] = msg->data[i + 12];
            }
        }
        // Log the dt (TBD)
        // RCLCPP_INFO(this->get_logger(), "dt : %f", )


        auto autoencoder_msg = px4_msgs::msg::AutoencoderOutputs();
        autoencoder_msg.timestamp = this->now().nanoseconds()/1000;
        autoencoder_msg.x = x;
        autoencoder_msg.x_hat = x_hat;
        publisher_->publish(autoencoder_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::AutoencoderOutputs>::SharedPtr publisher_;
    std::vector<double> x;
    std::vector<double> x_hat;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoencoderOutputDataAdvertiser>());
    rclcpp::shutdown();
    return 0;
}

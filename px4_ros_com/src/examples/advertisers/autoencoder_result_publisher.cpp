#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "px4_msgs/msg/autoencoder_outputs.hpp"

#define PX4_MSGS_AUTOENCODER_OUTPUTS_LENGTH 27

class AutoencoderResultPublisher : public rclcpp::Node {
public:
    AutoencoderResultPublisher() : Node("autoencoder_result_publisher") {
        input_output_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/autoencoder/input_outputs", 10,
            std::bind(&AutoencoderResultPublisher::input_output_callback, this, std::placeholders::_1));

        ae_pub_ = this->create_publisher<px4_msgs::msg::AutoencoderOutputs>(
            "/fmu/in/autoencoder_outputs", 10);
    }

private:
    void input_output_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    
        const double SENTINEL = -9999.0;
        const auto& data = msg->data;

        std::vector<double> input_data;
        std::vector<double> output_data;

        auto it = std::find(data.begin(), data.end(), SENTINEL);
        if (it != data.end()) {
            size_t idx = std::distance(data.begin(), it);
            input_data.assign(data.begin(), data.begin() + idx);
            output_data.assign(data.begin() + idx + 1, data.end());
            
            auto ae_msg = px4_msgs::msg::AutoencoderOutputs();
            ae_msg.timestamp = this->now().nanoseconds() / 1000;

            for(size_t i=0; i < PX4_MSGS_AUTOENCODER_OUTPUTS_LENGTH; ++i) {
                if (i < input_data.size()) ae_msg.x[i] = static_cast<float>(input_data[i]);
                else                       ae_msg.x[i] = 0.0f; // Fill with zero if not enough data

                if (i < output_data.size()) ae_msg.x_hat[i] = static_cast<float>(output_data[i]);
                else                        ae_msg.x_hat[i] = 0.0f; // Fill with zero if not enough data
            }
            ae_msg.input_size = output_data.size();
            ae_pub_->publish(ae_msg);


            RCLCPP_INFO(this->get_logger(), "Received input size: %zu, output size: %zu",
            input_data.size(), output_data.size());
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Sentinel value not found in input data");
        }
    }


    std::array<float, PX4_MSGS_AUTOENCODER_OUTPUTS_LENGTH> x_ = {};
    std::array<float, PX4_MSGS_AUTOENCODER_OUTPUTS_LENGTH> x_hat_ = {};

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_output_sub_;
    rclcpp::Publisher<px4_msgs::msg::AutoencoderOutputs>::SharedPtr ae_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoencoderResultPublisher>());
    rclcpp::shutdown();
    return 0;
}

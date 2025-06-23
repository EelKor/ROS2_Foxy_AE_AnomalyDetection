#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "px4_msgs/msg/failure_detector_status.hpp"
#include "px4_msgs/msg/autoencoder_outputs.hpp"



class AutoencoderOutputDataAdvertiser : public rclcpp::Node {
public:
    AutoencoderOutputDataAdvertiser() : Node("autoencoder_output_data_advertiser") {
        // Create a QoS profile for the subscription
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // Create a subscription to the autoencoder input/output data
        // The topic name is "/ae/out/autoencoder_input_output_data"
        // The message type is std_msgs::msg::Float32MultiArray
        // The queue size is 10
        // The callback function is bound to the class method "callback"
        // The callback function will be called when a new message is received
        // The callback function will receive a shared pointer to the message
        // The callback function will be called with the message as an argument

        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/dp/in/autoencoder_input_output_data", 10,
            std::bind(&AutoencoderOutputDataAdvertiser::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<px4_msgs::msg::AutoencoderOutputs>(
            "/fmu/in/autoencoder_outputs", 10);

        // Initialize vectors
        x_.resize(12);
        x_hat_.resize(12);
        for (int i = 0; i < 12; ++i) {
            x_[i] = 0.0;
            x_hat_[i] = 0.0;
        }
    }

private:
    // 프로그램 전체에서 하나만 존재하며, 컴파일 시점에 이미 값이 확정돼 있고, 
    // 크기·길이를 표현하기 좋은 부호 없는 정수”를 만들 때 사용하는 표준적인 관용구
    // constexpr는 컴파일 시점에 상수로 평가되는 변수를 정의하는 키워드
    // std::size_t는 C++에서 제공하는 부호 없는 정수형 데이터 타입으로,
    // 메모리의 크기나 배열의 인덱스 등을 표현하는 데 사용됩니다.
    // std::size_t는 unsigned int와 유사하지만, 부호가 없는 정수형으로 음수를 표현할 수 없습니다.

    static constexpr std::size_t NUM_ELEMENTS = 12; 
    static constexpr std::size_t TOTAL_ELEMENTS = 24;

    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        size_t size = msg->data.size();
        if (size != TOTAL_ELEMENTS) {
            RCLCPP_ERROR(this->get_logger(), "Received data size is not 24");
            return;
        }
        else {
            std::copy_n(msg->data.begin(), NUM_ELEMENTS, x_.begin());
            std::copy_n(msg->data.begin() + NUM_ELEMENTS, NUM_ELEMENTS, x_hat_.begin());
        }

        // Log the dt (TBD)
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds() * 1000.0; // Convert to milliseconds;
        last_time_ = now;
        RCLCPP_INFO(this->get_logger(), "dt: %f [ms]", dt);

        auto autoencoder_msg = px4_msgs::msg::AutoencoderOutputs();
        autoencoder_msg.timestamp = this->now().nanoseconds()/1000;
        std::copy_n(x_.begin(), NUM_ELEMENTS, autoencoder_msg.x.begin());
        std::copy_n(x_hat_.begin(), NUM_ELEMENTS, autoencoder_msg.x_hat.begin());
        publisher_->publish(autoencoder_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::AutoencoderOutputs>::SharedPtr publisher_;
    std::vector<float> x_;
    std::vector<float> x_hat_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoencoderOutputDataAdvertiser>());
    rclcpp::shutdown();
    return 0;
}
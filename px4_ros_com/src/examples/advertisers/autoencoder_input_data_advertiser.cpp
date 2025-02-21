/****************************************************************************
 *
 * Copyright 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Debug Vect uORB topic adverstiser example
 * @file debug_vect_advertiser.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include <cmath>

using namespace std::chrono_literals;

class AutoencoderInputDataAdvertiser : public rclcpp::Node
{
public:
	AutoencoderInputDataAdvertiser() : Node("autoencoder_input_data_advertiser")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
		
        // 구독자 생성
        sensor_combined_subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            std::bind(&AutoencoderInputDataAdvertiser::sensor_callback, this, std::placeholders::_1));

		vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&AutoencoderInputDataAdvertiser::attitude_callback, this, std::placeholders::_1));
            
        vehicle_attitude_setpoint_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/out/vehicle_attitude_setpoint", qos,
            std::bind(&AutoencoderInputDataAdvertiser::attitude_setpoint_callback, this, std::placeholders::_1));
    
        vehicle_rates_setpoint_subscription_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
            "/fmu/out/vehicle_rates_setpoint", qos,
            std::bind(&AutoencoderInputDataAdvertiser::rates_setpoint_callback, this, std::placeholders::_1));

        actuator_outputs_subscription_ = this->create_subscription<px4_msgs::msg::ActuatorOutputs>(
            "/fmu/out/actuator_outputs", qos,
            std::bind(&AutoencoderInputDataAdvertiser::actuator_outputs_callback, this, std::placeholders::_1));

    
		// 퍼블리셔 생성
		publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/dp/out/autoencoder_inputs", 5); // dp는 data process 의 약자
        publisher_dt_ = this->create_publisher<std_msgs::msg::UInt32>("/dp/out/dt", 5);
		// 타이머 생성
		timer_ = this->create_wall_timer(
            5ms, std::bind(&AutoencoderInputDataAdvertiser::publish_data, this)); // 100 hz 간격으로 토픽 발행
	}

private:

	// SensorCombined 데이터를 처리하는 콜백 함수
	void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
    {
        //timestamp_ = msg->timestamp.sec + msg->timestamp.nanosec / 1e9f; // in seconds
        timestamp_ = msg->timestamp; // in milliseconds
        if(prev_timestamp_ != 0) dt_ = timestamp_ - prev_timestamp_;
        prev_timestamp_ = timestamp_;
        accel_[0] = msg->accelerometer_m_s2[0];
        accel_[1] = msg->accelerometer_m_s2[1];
        accel_[2] = msg->accelerometer_m_s2[2];

        gyro_[0] = msg->gyro_rad[0];
        gyro_[1] = msg->gyro_rad[1];
        gyro_[2] = msg->gyro_rad[2];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timestamp in microseconds: %d | dt: %d", timestamp_, dt_);
    }

	// VehicleAttitude 데이터를 처리하는 콜백 함수
	void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
	{
		roll_ = atan2(2.0f * (msg->q[0] * msg->q[1] + msg->q[2] * msg->q[3]),
                     1.0f - 2.0f * (msg->q[1] * msg->q[1] + msg->q[2] * msg->q[2]));

		pitch_ = asin(2.0f * (msg->q[0] * msg->q[2] - msg->q[3] * msg->q[1]));

		yaw_ = atan2(2.0f * (msg->q[0] * msg->q[3] + msg->q[1] * msg->q[2]),
                    1.0f - 2.0f * (msg->q[2] * msg->q[2] + msg->q[3] * msg->q[3]));
	}

    void attitude_setpoint_callback(const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg)
    {
        roll_sp_ = atan2(2.0f * (msg->q_d[0] * msg->q_d[1] + msg->q_d[2] * msg->q_d[3]),
                     1.0f - 2.0f * (msg->q_d[1] * msg->q_d[1] + msg->q_d[2] * msg->q_d[2]));

        pitch_sp_ = asin(2.0f * (msg->q_d[0] * msg->q_d[2] - msg->q_d[3] * msg->q_d[1]));

		yaw_sp_ = atan2(2.0f * (msg->q_d[0] * msg->q_d[3] + msg->q_d[1] * msg->q_d[2]),
                    1.0f - 2.0f * (msg->q_d[2] * msg->q_d[2] + msg->q_d[3] * msg->q_d[3]));
    }

    void rates_setpoint_callback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
    {
        roll_rate_sp_ = msg->roll;
        pitch_rate_sp_ = msg->pitch;
		yaw_rate_sp_ = msg->yaw;
    }

    void actuator_outputs_callback(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg)
    {
        for(int i=0; i<12; i++) actuator_outputs_[i] = msg->output[i];
    }

	// Float32MultiArray 데이터를 퍼블리시하는 함수
	void publish_data()
    {
        auto double_msg = std_msgs::msg::Float64MultiArray();

        /*// 데이터를 배열에 추가 (Accel, Gyro, Roll, Pitch, Yaw)
        double_msg.data = {
            timestamp_, // timestamp
            accel_[0], accel_[1], accel_[2], // 가속도
            gyro_[0], gyro_[1], gyro_[2],   // 자이로스코프
            roll_, pitch_, yaw_,             // 오일러 각
            roll_sp_, pitch_sp_, yaw_sp_,
            roll_rate_sp_, pitch_rate_sp_, yaw_rate_sp_,
            actuator_outputs_[0],actuator_outputs_[1],actuator_outputs_[2],
            actuator_outputs_[3],actuator_outputs_[4],actuator_outputs_[5],
            actuator_outputs_[6],actuator_outputs_[7],actuator_outputs_[8],
            actuator_outputs_[9],actuator_outputs_[10],actuator_outputs_[11]};
*/
        double_msg.data = {
                timestamp_, // timestamp
                accel_[0], accel_[1], accel_[2], // 가속도
                gyro_[0], gyro_[1], gyro_[2],   // 자이로스코프
                roll_, pitch_, yaw_            // 오일러 각
                };

        // 로그 출력 및 퍼블리시
        /*
        RCLCPP_INFO(this->get_logger(), "Publishing: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                    float_msg.data[0], float_msg.data[1], float_msg.data[2],
                    float_msg.data[3], float_msg.data[4], float_msg.data[5],
                    float_msg.data[6], float_msg.data[7], float_msg.data[8]);
        */
        publisher_->publish(double_msg);
        publisher_dt_->publish(dt_);
    }

	// 멤버 변수
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_subscription_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_dt_;
    rclcpp::TimerBase::SharedPtr timer_;

	
	// 센서 데이터
    unsigned long timestamp_ = 0;
    unsigned long prev_timestamp_ = 0;
    unsigned long dt_ = 0;
    double accel_[3] = {0.0f, 0.0f, 0.0f}; // 가속도
    double gyro_[3] = {0.0f, 0.0f, 0.0f};  // 자이로스코프

    // 오일러 각
    double roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;

    // Euler Angle Setpoints
    double roll_sp_ = 0.0f, pitch_sp_ = 0.0f, yaw_sp_ = 0.0f;

    // VehicleRatesSetpoint
    double roll_rate_sp_ = 0.0f, pitch_rate_sp_= 0.0f, yaw_rate_sp_= 0.0f;

    //ActuaorOutputs
    double actuator_outputs_[12] = {0.0};
		
};



int main(int argc, char *argv[])
{
	std::cout << "Starting autoencoder_input_data_advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AutoencoderInputDataAdvertiser>());

	rclcpp::shutdown();
	return 0;
}

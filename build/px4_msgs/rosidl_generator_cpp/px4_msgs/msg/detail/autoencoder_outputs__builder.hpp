// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/AutoencoderOutputs.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__BUILDER_HPP_

#include "px4_msgs/msg/detail/autoencoder_outputs__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_AutoencoderOutputs_x_hat
{
public:
  explicit Init_AutoencoderOutputs_x_hat(::px4_msgs::msg::AutoencoderOutputs & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::AutoencoderOutputs x_hat(::px4_msgs::msg::AutoencoderOutputs::_x_hat_type arg)
  {
    msg_.x_hat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::AutoencoderOutputs msg_;
};

class Init_AutoencoderOutputs_x
{
public:
  explicit Init_AutoencoderOutputs_x(::px4_msgs::msg::AutoencoderOutputs & msg)
  : msg_(msg)
  {}
  Init_AutoencoderOutputs_x_hat x(::px4_msgs::msg::AutoencoderOutputs::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_AutoencoderOutputs_x_hat(msg_);
  }

private:
  ::px4_msgs::msg::AutoencoderOutputs msg_;
};

class Init_AutoencoderOutputs_timestamp
{
public:
  Init_AutoencoderOutputs_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AutoencoderOutputs_x timestamp(::px4_msgs::msg::AutoencoderOutputs::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_AutoencoderOutputs_x(msg_);
  }

private:
  ::px4_msgs::msg::AutoencoderOutputs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::AutoencoderOutputs>()
{
  return px4_msgs::msg::builder::Init_AutoencoderOutputs_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/AutoencoderOutputs.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__AutoencoderOutputs __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__AutoencoderOutputs __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AutoencoderOutputs_
{
  using Type = AutoencoderOutputs_<ContainerAllocator>;

  explicit AutoencoderOutputs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      std::fill<typename std::array<float, 12>::iterator, float>(this->x.begin(), this->x.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->x_hat.begin(), this->x_hat.end(), 0.0f);
    }
  }

  explicit AutoencoderOutputs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : x(_alloc),
    x_hat(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      std::fill<typename std::array<float, 12>::iterator, float>(this->x.begin(), this->x.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->x_hat.begin(), this->x_hat.end(), 0.0f);
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _x_type =
    std::array<float, 12>;
  _x_type x;
  using _x_hat_type =
    std::array<float, 12>;
  _x_hat_type x_hat;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__x(
    const std::array<float, 12> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__x_hat(
    const std::array<float, 12> & _arg)
  {
    this->x_hat = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__AutoencoderOutputs
    std::shared_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__AutoencoderOutputs
    std::shared_ptr<px4_msgs::msg::AutoencoderOutputs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AutoencoderOutputs_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->x_hat != other.x_hat) {
      return false;
    }
    return true;
  }
  bool operator!=(const AutoencoderOutputs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AutoencoderOutputs_

// alias to use template instance with default allocator
using AutoencoderOutputs =
  px4_msgs::msg::AutoencoderOutputs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__STRUCT_HPP_

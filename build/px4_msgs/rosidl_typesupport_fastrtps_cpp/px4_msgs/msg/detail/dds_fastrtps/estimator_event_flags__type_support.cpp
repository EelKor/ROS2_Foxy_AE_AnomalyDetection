// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/EstimatorEventFlags.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/estimator_event_flags__rosidl_typesupport_fastrtps_cpp.hpp"
#include "px4_msgs/msg/detail/estimator_event_flags__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace px4_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_serialize(
  const px4_msgs::msg::EstimatorEventFlags & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: timestamp_sample
  cdr << ros_message.timestamp_sample;
  // Member: information_event_changes
  cdr << ros_message.information_event_changes;
  // Member: gps_checks_passed
  cdr << (ros_message.gps_checks_passed ? true : false);
  // Member: reset_vel_to_gps
  cdr << (ros_message.reset_vel_to_gps ? true : false);
  // Member: reset_vel_to_flow
  cdr << (ros_message.reset_vel_to_flow ? true : false);
  // Member: reset_vel_to_vision
  cdr << (ros_message.reset_vel_to_vision ? true : false);
  // Member: reset_vel_to_zero
  cdr << (ros_message.reset_vel_to_zero ? true : false);
  // Member: reset_pos_to_last_known
  cdr << (ros_message.reset_pos_to_last_known ? true : false);
  // Member: reset_pos_to_gps
  cdr << (ros_message.reset_pos_to_gps ? true : false);
  // Member: reset_pos_to_vision
  cdr << (ros_message.reset_pos_to_vision ? true : false);
  // Member: starting_gps_fusion
  cdr << (ros_message.starting_gps_fusion ? true : false);
  // Member: starting_vision_pos_fusion
  cdr << (ros_message.starting_vision_pos_fusion ? true : false);
  // Member: starting_vision_vel_fusion
  cdr << (ros_message.starting_vision_vel_fusion ? true : false);
  // Member: starting_vision_yaw_fusion
  cdr << (ros_message.starting_vision_yaw_fusion ? true : false);
  // Member: yaw_aligned_to_imu_gps
  cdr << (ros_message.yaw_aligned_to_imu_gps ? true : false);
  // Member: reset_hgt_to_baro
  cdr << (ros_message.reset_hgt_to_baro ? true : false);
  // Member: reset_hgt_to_gps
  cdr << (ros_message.reset_hgt_to_gps ? true : false);
  // Member: reset_hgt_to_rng
  cdr << (ros_message.reset_hgt_to_rng ? true : false);
  // Member: reset_hgt_to_ev
  cdr << (ros_message.reset_hgt_to_ev ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs::msg::EstimatorEventFlags & ros_message)
{
  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: timestamp_sample
  cdr >> ros_message.timestamp_sample;

  // Member: information_event_changes
  cdr >> ros_message.information_event_changes;

  // Member: gps_checks_passed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.gps_checks_passed = tmp ? true : false;
  }

  // Member: reset_vel_to_gps
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_vel_to_gps = tmp ? true : false;
  }

  // Member: reset_vel_to_flow
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_vel_to_flow = tmp ? true : false;
  }

  // Member: reset_vel_to_vision
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_vel_to_vision = tmp ? true : false;
  }

  // Member: reset_vel_to_zero
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_vel_to_zero = tmp ? true : false;
  }

  // Member: reset_pos_to_last_known
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_pos_to_last_known = tmp ? true : false;
  }

  // Member: reset_pos_to_gps
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_pos_to_gps = tmp ? true : false;
  }

  // Member: reset_pos_to_vision
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_pos_to_vision = tmp ? true : false;
  }

  // Member: starting_gps_fusion
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.starting_gps_fusion = tmp ? true : false;
  }

  // Member: starting_vision_pos_fusion
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.starting_vision_pos_fusion = tmp ? true : false;
  }

  // Member: starting_vision_vel_fusion
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.starting_vision_vel_fusion = tmp ? true : false;
  }

  // Member: starting_vision_yaw_fusion
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.starting_vision_yaw_fusion = tmp ? true : false;
  }

  // Member: yaw_aligned_to_imu_gps
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.yaw_aligned_to_imu_gps = tmp ? true : false;
  }

  // Member: reset_hgt_to_baro
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_hgt_to_baro = tmp ? true : false;
  }

  // Member: reset_hgt_to_gps
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_hgt_to_gps = tmp ? true : false;
  }

  // Member: reset_hgt_to_rng
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_hgt_to_rng = tmp ? true : false;
  }

  // Member: reset_hgt_to_ev
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset_hgt_to_ev = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
get_serialized_size(
  const px4_msgs::msg::EstimatorEventFlags & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timestamp_sample
  {
    size_t item_size = sizeof(ros_message.timestamp_sample);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: information_event_changes
  {
    size_t item_size = sizeof(ros_message.information_event_changes);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gps_checks_passed
  {
    size_t item_size = sizeof(ros_message.gps_checks_passed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_vel_to_gps
  {
    size_t item_size = sizeof(ros_message.reset_vel_to_gps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_vel_to_flow
  {
    size_t item_size = sizeof(ros_message.reset_vel_to_flow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_vel_to_vision
  {
    size_t item_size = sizeof(ros_message.reset_vel_to_vision);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_vel_to_zero
  {
    size_t item_size = sizeof(ros_message.reset_vel_to_zero);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_pos_to_last_known
  {
    size_t item_size = sizeof(ros_message.reset_pos_to_last_known);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_pos_to_gps
  {
    size_t item_size = sizeof(ros_message.reset_pos_to_gps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_pos_to_vision
  {
    size_t item_size = sizeof(ros_message.reset_pos_to_vision);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: starting_gps_fusion
  {
    size_t item_size = sizeof(ros_message.starting_gps_fusion);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: starting_vision_pos_fusion
  {
    size_t item_size = sizeof(ros_message.starting_vision_pos_fusion);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: starting_vision_vel_fusion
  {
    size_t item_size = sizeof(ros_message.starting_vision_vel_fusion);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: starting_vision_yaw_fusion
  {
    size_t item_size = sizeof(ros_message.starting_vision_yaw_fusion);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: yaw_aligned_to_imu_gps
  {
    size_t item_size = sizeof(ros_message.yaw_aligned_to_imu_gps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_hgt_to_baro
  {
    size_t item_size = sizeof(ros_message.reset_hgt_to_baro);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_hgt_to_gps
  {
    size_t item_size = sizeof(ros_message.reset_hgt_to_gps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_hgt_to_rng
  {
    size_t item_size = sizeof(ros_message.reset_hgt_to_rng);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_hgt_to_ev
  {
    size_t item_size = sizeof(ros_message.reset_hgt_to_ev);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
max_serialized_size_EstimatorEventFlags(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: timestamp_sample
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: information_event_changes
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gps_checks_passed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_vel_to_gps
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_vel_to_flow
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_vel_to_vision
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_vel_to_zero
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_pos_to_last_known
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_pos_to_gps
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_pos_to_vision
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: starting_gps_fusion
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: starting_vision_pos_fusion
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: starting_vision_vel_fusion
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: starting_vision_yaw_fusion
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: yaw_aligned_to_imu_gps
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_hgt_to_baro
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_hgt_to_gps
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_hgt_to_rng
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_hgt_to_ev
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _EstimatorEventFlags__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::EstimatorEventFlags *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EstimatorEventFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<px4_msgs::msg::EstimatorEventFlags *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EstimatorEventFlags__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::EstimatorEventFlags *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EstimatorEventFlags__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_EstimatorEventFlags(full_bounded, 0);
}

static message_type_support_callbacks_t _EstimatorEventFlags__callbacks = {
  "px4_msgs::msg",
  "EstimatorEventFlags",
  _EstimatorEventFlags__cdr_serialize,
  _EstimatorEventFlags__cdr_deserialize,
  _EstimatorEventFlags__get_serialized_size,
  _EstimatorEventFlags__max_serialized_size
};

static rosidl_message_type_support_t _EstimatorEventFlags__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EstimatorEventFlags__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace px4_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_px4_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::EstimatorEventFlags>()
{
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_EstimatorEventFlags__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4_msgs, msg, EstimatorEventFlags)() {
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_EstimatorEventFlags__handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/SystemPower.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/system_power__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/system_power__struct.h"
#include "px4_msgs/msg/detail/system_power__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SystemPower__ros_msg_type = px4_msgs__msg__SystemPower;

static bool _SystemPower__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SystemPower__ros_msg_type * ros_message = static_cast<const _SystemPower__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: voltage5v_v
  {
    cdr << ros_message->voltage5v_v;
  }

  // Field name: voltage_payload_v
  {
    cdr << ros_message->voltage_payload_v;
  }

  // Field name: sensors3v3
  {
    size_t size = 4;
    auto array_ptr = ros_message->sensors3v3;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: sensors3v3_valid
  {
    cdr << ros_message->sensors3v3_valid;
  }

  // Field name: usb_connected
  {
    cdr << ros_message->usb_connected;
  }

  // Field name: brick_valid
  {
    cdr << ros_message->brick_valid;
  }

  // Field name: usb_valid
  {
    cdr << ros_message->usb_valid;
  }

  // Field name: servo_valid
  {
    cdr << ros_message->servo_valid;
  }

  // Field name: periph_5v_oc
  {
    cdr << ros_message->periph_5v_oc;
  }

  // Field name: hipower_5v_oc
  {
    cdr << ros_message->hipower_5v_oc;
  }

  // Field name: comp_5v_valid
  {
    cdr << ros_message->comp_5v_valid;
  }

  // Field name: can1_gps1_5v_valid
  {
    cdr << ros_message->can1_gps1_5v_valid;
  }

  // Field name: payload_v_valid
  {
    cdr << ros_message->payload_v_valid;
  }

  return true;
}

static bool _SystemPower__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SystemPower__ros_msg_type * ros_message = static_cast<_SystemPower__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: voltage5v_v
  {
    cdr >> ros_message->voltage5v_v;
  }

  // Field name: voltage_payload_v
  {
    cdr >> ros_message->voltage_payload_v;
  }

  // Field name: sensors3v3
  {
    size_t size = 4;
    auto array_ptr = ros_message->sensors3v3;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: sensors3v3_valid
  {
    cdr >> ros_message->sensors3v3_valid;
  }

  // Field name: usb_connected
  {
    cdr >> ros_message->usb_connected;
  }

  // Field name: brick_valid
  {
    cdr >> ros_message->brick_valid;
  }

  // Field name: usb_valid
  {
    cdr >> ros_message->usb_valid;
  }

  // Field name: servo_valid
  {
    cdr >> ros_message->servo_valid;
  }

  // Field name: periph_5v_oc
  {
    cdr >> ros_message->periph_5v_oc;
  }

  // Field name: hipower_5v_oc
  {
    cdr >> ros_message->hipower_5v_oc;
  }

  // Field name: comp_5v_valid
  {
    cdr >> ros_message->comp_5v_valid;
  }

  // Field name: can1_gps1_5v_valid
  {
    cdr >> ros_message->can1_gps1_5v_valid;
  }

  // Field name: payload_v_valid
  {
    cdr >> ros_message->payload_v_valid;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__SystemPower(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SystemPower__ros_msg_type * ros_message = static_cast<const _SystemPower__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name voltage5v_v
  {
    size_t item_size = sizeof(ros_message->voltage5v_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name voltage_payload_v
  {
    size_t item_size = sizeof(ros_message->voltage_payload_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sensors3v3
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->sensors3v3;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sensors3v3_valid
  {
    size_t item_size = sizeof(ros_message->sensors3v3_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name usb_connected
  {
    size_t item_size = sizeof(ros_message->usb_connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name brick_valid
  {
    size_t item_size = sizeof(ros_message->brick_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name usb_valid
  {
    size_t item_size = sizeof(ros_message->usb_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name servo_valid
  {
    size_t item_size = sizeof(ros_message->servo_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name periph_5v_oc
  {
    size_t item_size = sizeof(ros_message->periph_5v_oc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hipower_5v_oc
  {
    size_t item_size = sizeof(ros_message->hipower_5v_oc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name comp_5v_valid
  {
    size_t item_size = sizeof(ros_message->comp_5v_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name can1_gps1_5v_valid
  {
    size_t item_size = sizeof(ros_message->can1_gps1_5v_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name payload_v_valid
  {
    size_t item_size = sizeof(ros_message->payload_v_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SystemPower__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__SystemPower(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__SystemPower(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: voltage5v_v
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: voltage_payload_v
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sensors3v3
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sensors3v3_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: usb_connected
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: brick_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: usb_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: servo_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: periph_5v_oc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: hipower_5v_oc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: comp_5v_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: can1_gps1_5v_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: payload_v_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _SystemPower__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_px4_msgs__msg__SystemPower(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SystemPower = {
  "px4_msgs::msg",
  "SystemPower",
  _SystemPower__cdr_serialize,
  _SystemPower__cdr_deserialize,
  _SystemPower__get_serialized_size,
  _SystemPower__max_serialized_size
};

static rosidl_message_type_support_t _SystemPower__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SystemPower,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, SystemPower)() {
  return &_SystemPower__type_support;
}

#if defined(__cplusplus)
}
#endif

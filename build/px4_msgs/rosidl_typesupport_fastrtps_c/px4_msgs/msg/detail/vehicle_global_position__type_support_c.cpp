// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/VehicleGlobalPosition.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_global_position__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/vehicle_global_position__struct.h"
#include "px4_msgs/msg/detail/vehicle_global_position__functions.h"
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


using _VehicleGlobalPosition__ros_msg_type = px4_msgs__msg__VehicleGlobalPosition;

static bool _VehicleGlobalPosition__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VehicleGlobalPosition__ros_msg_type * ros_message = static_cast<const _VehicleGlobalPosition__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr << ros_message->timestamp_sample;
  }

  // Field name: lat
  {
    cdr << ros_message->lat;
  }

  // Field name: lon
  {
    cdr << ros_message->lon;
  }

  // Field name: alt
  {
    cdr << ros_message->alt;
  }

  // Field name: alt_ellipsoid
  {
    cdr << ros_message->alt_ellipsoid;
  }

  // Field name: lat_lon_valid
  {
    cdr << (ros_message->lat_lon_valid ? true : false);
  }

  // Field name: alt_valid
  {
    cdr << (ros_message->alt_valid ? true : false);
  }

  // Field name: delta_alt
  {
    cdr << ros_message->delta_alt;
  }

  // Field name: delta_terrain
  {
    cdr << ros_message->delta_terrain;
  }

  // Field name: lat_lon_reset_counter
  {
    cdr << ros_message->lat_lon_reset_counter;
  }

  // Field name: alt_reset_counter
  {
    cdr << ros_message->alt_reset_counter;
  }

  // Field name: terrain_reset_counter
  {
    cdr << ros_message->terrain_reset_counter;
  }

  // Field name: eph
  {
    cdr << ros_message->eph;
  }

  // Field name: epv
  {
    cdr << ros_message->epv;
  }

  // Field name: terrain_alt
  {
    cdr << ros_message->terrain_alt;
  }

  // Field name: terrain_alt_valid
  {
    cdr << (ros_message->terrain_alt_valid ? true : false);
  }

  // Field name: dead_reckoning
  {
    cdr << (ros_message->dead_reckoning ? true : false);
  }

  return true;
}

static bool _VehicleGlobalPosition__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VehicleGlobalPosition__ros_msg_type * ros_message = static_cast<_VehicleGlobalPosition__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr >> ros_message->timestamp_sample;
  }

  // Field name: lat
  {
    cdr >> ros_message->lat;
  }

  // Field name: lon
  {
    cdr >> ros_message->lon;
  }

  // Field name: alt
  {
    cdr >> ros_message->alt;
  }

  // Field name: alt_ellipsoid
  {
    cdr >> ros_message->alt_ellipsoid;
  }

  // Field name: lat_lon_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->lat_lon_valid = tmp ? true : false;
  }

  // Field name: alt_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->alt_valid = tmp ? true : false;
  }

  // Field name: delta_alt
  {
    cdr >> ros_message->delta_alt;
  }

  // Field name: delta_terrain
  {
    cdr >> ros_message->delta_terrain;
  }

  // Field name: lat_lon_reset_counter
  {
    cdr >> ros_message->lat_lon_reset_counter;
  }

  // Field name: alt_reset_counter
  {
    cdr >> ros_message->alt_reset_counter;
  }

  // Field name: terrain_reset_counter
  {
    cdr >> ros_message->terrain_reset_counter;
  }

  // Field name: eph
  {
    cdr >> ros_message->eph;
  }

  // Field name: epv
  {
    cdr >> ros_message->epv;
  }

  // Field name: terrain_alt
  {
    cdr >> ros_message->terrain_alt;
  }

  // Field name: terrain_alt_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->terrain_alt_valid = tmp ? true : false;
  }

  // Field name: dead_reckoning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->dead_reckoning = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__VehicleGlobalPosition(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VehicleGlobalPosition__ros_msg_type * ros_message = static_cast<const _VehicleGlobalPosition__ros_msg_type *>(untyped_ros_message);
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
  // field.name timestamp_sample
  {
    size_t item_size = sizeof(ros_message->timestamp_sample);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat
  {
    size_t item_size = sizeof(ros_message->lat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lon
  {
    size_t item_size = sizeof(ros_message->lon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name alt
  {
    size_t item_size = sizeof(ros_message->alt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name alt_ellipsoid
  {
    size_t item_size = sizeof(ros_message->alt_ellipsoid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat_lon_valid
  {
    size_t item_size = sizeof(ros_message->lat_lon_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name alt_valid
  {
    size_t item_size = sizeof(ros_message->alt_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name delta_alt
  {
    size_t item_size = sizeof(ros_message->delta_alt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name delta_terrain
  {
    size_t item_size = sizeof(ros_message->delta_terrain);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat_lon_reset_counter
  {
    size_t item_size = sizeof(ros_message->lat_lon_reset_counter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name alt_reset_counter
  {
    size_t item_size = sizeof(ros_message->alt_reset_counter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name terrain_reset_counter
  {
    size_t item_size = sizeof(ros_message->terrain_reset_counter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name eph
  {
    size_t item_size = sizeof(ros_message->eph);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name epv
  {
    size_t item_size = sizeof(ros_message->epv);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name terrain_alt
  {
    size_t item_size = sizeof(ros_message->terrain_alt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name terrain_alt_valid
  {
    size_t item_size = sizeof(ros_message->terrain_alt_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dead_reckoning
  {
    size_t item_size = sizeof(ros_message->dead_reckoning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VehicleGlobalPosition__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__VehicleGlobalPosition(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__VehicleGlobalPosition(
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
  // member: timestamp_sample
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: lat
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: lon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: alt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: alt_ellipsoid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lat_lon_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: alt_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: delta_alt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: delta_terrain
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lat_lon_reset_counter
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: alt_reset_counter
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: terrain_reset_counter
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: eph
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: epv
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: terrain_alt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: terrain_alt_valid
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: dead_reckoning
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _VehicleGlobalPosition__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_px4_msgs__msg__VehicleGlobalPosition(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VehicleGlobalPosition = {
  "px4_msgs::msg",
  "VehicleGlobalPosition",
  _VehicleGlobalPosition__cdr_serialize,
  _VehicleGlobalPosition__cdr_deserialize,
  _VehicleGlobalPosition__get_serialized_size,
  _VehicleGlobalPosition__max_serialized_size
};

static rosidl_message_type_support_t _VehicleGlobalPosition__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VehicleGlobalPosition,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, VehicleGlobalPosition)() {
  return &_VehicleGlobalPosition__type_support;
}

#if defined(__cplusplus)
}
#endif

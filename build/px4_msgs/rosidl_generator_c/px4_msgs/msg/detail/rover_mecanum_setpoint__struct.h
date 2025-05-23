// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverMecanumSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RoverMecanumSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__RoverMecanumSetpoint
{
  uint64_t timestamp;
  float forward_speed_setpoint;
  float forward_speed_setpoint_normalized;
  float lateral_speed_setpoint;
  float lateral_speed_setpoint_normalized;
  float yaw_rate_setpoint;
  float yaw_rate_setpoint_normalized;
  float yaw_setpoint;
} px4_msgs__msg__RoverMecanumSetpoint;

// Struct for a sequence of px4_msgs__msg__RoverMecanumSetpoint.
typedef struct px4_msgs__msg__RoverMecanumSetpoint__Sequence
{
  px4_msgs__msg__RoverMecanumSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverMecanumSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_SETPOINT__STRUCT_H_

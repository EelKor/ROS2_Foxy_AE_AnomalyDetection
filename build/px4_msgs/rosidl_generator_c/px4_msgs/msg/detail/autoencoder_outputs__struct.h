// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/AutoencoderOutputs.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/AutoencoderOutputs in the package px4_msgs.
typedef struct px4_msgs__msg__AutoencoderOutputs
{
  uint64_t timestamp;
  float x[12];
  float x_hat[12];
} px4_msgs__msg__AutoencoderOutputs;

// Struct for a sequence of px4_msgs__msg__AutoencoderOutputs.
typedef struct px4_msgs__msg__AutoencoderOutputs__Sequence
{
  px4_msgs__msg__AutoencoderOutputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__AutoencoderOutputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__AUTOENCODER_OUTPUTS__STRUCT_H_

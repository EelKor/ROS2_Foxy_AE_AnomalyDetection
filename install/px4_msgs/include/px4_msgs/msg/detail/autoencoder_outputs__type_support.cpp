// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/AutoencoderOutputs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "px4_msgs/msg/detail/autoencoder_outputs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace px4_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void AutoencoderOutputs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) px4_msgs::msg::AutoencoderOutputs(_init);
}

void AutoencoderOutputs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<px4_msgs::msg::AutoencoderOutputs *>(message_memory);
  typed_message->~AutoencoderOutputs();
}

size_t size_function__AutoencoderOutputs__x(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__AutoencoderOutputs__x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__AutoencoderOutputs__x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__AutoencoderOutputs__x_hat(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__AutoencoderOutputs__x_hat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__AutoencoderOutputs__x_hat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AutoencoderOutputs_message_member_array[3] = {
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs::msg::AutoencoderOutputs, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(px4_msgs::msg::AutoencoderOutputs, x),  // bytes offset in struct
    nullptr,  // default value
    size_function__AutoencoderOutputs__x,  // size() function pointer
    get_const_function__AutoencoderOutputs__x,  // get_const(index) function pointer
    get_function__AutoencoderOutputs__x,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x_hat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(px4_msgs::msg::AutoencoderOutputs, x_hat),  // bytes offset in struct
    nullptr,  // default value
    size_function__AutoencoderOutputs__x_hat,  // size() function pointer
    get_const_function__AutoencoderOutputs__x_hat,  // get_const(index) function pointer
    get_function__AutoencoderOutputs__x_hat,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AutoencoderOutputs_message_members = {
  "px4_msgs::msg",  // message namespace
  "AutoencoderOutputs",  // message name
  3,  // number of fields
  sizeof(px4_msgs::msg::AutoencoderOutputs),
  AutoencoderOutputs_message_member_array,  // message members
  AutoencoderOutputs_init_function,  // function to initialize message memory (memory has to be allocated)
  AutoencoderOutputs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AutoencoderOutputs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AutoencoderOutputs_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace px4_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::AutoencoderOutputs>()
{
  return &::px4_msgs::msg::rosidl_typesupport_introspection_cpp::AutoencoderOutputs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, px4_msgs, msg, AutoencoderOutputs)() {
  return &::px4_msgs::msg::rosidl_typesupport_introspection_cpp::AutoencoderOutputs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hydrone_msgs:msg/QRCode.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__QR_CODE__FUNCTIONS_H_
#define HYDRONE_MSGS__MSG__DETAIL__QR_CODE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hydrone_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "hydrone_msgs/msg/detail/qr_code__struct.h"

/// Initialize msg/QRCode message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hydrone_msgs__msg__QRCode
 * )) before or use
 * hydrone_msgs__msg__QRCode__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__msg__QRCode__init(hydrone_msgs__msg__QRCode * msg);

/// Finalize msg/QRCode message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__msg__QRCode__fini(hydrone_msgs__msg__QRCode * msg);

/// Create msg/QRCode message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hydrone_msgs__msg__QRCode__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
hydrone_msgs__msg__QRCode *
hydrone_msgs__msg__QRCode__create();

/// Destroy msg/QRCode message.
/**
 * It calls
 * hydrone_msgs__msg__QRCode__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__msg__QRCode__destroy(hydrone_msgs__msg__QRCode * msg);

/// Check for msg/QRCode message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__msg__QRCode__are_equal(const hydrone_msgs__msg__QRCode * lhs, const hydrone_msgs__msg__QRCode * rhs);

/// Copy a msg/QRCode message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__msg__QRCode__copy(
  const hydrone_msgs__msg__QRCode * input,
  hydrone_msgs__msg__QRCode * output);

/// Initialize array of msg/QRCode messages.
/**
 * It allocates the memory for the number of elements and calls
 * hydrone_msgs__msg__QRCode__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__msg__QRCode__Sequence__init(hydrone_msgs__msg__QRCode__Sequence * array, size_t size);

/// Finalize array of msg/QRCode messages.
/**
 * It calls
 * hydrone_msgs__msg__QRCode__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__msg__QRCode__Sequence__fini(hydrone_msgs__msg__QRCode__Sequence * array);

/// Create array of msg/QRCode messages.
/**
 * It allocates the memory for the array and calls
 * hydrone_msgs__msg__QRCode__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
hydrone_msgs__msg__QRCode__Sequence *
hydrone_msgs__msg__QRCode__Sequence__create(size_t size);

/// Destroy array of msg/QRCode messages.
/**
 * It calls
 * hydrone_msgs__msg__QRCode__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__msg__QRCode__Sequence__destroy(hydrone_msgs__msg__QRCode__Sequence * array);

/// Check for msg/QRCode message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__msg__QRCode__Sequence__are_equal(const hydrone_msgs__msg__QRCode__Sequence * lhs, const hydrone_msgs__msg__QRCode__Sequence * rhs);

/// Copy an array of msg/QRCode messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__msg__QRCode__Sequence__copy(
  const hydrone_msgs__msg__QRCode__Sequence * input,
  hydrone_msgs__msg__QRCode__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__MSG__DETAIL__QR_CODE__FUNCTIONS_H_

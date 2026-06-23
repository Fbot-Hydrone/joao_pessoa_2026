// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__FUNCTIONS_H_
#define BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "biguasim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "biguasim_interfaces/msg/detail/imaging_sonar__struct.h"

/// Initialize msg/ImagingSonar message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * biguasim_interfaces__msg__ImagingSonar
 * )) before or use
 * biguasim_interfaces__msg__ImagingSonar__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
bool
biguasim_interfaces__msg__ImagingSonar__init(biguasim_interfaces__msg__ImagingSonar * msg);

/// Finalize msg/ImagingSonar message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
void
biguasim_interfaces__msg__ImagingSonar__fini(biguasim_interfaces__msg__ImagingSonar * msg);

/// Create msg/ImagingSonar message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * biguasim_interfaces__msg__ImagingSonar__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
biguasim_interfaces__msg__ImagingSonar *
biguasim_interfaces__msg__ImagingSonar__create();

/// Destroy msg/ImagingSonar message.
/**
 * It calls
 * biguasim_interfaces__msg__ImagingSonar__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
void
biguasim_interfaces__msg__ImagingSonar__destroy(biguasim_interfaces__msg__ImagingSonar * msg);

/// Check for msg/ImagingSonar message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
bool
biguasim_interfaces__msg__ImagingSonar__are_equal(const biguasim_interfaces__msg__ImagingSonar * lhs, const biguasim_interfaces__msg__ImagingSonar * rhs);

/// Copy a msg/ImagingSonar message.
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
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
bool
biguasim_interfaces__msg__ImagingSonar__copy(
  const biguasim_interfaces__msg__ImagingSonar * input,
  biguasim_interfaces__msg__ImagingSonar * output);

/// Initialize array of msg/ImagingSonar messages.
/**
 * It allocates the memory for the number of elements and calls
 * biguasim_interfaces__msg__ImagingSonar__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
bool
biguasim_interfaces__msg__ImagingSonar__Sequence__init(biguasim_interfaces__msg__ImagingSonar__Sequence * array, size_t size);

/// Finalize array of msg/ImagingSonar messages.
/**
 * It calls
 * biguasim_interfaces__msg__ImagingSonar__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
void
biguasim_interfaces__msg__ImagingSonar__Sequence__fini(biguasim_interfaces__msg__ImagingSonar__Sequence * array);

/// Create array of msg/ImagingSonar messages.
/**
 * It allocates the memory for the array and calls
 * biguasim_interfaces__msg__ImagingSonar__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
biguasim_interfaces__msg__ImagingSonar__Sequence *
biguasim_interfaces__msg__ImagingSonar__Sequence__create(size_t size);

/// Destroy array of msg/ImagingSonar messages.
/**
 * It calls
 * biguasim_interfaces__msg__ImagingSonar__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
void
biguasim_interfaces__msg__ImagingSonar__Sequence__destroy(biguasim_interfaces__msg__ImagingSonar__Sequence * array);

/// Check for msg/ImagingSonar message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
bool
biguasim_interfaces__msg__ImagingSonar__Sequence__are_equal(const biguasim_interfaces__msg__ImagingSonar__Sequence * lhs, const biguasim_interfaces__msg__ImagingSonar__Sequence * rhs);

/// Copy an array of msg/ImagingSonar messages.
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
ROSIDL_GENERATOR_C_PUBLIC_biguasim_interfaces
bool
biguasim_interfaces__msg__ImagingSonar__Sequence__copy(
  const biguasim_interfaces__msg__ImagingSonar__Sequence * input,
  biguasim_interfaces__msg__ImagingSonar__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__FUNCTIONS_H_

// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hydrone_msgs:srv/SetPhase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__FUNCTIONS_H_
#define HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hydrone_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "hydrone_msgs/srv/detail/set_phase__struct.h"

/// Initialize srv/SetPhase message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hydrone_msgs__srv__SetPhase_Request
 * )) before or use
 * hydrone_msgs__srv__SetPhase_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Request__init(hydrone_msgs__srv__SetPhase_Request * msg);

/// Finalize srv/SetPhase message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Request__fini(hydrone_msgs__srv__SetPhase_Request * msg);

/// Create srv/SetPhase message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hydrone_msgs__srv__SetPhase_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
hydrone_msgs__srv__SetPhase_Request *
hydrone_msgs__srv__SetPhase_Request__create();

/// Destroy srv/SetPhase message.
/**
 * It calls
 * hydrone_msgs__srv__SetPhase_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Request__destroy(hydrone_msgs__srv__SetPhase_Request * msg);

/// Check for srv/SetPhase message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Request__are_equal(const hydrone_msgs__srv__SetPhase_Request * lhs, const hydrone_msgs__srv__SetPhase_Request * rhs);

/// Copy a srv/SetPhase message.
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
hydrone_msgs__srv__SetPhase_Request__copy(
  const hydrone_msgs__srv__SetPhase_Request * input,
  hydrone_msgs__srv__SetPhase_Request * output);

/// Initialize array of srv/SetPhase messages.
/**
 * It allocates the memory for the number of elements and calls
 * hydrone_msgs__srv__SetPhase_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Request__Sequence__init(hydrone_msgs__srv__SetPhase_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetPhase messages.
/**
 * It calls
 * hydrone_msgs__srv__SetPhase_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Request__Sequence__fini(hydrone_msgs__srv__SetPhase_Request__Sequence * array);

/// Create array of srv/SetPhase messages.
/**
 * It allocates the memory for the array and calls
 * hydrone_msgs__srv__SetPhase_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
hydrone_msgs__srv__SetPhase_Request__Sequence *
hydrone_msgs__srv__SetPhase_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetPhase messages.
/**
 * It calls
 * hydrone_msgs__srv__SetPhase_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Request__Sequence__destroy(hydrone_msgs__srv__SetPhase_Request__Sequence * array);

/// Check for srv/SetPhase message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Request__Sequence__are_equal(const hydrone_msgs__srv__SetPhase_Request__Sequence * lhs, const hydrone_msgs__srv__SetPhase_Request__Sequence * rhs);

/// Copy an array of srv/SetPhase messages.
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
hydrone_msgs__srv__SetPhase_Request__Sequence__copy(
  const hydrone_msgs__srv__SetPhase_Request__Sequence * input,
  hydrone_msgs__srv__SetPhase_Request__Sequence * output);

/// Initialize srv/SetPhase message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hydrone_msgs__srv__SetPhase_Response
 * )) before or use
 * hydrone_msgs__srv__SetPhase_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Response__init(hydrone_msgs__srv__SetPhase_Response * msg);

/// Finalize srv/SetPhase message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Response__fini(hydrone_msgs__srv__SetPhase_Response * msg);

/// Create srv/SetPhase message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hydrone_msgs__srv__SetPhase_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
hydrone_msgs__srv__SetPhase_Response *
hydrone_msgs__srv__SetPhase_Response__create();

/// Destroy srv/SetPhase message.
/**
 * It calls
 * hydrone_msgs__srv__SetPhase_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Response__destroy(hydrone_msgs__srv__SetPhase_Response * msg);

/// Check for srv/SetPhase message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Response__are_equal(const hydrone_msgs__srv__SetPhase_Response * lhs, const hydrone_msgs__srv__SetPhase_Response * rhs);

/// Copy a srv/SetPhase message.
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
hydrone_msgs__srv__SetPhase_Response__copy(
  const hydrone_msgs__srv__SetPhase_Response * input,
  hydrone_msgs__srv__SetPhase_Response * output);

/// Initialize array of srv/SetPhase messages.
/**
 * It allocates the memory for the number of elements and calls
 * hydrone_msgs__srv__SetPhase_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Response__Sequence__init(hydrone_msgs__srv__SetPhase_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetPhase messages.
/**
 * It calls
 * hydrone_msgs__srv__SetPhase_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Response__Sequence__fini(hydrone_msgs__srv__SetPhase_Response__Sequence * array);

/// Create array of srv/SetPhase messages.
/**
 * It allocates the memory for the array and calls
 * hydrone_msgs__srv__SetPhase_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
hydrone_msgs__srv__SetPhase_Response__Sequence *
hydrone_msgs__srv__SetPhase_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetPhase messages.
/**
 * It calls
 * hydrone_msgs__srv__SetPhase_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
void
hydrone_msgs__srv__SetPhase_Response__Sequence__destroy(hydrone_msgs__srv__SetPhase_Response__Sequence * array);

/// Check for srv/SetPhase message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hydrone_msgs
bool
hydrone_msgs__srv__SetPhase_Response__Sequence__are_equal(const hydrone_msgs__srv__SetPhase_Response__Sequence * lhs, const hydrone_msgs__srv__SetPhase_Response__Sequence * rhs);

/// Copy an array of srv/SetPhase messages.
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
hydrone_msgs__srv__SetPhase_Response__Sequence__copy(
  const hydrone_msgs__srv__SetPhase_Response__Sequence * input,
  hydrone_msgs__srv__SetPhase_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__FUNCTIONS_H_

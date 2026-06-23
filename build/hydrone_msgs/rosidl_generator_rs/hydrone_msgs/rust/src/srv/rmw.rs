#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__srv__SetPhase_Request() -> *const std::ffi::c_void;
}

#[link(name = "hydrone_msgs__rosidl_generator_c")]
extern "C" {
    fn hydrone_msgs__srv__SetPhase_Request__init(msg: *mut SetPhase_Request) -> bool;
    fn hydrone_msgs__srv__SetPhase_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPhase_Request>, size: usize) -> bool;
    fn hydrone_msgs__srv__SetPhase_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPhase_Request>);
    fn hydrone_msgs__srv__SetPhase_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPhase_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPhase_Request>) -> bool;
}

// Corresponds to hydrone_msgs__srv__SetPhase_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPhase_Request {
    /// 1, 2, 3 or 4
    pub phase: u8,

    /// Using open hardware drone?
    pub open_hardware: bool,

    /// Phase 3 only: use two drones simultaneously?
    pub use_two_drones: bool,

}



impl Default for SetPhase_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hydrone_msgs__srv__SetPhase_Request__init(&mut msg as *mut _) {
        panic!("Call to hydrone_msgs__srv__SetPhase_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPhase_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__srv__SetPhase_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__srv__SetPhase_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__srv__SetPhase_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPhase_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPhase_Request where Self: Sized {
  const TYPE_NAME: &'static str = "hydrone_msgs/srv/SetPhase_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__srv__SetPhase_Request() }
  }
}


#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__srv__SetPhase_Response() -> *const std::ffi::c_void;
}

#[link(name = "hydrone_msgs__rosidl_generator_c")]
extern "C" {
    fn hydrone_msgs__srv__SetPhase_Response__init(msg: *mut SetPhase_Response) -> bool;
    fn hydrone_msgs__srv__SetPhase_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPhase_Response>, size: usize) -> bool;
    fn hydrone_msgs__srv__SetPhase_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPhase_Response>);
    fn hydrone_msgs__srv__SetPhase_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPhase_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPhase_Response>) -> bool;
}

// Corresponds to hydrone_msgs__srv__SetPhase_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPhase_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetPhase_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hydrone_msgs__srv__SetPhase_Response__init(&mut msg as *mut _) {
        panic!("Call to hydrone_msgs__srv__SetPhase_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPhase_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__srv__SetPhase_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__srv__SetPhase_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__srv__SetPhase_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPhase_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPhase_Response where Self: Sized {
  const TYPE_NAME: &'static str = "hydrone_msgs/srv/SetPhase_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__srv__SetPhase_Response() }
  }
}






#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__hydrone_msgs__srv__SetPhase() -> *const std::ffi::c_void;
}

// Corresponds to hydrone_msgs__srv__SetPhase
#[allow(missing_docs, non_camel_case_types)]
pub struct SetPhase;

impl rosidl_runtime_rs::Service for SetPhase {
    type Request = SetPhase_Request;
    type Response = SetPhase_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__hydrone_msgs__srv__SetPhase() }
    }
}



#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ArmMotors_Request() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__srv__ArmMotors_Request__init(msg: *mut ArmMotors_Request) -> bool;
    fn ardupilot_msgs__srv__ArmMotors_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ArmMotors_Request>, size: usize) -> bool;
    fn ardupilot_msgs__srv__ArmMotors_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ArmMotors_Request>);
    fn ardupilot_msgs__srv__ArmMotors_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ArmMotors_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ArmMotors_Request>) -> bool;
}

// Corresponds to ardupilot_msgs__srv__ArmMotors_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmMotors_Request {
    /// This service requests the vehicle to arm or disarm its motors.
    /// Set true to arm motors, false to disarm motors.
    pub arm: bool,

}



impl Default for ArmMotors_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__srv__ArmMotors_Request__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__srv__ArmMotors_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ArmMotors_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ArmMotors_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ArmMotors_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ArmMotors_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ArmMotors_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ArmMotors_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/srv/ArmMotors_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ArmMotors_Request() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ArmMotors_Response() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__srv__ArmMotors_Response__init(msg: *mut ArmMotors_Response) -> bool;
    fn ardupilot_msgs__srv__ArmMotors_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ArmMotors_Response>, size: usize) -> bool;
    fn ardupilot_msgs__srv__ArmMotors_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ArmMotors_Response>);
    fn ardupilot_msgs__srv__ArmMotors_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ArmMotors_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ArmMotors_Response>) -> bool;
}

// Corresponds to ardupilot_msgs__srv__ArmMotors_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmMotors_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: bool,

}



impl Default for ArmMotors_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__srv__ArmMotors_Response__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__srv__ArmMotors_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ArmMotors_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ArmMotors_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ArmMotors_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ArmMotors_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ArmMotors_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ArmMotors_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/srv/ArmMotors_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ArmMotors_Response() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ModeSwitch_Request() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__srv__ModeSwitch_Request__init(msg: *mut ModeSwitch_Request) -> bool;
    fn ardupilot_msgs__srv__ModeSwitch_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ModeSwitch_Request>, size: usize) -> bool;
    fn ardupilot_msgs__srv__ModeSwitch_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ModeSwitch_Request>);
    fn ardupilot_msgs__srv__ModeSwitch_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ModeSwitch_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ModeSwitch_Request>) -> bool;
}

// Corresponds to ardupilot_msgs__srv__ModeSwitch_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ModeSwitch_Request {
    /// This service requests the vehicle to switch its drive/flight mode
    /// mode : Set the value to the drive/flight mode to be used
    /// Copter : https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE
    /// Rover  : https://mavlink.io/en/messages/ardupilotmega.html#ROVER_MODE
    /// Plane  : https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE
    pub mode: u8,

}



impl Default for ModeSwitch_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__srv__ModeSwitch_Request__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__srv__ModeSwitch_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ModeSwitch_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ModeSwitch_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ModeSwitch_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ModeSwitch_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ModeSwitch_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ModeSwitch_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/srv/ModeSwitch_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ModeSwitch_Request() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ModeSwitch_Response() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__srv__ModeSwitch_Response__init(msg: *mut ModeSwitch_Response) -> bool;
    fn ardupilot_msgs__srv__ModeSwitch_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ModeSwitch_Response>, size: usize) -> bool;
    fn ardupilot_msgs__srv__ModeSwitch_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ModeSwitch_Response>);
    fn ardupilot_msgs__srv__ModeSwitch_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ModeSwitch_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ModeSwitch_Response>) -> bool;
}

// Corresponds to ardupilot_msgs__srv__ModeSwitch_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ModeSwitch_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub curr_mode: u8,

}



impl Default for ModeSwitch_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__srv__ModeSwitch_Response__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__srv__ModeSwitch_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ModeSwitch_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ModeSwitch_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ModeSwitch_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__ModeSwitch_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ModeSwitch_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ModeSwitch_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/srv/ModeSwitch_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__ModeSwitch_Response() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__Takeoff_Request() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__srv__Takeoff_Request__init(msg: *mut Takeoff_Request) -> bool;
    fn ardupilot_msgs__srv__Takeoff_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Takeoff_Request>, size: usize) -> bool;
    fn ardupilot_msgs__srv__Takeoff_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Takeoff_Request>);
    fn ardupilot_msgs__srv__Takeoff_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Takeoff_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Takeoff_Request>) -> bool;
}

// Corresponds to ardupilot_msgs__srv__Takeoff_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Takeoff_Request {
    /// This service requests the vehicle to takeoff (VTOL style for QuadPlane).
    /// alt : Set the takeoff altitude above home or above terrain(in case of rangefinder)
    pub alt: f32,

}



impl Default for Takeoff_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__srv__Takeoff_Request__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__srv__Takeoff_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Takeoff_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__Takeoff_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__Takeoff_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__Takeoff_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Takeoff_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Takeoff_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/srv/Takeoff_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__Takeoff_Request() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__Takeoff_Response() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__srv__Takeoff_Response__init(msg: *mut Takeoff_Response) -> bool;
    fn ardupilot_msgs__srv__Takeoff_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Takeoff_Response>, size: usize) -> bool;
    fn ardupilot_msgs__srv__Takeoff_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Takeoff_Response>);
    fn ardupilot_msgs__srv__Takeoff_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Takeoff_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Takeoff_Response>) -> bool;
}

// Corresponds to ardupilot_msgs__srv__Takeoff_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Takeoff_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: bool,

}



impl Default for Takeoff_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__srv__Takeoff_Response__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__srv__Takeoff_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Takeoff_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__Takeoff_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__Takeoff_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__srv__Takeoff_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Takeoff_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Takeoff_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/srv/Takeoff_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__srv__Takeoff_Response() }
  }
}






#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ardupilot_msgs__srv__ArmMotors() -> *const std::ffi::c_void;
}

// Corresponds to ardupilot_msgs__srv__ArmMotors
#[allow(missing_docs, non_camel_case_types)]
pub struct ArmMotors;

impl rosidl_runtime_rs::Service for ArmMotors {
    type Request = ArmMotors_Request;
    type Response = ArmMotors_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ardupilot_msgs__srv__ArmMotors() }
    }
}




#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ardupilot_msgs__srv__ModeSwitch() -> *const std::ffi::c_void;
}

// Corresponds to ardupilot_msgs__srv__ModeSwitch
#[allow(missing_docs, non_camel_case_types)]
pub struct ModeSwitch;

impl rosidl_runtime_rs::Service for ModeSwitch {
    type Request = ModeSwitch_Request;
    type Response = ModeSwitch_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ardupilot_msgs__srv__ModeSwitch() }
    }
}




#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ardupilot_msgs__srv__Takeoff() -> *const std::ffi::c_void;
}

// Corresponds to ardupilot_msgs__srv__Takeoff
#[allow(missing_docs, non_camel_case_types)]
pub struct Takeoff;

impl rosidl_runtime_rs::Service for Takeoff {
    type Request = Takeoff_Request;
    type Response = Takeoff_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ardupilot_msgs__srv__Takeoff() }
    }
}



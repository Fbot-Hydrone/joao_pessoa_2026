#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__LandingBase() -> *const std::ffi::c_void;
}

#[link(name = "hydrone_msgs__rosidl_generator_c")]
extern "C" {
    fn hydrone_msgs__msg__LandingBase__init(msg: *mut LandingBase) -> bool;
    fn hydrone_msgs__msg__LandingBase__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LandingBase>, size: usize) -> bool;
    fn hydrone_msgs__msg__LandingBase__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LandingBase>);
    fn hydrone_msgs__msg__LandingBase__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LandingBase>, out_seq: *mut rosidl_runtime_rs::Sequence<LandingBase>) -> bool;
}

// Corresponds to hydrone_msgs__msg__LandingBase
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Represents a detected landing base in the arena

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LandingBase {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// 0-5 (6 bases total)
    pub base_id: u8,

    /// Position and orientation of the base
    pub pose: geometry_msgs::msg::rmw::Pose,

    /// True if base is elevated (suspended)
    pub is_suspended: bool,

    /// Whether drone has already landed here
    pub is_visited: bool,

    /// Detection confidence 0.0-1.0
    pub confidence: f32,

    /// Height above ground in meters
    pub height: f32,

}



impl Default for LandingBase {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hydrone_msgs__msg__LandingBase__init(&mut msg as *mut _) {
        panic!("Call to hydrone_msgs__msg__LandingBase__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LandingBase {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__LandingBase__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__LandingBase__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__LandingBase__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LandingBase {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LandingBase where Self: Sized {
  const TYPE_NAME: &'static str = "hydrone_msgs/msg/LandingBase";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__LandingBase() }
  }
}


#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__MissionState() -> *const std::ffi::c_void;
}

#[link(name = "hydrone_msgs__rosidl_generator_c")]
extern "C" {
    fn hydrone_msgs__msg__MissionState__init(msg: *mut MissionState) -> bool;
    fn hydrone_msgs__msg__MissionState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MissionState>, size: usize) -> bool;
    fn hydrone_msgs__msg__MissionState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MissionState>);
    fn hydrone_msgs__msg__MissionState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MissionState>, out_seq: *mut rosidl_runtime_rs::Sequence<MissionState>) -> bool;
}

// Corresponds to hydrone_msgs__msg__MissionState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Mission state for the state machine

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MissionState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Competition phase: 1, 2, 3 or 4
    pub phase: u8,

    /// Current state within the mission
    pub state: u8,

    /// Human-readable state name
    pub state_name: rosidl_runtime_rs::String,

    /// Accumulated score for current attempt
    pub score: f32,

    /// True if using open hardware (2x multiplier)
    pub open_hardware: bool,

}



impl Default for MissionState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hydrone_msgs__msg__MissionState__init(&mut msg as *mut _) {
        panic!("Call to hydrone_msgs__msg__MissionState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MissionState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__MissionState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__MissionState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__MissionState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MissionState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MissionState where Self: Sized {
  const TYPE_NAME: &'static str = "hydrone_msgs/msg/MissionState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__MissionState() }
  }
}


#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__HumanGesture() -> *const std::ffi::c_void;
}

#[link(name = "hydrone_msgs__rosidl_generator_c")]
extern "C" {
    fn hydrone_msgs__msg__HumanGesture__init(msg: *mut HumanGesture) -> bool;
    fn hydrone_msgs__msg__HumanGesture__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HumanGesture>, size: usize) -> bool;
    fn hydrone_msgs__msg__HumanGesture__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HumanGesture>);
    fn hydrone_msgs__msg__HumanGesture__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HumanGesture>, out_seq: *mut rosidl_runtime_rs::Sequence<HumanGesture>) -> bool;
}

// Corresponds to hydrone_msgs__msg__HumanGesture
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Human gesture/command detected by vision system (Phase 3)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HumanGesture {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// e.g. "arms_up", "point_left", "point_right", etc.
    pub gesture_name: rosidl_runtime_rs::String,

    /// Detection confidence 0.0-1.0
    pub confidence: f32,

    /// Position of the human in the world
    pub human_position: geometry_msgs::msg::rmw::Point,

    /// Flat array of skeleton keypoints (x,y,z per joint)
    pub skeleton_keypoints: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for HumanGesture {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hydrone_msgs__msg__HumanGesture__init(&mut msg as *mut _) {
        panic!("Call to hydrone_msgs__msg__HumanGesture__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HumanGesture {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__HumanGesture__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__HumanGesture__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__HumanGesture__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HumanGesture {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HumanGesture where Self: Sized {
  const TYPE_NAME: &'static str = "hydrone_msgs/msg/HumanGesture";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__HumanGesture() }
  }
}


#[link(name = "hydrone_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__QRCode() -> *const std::ffi::c_void;
}

#[link(name = "hydrone_msgs__rosidl_generator_c")]
extern "C" {
    fn hydrone_msgs__msg__QRCode__init(msg: *mut QRCode) -> bool;
    fn hydrone_msgs__msg__QRCode__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<QRCode>, size: usize) -> bool;
    fn hydrone_msgs__msg__QRCode__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<QRCode>);
    fn hydrone_msgs__msg__QRCode__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<QRCode>, out_seq: *mut rosidl_runtime_rs::Sequence<QRCode>) -> bool;
}

// Corresponds to hydrone_msgs__msg__QRCode
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// QR Code detection result (Phase 4)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct QRCode {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Letter identifier: A, B, C, D or E
    pub qr_id: rosidl_runtime_rs::String,

    /// Estimated pose of the QR code in the world
    pub pose: geometry_msgs::msg::rmw::Pose,

    /// True if this is the first time this QR was detected
    pub is_new: bool,

}



impl Default for QRCode {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hydrone_msgs__msg__QRCode__init(&mut msg as *mut _) {
        panic!("Call to hydrone_msgs__msg__QRCode__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for QRCode {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__QRCode__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__QRCode__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hydrone_msgs__msg__QRCode__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for QRCode {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for QRCode where Self: Sized {
  const TYPE_NAME: &'static str = "hydrone_msgs/msg/QRCode";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hydrone_msgs__msg__QRCode() }
  }
}



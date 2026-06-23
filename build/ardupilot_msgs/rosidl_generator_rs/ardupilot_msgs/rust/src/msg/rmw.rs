#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__GlobalPosition() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__msg__GlobalPosition__init(msg: *mut GlobalPosition) -> bool;
    fn ardupilot_msgs__msg__GlobalPosition__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GlobalPosition>, size: usize) -> bool;
    fn ardupilot_msgs__msg__GlobalPosition__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GlobalPosition>);
    fn ardupilot_msgs__msg__GlobalPosition__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GlobalPosition>, out_seq: *mut rosidl_runtime_rs::Sequence<GlobalPosition>) -> bool;
}

// Corresponds to ardupilot_msgs__msg__GlobalPosition
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Experimental REP-147 Goal Interface
/// https://ros.org/reps/rep-0147.html#goal-interface

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GlobalPosition {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coordinate_frame: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_mask: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub latitude: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub longitude: f64,

    /// in meters, AMSL or above terrain
    pub altitude: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration_or_force: geometry_msgs::msg::rmw::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f32,

}

impl GlobalPosition {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FRAME_GLOBAL_INT: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FRAME_GLOBAL_REL_ALT: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FRAME_GLOBAL_TERRAIN_ALT: u8 = 11;

    /// Position ignore flags
    pub const IGNORE_LATITUDE: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_LONGITUDE: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_ALTITUDE: u16 = 4;

    /// Velocity vector ignore flags
    pub const IGNORE_VX: u16 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_VY: u16 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_VZ: u16 = 32;

    /// Acceleration/Force vector ignore flags
    pub const IGNORE_AFX: u16 = 64;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_AFY: u16 = 128;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_AFZ: u16 = 256;

    /// Force in af vector flag
    pub const FORCE: u16 = 512;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_YAW: u16 = 1024;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IGNORE_YAW_RATE: u16 = 2048;

}


impl Default for GlobalPosition {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__msg__GlobalPosition__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__msg__GlobalPosition__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GlobalPosition {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__GlobalPosition__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__GlobalPosition__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__GlobalPosition__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GlobalPosition {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GlobalPosition where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/msg/GlobalPosition";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__GlobalPosition() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__Rc() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__msg__Rc__init(msg: *mut Rc) -> bool;
    fn ardupilot_msgs__msg__Rc__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Rc>, size: usize) -> bool;
    fn ardupilot_msgs__msg__Rc__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Rc>);
    fn ardupilot_msgs__msg__Rc__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Rc>, out_seq: *mut rosidl_runtime_rs::Sequence<Rc>) -> bool;
}

// Corresponds to ardupilot_msgs__msg__Rc
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Rc {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// returns true if radio is connected.
    pub is_connected: bool,

    /// returns [0, 100] for receiver RSSI.
    pub receiver_rssi: u8,

    /// channels values.
    pub channels: rosidl_runtime_rs::BoundedSequence<i16, 32>,

    /// sets true if a channel is overridden.
    pub active_overrides: rosidl_runtime_rs::BoundedSequence<bool, 32>,

}



impl Default for Rc {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__msg__Rc__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__msg__Rc__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Rc {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Rc__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Rc__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Rc__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Rc {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Rc where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/msg/Rc";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__Rc() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__Status() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__msg__Status__init(msg: *mut Status) -> bool;
    fn ardupilot_msgs__msg__Status__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Status>, size: usize) -> bool;
    fn ardupilot_msgs__msg__Status__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Status>);
    fn ardupilot_msgs__msg__Status__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Status>, out_seq: *mut rosidl_runtime_rs::Sequence<Status>) -> bool;
}

// Corresponds to ardupilot_msgs__msg__Status
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Status {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// From AP_Vehicle_Type.h
    pub vehicle_type: u8,

    /// true if vehicle is armed.
    pub armed: bool,

    /// Vehicle mode, enum depending on vehicle type.
    pub mode: u8,

    /// True if flying/driving/diving/tracking.
    pub flying: bool,

    /// True is external control is enabled.
    pub external_control: bool,

    /// Array containing all active failsafe.
    pub failsafe: rosidl_runtime_rs::Sequence<u8>,

}

impl Status {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_ROVER: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_ARDUCOPTER: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_ARDUPLANE: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_ANTENNATRACKER: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_UNKNOWN: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_REPLAY: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_ARDUSUB: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_IOFIRMWARE: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_AP_PERIPH: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_AP_DAL_STANDALONE: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_AP_BOOTLOADER: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_BLIMP: u8 = 12;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APM_HELI: u8 = 13;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FS_RADIO: u8 = 21;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FS_BATTERY: u8 = 22;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FS_GCS: u8 = 23;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FS_EKF: u8 = 24;

}


impl Default for Status {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__msg__Status__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__msg__Status__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Status {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Status__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Status__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Status__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Status {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Status where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/msg/Status";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__Status() }
  }
}


#[link(name = "ardupilot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__Airspeed() -> *const std::ffi::c_void;
}

#[link(name = "ardupilot_msgs__rosidl_generator_c")]
extern "C" {
    fn ardupilot_msgs__msg__Airspeed__init(msg: *mut Airspeed) -> bool;
    fn ardupilot_msgs__msg__Airspeed__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Airspeed>, size: usize) -> bool;
    fn ardupilot_msgs__msg__Airspeed__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Airspeed>);
    fn ardupilot_msgs__msg__Airspeed__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Airspeed>, out_seq: *mut rosidl_runtime_rs::Sequence<Airspeed>) -> bool;
}

// Corresponds to ardupilot_msgs__msg__Airspeed
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Airspeed {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// True Airspeed vector in ROS REP103 axis orientation
    /// x: forward; y: left; z: up
    pub true_airspeed: geometry_msgs::msg::rmw::Vector3,

    /// Equivalent to True airspeed conversion factor.
    pub eas_2_tas: f32,

}



impl Default for Airspeed {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ardupilot_msgs__msg__Airspeed__init(&mut msg as *mut _) {
        panic!("Call to ardupilot_msgs__msg__Airspeed__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Airspeed {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Airspeed__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Airspeed__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ardupilot_msgs__msg__Airspeed__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Airspeed {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Airspeed where Self: Sized {
  const TYPE_NAME: &'static str = "ardupilot_msgs/msg/Airspeed";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ardupilot_msgs__msg__Airspeed() }
  }
}



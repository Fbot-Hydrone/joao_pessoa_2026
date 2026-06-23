#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "biguasim_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__biguasim_interfaces__msg__DVLSensorRange() -> *const std::ffi::c_void;
}

#[link(name = "biguasim_interfaces__rosidl_generator_c")]
extern "C" {
    fn biguasim_interfaces__msg__DVLSensorRange__init(msg: *mut DVLSensorRange) -> bool;
    fn biguasim_interfaces__msg__DVLSensorRange__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DVLSensorRange>, size: usize) -> bool;
    fn biguasim_interfaces__msg__DVLSensorRange__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DVLSensorRange>);
    fn biguasim_interfaces__msg__DVLSensorRange__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DVLSensorRange>, out_seq: *mut rosidl_runtime_rs::Sequence<DVLSensorRange>) -> bool;
}

// Corresponds to biguasim_interfaces__msg__DVLSensorRange
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// DVLSensor message
/// Contains header, velocity, and range measurements

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DVLSensorRange {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Range measurements in meters from the 4 sonar beams
    pub range: [f32; 4],

}



impl Default for DVLSensorRange {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !biguasim_interfaces__msg__DVLSensorRange__init(&mut msg as *mut _) {
        panic!("Call to biguasim_interfaces__msg__DVLSensorRange__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DVLSensorRange {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { biguasim_interfaces__msg__DVLSensorRange__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { biguasim_interfaces__msg__DVLSensorRange__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { biguasim_interfaces__msg__DVLSensorRange__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DVLSensorRange {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DVLSensorRange where Self: Sized {
  const TYPE_NAME: &'static str = "biguasim_interfaces/msg/DVLSensorRange";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__biguasim_interfaces__msg__DVLSensorRange() }
  }
}


#[link(name = "biguasim_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__biguasim_interfaces__msg__ImagingSonar() -> *const std::ffi::c_void;
}

#[link(name = "biguasim_interfaces__rosidl_generator_c")]
extern "C" {
    fn biguasim_interfaces__msg__ImagingSonar__init(msg: *mut ImagingSonar) -> bool;
    fn biguasim_interfaces__msg__ImagingSonar__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ImagingSonar>, size: usize) -> bool;
    fn biguasim_interfaces__msg__ImagingSonar__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ImagingSonar>);
    fn biguasim_interfaces__msg__ImagingSonar__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ImagingSonar>, out_seq: *mut rosidl_runtime_rs::Sequence<ImagingSonar>) -> bool;
}

// Corresponds to biguasim_interfaces__msg__ImagingSonar
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// ImagingSonar message
/// Contains timestamp, bins information, and image data

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ImagingSonar {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Number of azimuth bins
    pub bins_azimuth: i32,

    /// Number of range bins
    pub bins_range: i32,

    /// Raw sonar image (as received)
    pub raw_image: sensor_msgs::msg::rmw::Image,

    /// Ground-truth intensity (float, same size as raw)
    pub intensity: sensor_msgs::msg::rmw::Image,

    /// Ground-truth elevation (per-pixel angle or height)
    pub elevation: sensor_msgs::msg::rmw::Image,

    /// Ground-truth 3D points
    pub point_cloud: sensor_msgs::msg::rmw::PointCloud2,

}



impl Default for ImagingSonar {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !biguasim_interfaces__msg__ImagingSonar__init(&mut msg as *mut _) {
        panic!("Call to biguasim_interfaces__msg__ImagingSonar__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ImagingSonar {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { biguasim_interfaces__msg__ImagingSonar__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { biguasim_interfaces__msg__ImagingSonar__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { biguasim_interfaces__msg__ImagingSonar__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ImagingSonar {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ImagingSonar where Self: Sized {
  const TYPE_NAME: &'static str = "biguasim_interfaces/msg/ImagingSonar";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__biguasim_interfaces__msg__ImagingSonar() }
  }
}



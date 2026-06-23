#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ardupilot_msgs__msg__GlobalPosition
/// Experimental REP-147 Goal Interface
/// https://ros.org/reps/rep-0147.html#goal-interface

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GlobalPosition {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    pub velocity: geometry_msgs::msg::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration_or_force: geometry_msgs::msg::Twist,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GlobalPosition::default())
  }
}

impl rosidl_runtime_rs::Message for GlobalPosition {
  type RmwMsg = super::msg::rmw::GlobalPosition;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        coordinate_frame: msg.coordinate_frame,
        type_mask: msg.type_mask,
        latitude: msg.latitude,
        longitude: msg.longitude,
        altitude: msg.altitude,
        velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        acceleration_or_force: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.acceleration_or_force)).into_owned(),
        yaw: msg.yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      coordinate_frame: msg.coordinate_frame,
      type_mask: msg.type_mask,
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: msg.altitude,
        velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        acceleration_or_force: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.acceleration_or_force)).into_owned(),
      yaw: msg.yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      coordinate_frame: msg.coordinate_frame,
      type_mask: msg.type_mask,
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: msg.altitude,
      velocity: geometry_msgs::msg::Twist::from_rmw_message(msg.velocity),
      acceleration_or_force: geometry_msgs::msg::Twist::from_rmw_message(msg.acceleration_or_force),
      yaw: msg.yaw,
    }
  }
}


// Corresponds to ardupilot_msgs__msg__Rc

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Rc {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Rc::default())
  }
}

impl rosidl_runtime_rs::Message for Rc {
  type RmwMsg = super::msg::rmw::Rc;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_connected: msg.is_connected,
        receiver_rssi: msg.receiver_rssi,
        channels: msg.channels,
        active_overrides: msg.active_overrides,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_connected: msg.is_connected,
      receiver_rssi: msg.receiver_rssi,
        channels: msg.channels.clone(),
        active_overrides: msg.active_overrides.clone(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_connected: msg.is_connected,
      receiver_rssi: msg.receiver_rssi,
      channels: msg.channels,
      active_overrides: msg.active_overrides,
    }
  }
}


// Corresponds to ardupilot_msgs__msg__Status

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Status {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    pub failsafe: Vec<u8>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Status::default())
  }
}

impl rosidl_runtime_rs::Message for Status {
  type RmwMsg = super::msg::rmw::Status;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        vehicle_type: msg.vehicle_type,
        armed: msg.armed,
        mode: msg.mode,
        flying: msg.flying,
        external_control: msg.external_control,
        failsafe: msg.failsafe.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      vehicle_type: msg.vehicle_type,
      armed: msg.armed,
      mode: msg.mode,
      flying: msg.flying,
      external_control: msg.external_control,
        failsafe: msg.failsafe.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      vehicle_type: msg.vehicle_type,
      armed: msg.armed,
      mode: msg.mode,
      flying: msg.flying,
      external_control: msg.external_control,
      failsafe: msg.failsafe
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ardupilot_msgs__msg__Airspeed

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Airspeed {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// True Airspeed vector in ROS REP103 axis orientation
    /// x: forward; y: left; z: up
    pub true_airspeed: geometry_msgs::msg::Vector3,

    /// Equivalent to True airspeed conversion factor.
    pub eas_2_tas: f32,

}



impl Default for Airspeed {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Airspeed::default())
  }
}

impl rosidl_runtime_rs::Message for Airspeed {
  type RmwMsg = super::msg::rmw::Airspeed;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        true_airspeed: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.true_airspeed)).into_owned(),
        eas_2_tas: msg.eas_2_tas,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        true_airspeed: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.true_airspeed)).into_owned(),
      eas_2_tas: msg.eas_2_tas,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      true_airspeed: geometry_msgs::msg::Vector3::from_rmw_message(msg.true_airspeed),
      eas_2_tas: msg.eas_2_tas,
    }
  }
}



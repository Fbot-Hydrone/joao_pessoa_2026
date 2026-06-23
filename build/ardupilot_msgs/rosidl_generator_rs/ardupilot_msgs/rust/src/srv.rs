#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to ardupilot_msgs__srv__ArmMotors_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmMotors_Request {
    /// This service requests the vehicle to arm or disarm its motors.
    /// Set true to arm motors, false to disarm motors.
    pub arm: bool,

}



impl Default for ArmMotors_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ArmMotors_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ArmMotors_Request {
  type RmwMsg = super::srv::rmw::ArmMotors_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        arm: msg.arm,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      arm: msg.arm,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      arm: msg.arm,
    }
  }
}


// Corresponds to ardupilot_msgs__srv__ArmMotors_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmMotors_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: bool,

}



impl Default for ArmMotors_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ArmMotors_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ArmMotors_Response {
  type RmwMsg = super::srv::rmw::ArmMotors_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: msg.result,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      result: msg.result,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: msg.result,
    }
  }
}


// Corresponds to ardupilot_msgs__srv__ModeSwitch_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ModeSwitch_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ModeSwitch_Request {
  type RmwMsg = super::srv::rmw::ModeSwitch_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: msg.mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      mode: msg.mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mode: msg.mode,
    }
  }
}


// Corresponds to ardupilot_msgs__srv__ModeSwitch_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ModeSwitch_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ModeSwitch_Response {
  type RmwMsg = super::srv::rmw::ModeSwitch_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        curr_mode: msg.curr_mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      curr_mode: msg.curr_mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      curr_mode: msg.curr_mode,
    }
  }
}


// Corresponds to ardupilot_msgs__srv__Takeoff_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Takeoff_Request {
    /// This service requests the vehicle to takeoff (VTOL style for QuadPlane).
    /// alt : Set the takeoff altitude above home or above terrain(in case of rangefinder)
    pub alt: f32,

}



impl Default for Takeoff_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Takeoff_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Takeoff_Request {
  type RmwMsg = super::srv::rmw::Takeoff_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        alt: msg.alt,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      alt: msg.alt,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      alt: msg.alt,
    }
  }
}


// Corresponds to ardupilot_msgs__srv__Takeoff_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Takeoff_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: bool,

}



impl Default for Takeoff_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Takeoff_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Takeoff_Response {
  type RmwMsg = super::srv::rmw::Takeoff_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
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



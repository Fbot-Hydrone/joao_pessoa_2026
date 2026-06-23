#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to hydrone_msgs__srv__SetPhase_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPhase_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetPhase_Request {
  type RmwMsg = super::srv::rmw::SetPhase_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        phase: msg.phase,
        open_hardware: msg.open_hardware,
        use_two_drones: msg.use_two_drones,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      phase: msg.phase,
      open_hardware: msg.open_hardware,
      use_two_drones: msg.use_two_drones,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      phase: msg.phase,
      open_hardware: msg.open_hardware,
      use_two_drones: msg.use_two_drones,
    }
  }
}


// Corresponds to hydrone_msgs__srv__SetPhase_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPhase_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetPhase_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPhase_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetPhase_Response {
  type RmwMsg = super::srv::rmw::SetPhase_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
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



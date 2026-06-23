#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to hydrone_msgs__msg__LandingBase
/// Represents a detected landing base in the arena

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LandingBase {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// 0-5 (6 bases total)
    pub base_id: u8,

    /// Position and orientation of the base
    pub pose: geometry_msgs::msg::Pose,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LandingBase::default())
  }
}

impl rosidl_runtime_rs::Message for LandingBase {
  type RmwMsg = super::msg::rmw::LandingBase;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        base_id: msg.base_id,
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        is_suspended: msg.is_suspended,
        is_visited: msg.is_visited,
        confidence: msg.confidence,
        height: msg.height,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      base_id: msg.base_id,
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      is_suspended: msg.is_suspended,
      is_visited: msg.is_visited,
      confidence: msg.confidence,
      height: msg.height,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      base_id: msg.base_id,
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      is_suspended: msg.is_suspended,
      is_visited: msg.is_visited,
      confidence: msg.confidence,
      height: msg.height,
    }
  }
}


// Corresponds to hydrone_msgs__msg__MissionState
/// Mission state for the state machine

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MissionState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Competition phase: 1, 2, 3 or 4
    pub phase: u8,

    /// Current state within the mission
    pub state: u8,

    /// Human-readable state name
    pub state_name: std::string::String,

    /// Accumulated score for current attempt
    pub score: f32,

    /// True if using open hardware (2x multiplier)
    pub open_hardware: bool,

}



impl Default for MissionState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MissionState::default())
  }
}

impl rosidl_runtime_rs::Message for MissionState {
  type RmwMsg = super::msg::rmw::MissionState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        phase: msg.phase,
        state: msg.state,
        state_name: msg.state_name.as_str().into(),
        score: msg.score,
        open_hardware: msg.open_hardware,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      phase: msg.phase,
      state: msg.state,
        state_name: msg.state_name.as_str().into(),
      score: msg.score,
      open_hardware: msg.open_hardware,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      phase: msg.phase,
      state: msg.state,
      state_name: msg.state_name.to_string(),
      score: msg.score,
      open_hardware: msg.open_hardware,
    }
  }
}


// Corresponds to hydrone_msgs__msg__HumanGesture
/// Human gesture/command detected by vision system (Phase 3)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HumanGesture {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// e.g. "arms_up", "point_left", "point_right", etc.
    pub gesture_name: std::string::String,

    /// Detection confidence 0.0-1.0
    pub confidence: f32,

    /// Position of the human in the world
    pub human_position: geometry_msgs::msg::Point,

    /// Flat array of skeleton keypoints (x,y,z per joint)
    pub skeleton_keypoints: Vec<f32>,

}



impl Default for HumanGesture {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HumanGesture::default())
  }
}

impl rosidl_runtime_rs::Message for HumanGesture {
  type RmwMsg = super::msg::rmw::HumanGesture;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        gesture_name: msg.gesture_name.as_str().into(),
        confidence: msg.confidence,
        human_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.human_position)).into_owned(),
        skeleton_keypoints: msg.skeleton_keypoints.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        gesture_name: msg.gesture_name.as_str().into(),
      confidence: msg.confidence,
        human_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.human_position)).into_owned(),
        skeleton_keypoints: msg.skeleton_keypoints.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      gesture_name: msg.gesture_name.to_string(),
      confidence: msg.confidence,
      human_position: geometry_msgs::msg::Point::from_rmw_message(msg.human_position),
      skeleton_keypoints: msg.skeleton_keypoints
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to hydrone_msgs__msg__QRCode
/// QR Code detection result (Phase 4)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct QRCode {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Letter identifier: A, B, C, D or E
    pub qr_id: std::string::String,

    /// Estimated pose of the QR code in the world
    pub pose: geometry_msgs::msg::Pose,

    /// True if this is the first time this QR was detected
    pub is_new: bool,

}



impl Default for QRCode {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::QRCode::default())
  }
}

impl rosidl_runtime_rs::Message for QRCode {
  type RmwMsg = super::msg::rmw::QRCode;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        qr_id: msg.qr_id.as_str().into(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        is_new: msg.is_new,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        qr_id: msg.qr_id.as_str().into(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      is_new: msg.is_new,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      qr_id: msg.qr_id.to_string(),
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      is_new: msg.is_new,
    }
  }
}



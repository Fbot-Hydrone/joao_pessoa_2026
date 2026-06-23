#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to biguasim_interfaces__msg__DVLSensorRange
/// DVLSensor message
/// Contains header, velocity, and range measurements

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DVLSensorRange {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Range measurements in meters from the 4 sonar beams
    pub range: [f32; 4],

}



impl Default for DVLSensorRange {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DVLSensorRange::default())
  }
}

impl rosidl_runtime_rs::Message for DVLSensorRange {
  type RmwMsg = super::msg::rmw::DVLSensorRange;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        range: msg.range,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        range: msg.range,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      range: msg.range,
    }
  }
}


// Corresponds to biguasim_interfaces__msg__ImagingSonar
/// ImagingSonar message
/// Contains timestamp, bins information, and image data

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ImagingSonar {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Number of azimuth bins
    pub bins_azimuth: i32,

    /// Number of range bins
    pub bins_range: i32,

    /// Raw sonar image (as received)
    pub raw_image: sensor_msgs::msg::Image,

    /// Ground-truth intensity (float, same size as raw)
    pub intensity: sensor_msgs::msg::Image,

    /// Ground-truth elevation (per-pixel angle or height)
    pub elevation: sensor_msgs::msg::Image,

    /// Ground-truth 3D points
    pub point_cloud: sensor_msgs::msg::PointCloud2,

}



impl Default for ImagingSonar {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ImagingSonar::default())
  }
}

impl rosidl_runtime_rs::Message for ImagingSonar {
  type RmwMsg = super::msg::rmw::ImagingSonar;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        bins_azimuth: msg.bins_azimuth,
        bins_range: msg.bins_range,
        raw_image: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Owned(msg.raw_image)).into_owned(),
        intensity: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Owned(msg.intensity)).into_owned(),
        elevation: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Owned(msg.elevation)).into_owned(),
        point_cloud: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Owned(msg.point_cloud)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      bins_azimuth: msg.bins_azimuth,
      bins_range: msg.bins_range,
        raw_image: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Borrowed(&msg.raw_image)).into_owned(),
        intensity: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Borrowed(&msg.intensity)).into_owned(),
        elevation: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Borrowed(&msg.elevation)).into_owned(),
        point_cloud: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Borrowed(&msg.point_cloud)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      bins_azimuth: msg.bins_azimuth,
      bins_range: msg.bins_range,
      raw_image: sensor_msgs::msg::Image::from_rmw_message(msg.raw_image),
      intensity: sensor_msgs::msg::Image::from_rmw_message(msg.intensity),
      elevation: sensor_msgs::msg::Image::from_rmw_message(msg.elevation),
      point_cloud: sensor_msgs::msg::PointCloud2::from_rmw_message(msg.point_cloud),
    }
  }
}



#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to micro_ros_msgs__msg__Graph

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Graph {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: Vec<super::msg::Node>,

}



impl Default for Graph {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Graph::default())
  }
}

impl rosidl_runtime_rs::Message for Graph {
  type RmwMsg = super::msg::rmw::Graph;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nodes: msg.nodes
          .into_iter()
          .map(|elem| super::msg::Node::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nodes: msg.nodes
          .iter()
          .map(|elem| super::msg::Node::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      nodes: msg.nodes
          .into_iter()
          .map(super::msg::Node::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to micro_ros_msgs__msg__Node

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Node {

    // This member is not documented.
    #[allow(missing_docs)]
    pub node_namespace: rosidl_runtime_rs::BoundedString<256>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub node_name: rosidl_runtime_rs::BoundedString<256>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub entities: Vec<super::msg::Entity>,

}



impl Default for Node {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Node::default())
  }
}

impl rosidl_runtime_rs::Message for Node {
  type RmwMsg = super::msg::rmw::Node;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        node_namespace: msg.node_namespace,
        node_name: msg.node_name,
        entities: msg.entities
          .into_iter()
          .map(|elem| super::msg::Entity::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        node_namespace: msg.node_namespace.clone(),
        node_name: msg.node_name.clone(),
        entities: msg.entities
          .iter()
          .map(|elem| super::msg::Entity::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      node_namespace: msg.node_namespace,
      node_name: msg.node_name,
      entities: msg.entities
          .into_iter()
          .map(super::msg::Entity::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to micro_ros_msgs__msg__Entity

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Entity {

    // This member is not documented.
    #[allow(missing_docs)]
    pub entity_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::BoundedString<256>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub types: Vec<rosidl_runtime_rs::BoundedString<256>>,

}

impl Entity {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PUBLISHER: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUBSCRIBER: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_SERVER: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_CLIENT: u8 = 3;

}


impl Default for Entity {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Entity::default())
  }
}

impl rosidl_runtime_rs::Message for Entity {
  type RmwMsg = super::msg::rmw::Entity;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        entity_type: msg.entity_type,
        name: msg.name,
        types: msg.types.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      entity_type: msg.entity_type,
        name: msg.name.clone(),
        types: msg.types.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      entity_type: msg.entity_type,
      name: msg.name,
      types: msg.types
          .into_iter()
          .collect(),
    }
  }
}



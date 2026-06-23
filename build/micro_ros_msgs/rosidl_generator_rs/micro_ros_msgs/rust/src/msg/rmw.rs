#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "micro_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__micro_ros_msgs__msg__Graph() -> *const std::ffi::c_void;
}

#[link(name = "micro_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn micro_ros_msgs__msg__Graph__init(msg: *mut Graph) -> bool;
    fn micro_ros_msgs__msg__Graph__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Graph>, size: usize) -> bool;
    fn micro_ros_msgs__msg__Graph__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Graph>);
    fn micro_ros_msgs__msg__Graph__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Graph>, out_seq: *mut rosidl_runtime_rs::Sequence<Graph>) -> bool;
}

// Corresponds to micro_ros_msgs__msg__Graph
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Graph {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Node>,

}



impl Default for Graph {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !micro_ros_msgs__msg__Graph__init(&mut msg as *mut _) {
        panic!("Call to micro_ros_msgs__msg__Graph__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Graph {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Graph__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Graph__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Graph__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Graph {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Graph where Self: Sized {
  const TYPE_NAME: &'static str = "micro_ros_msgs/msg/Graph";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__micro_ros_msgs__msg__Graph() }
  }
}


#[link(name = "micro_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__micro_ros_msgs__msg__Node() -> *const std::ffi::c_void;
}

#[link(name = "micro_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn micro_ros_msgs__msg__Node__init(msg: *mut Node) -> bool;
    fn micro_ros_msgs__msg__Node__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Node>, size: usize) -> bool;
    fn micro_ros_msgs__msg__Node__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Node>);
    fn micro_ros_msgs__msg__Node__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Node>, out_seq: *mut rosidl_runtime_rs::Sequence<Node>) -> bool;
}

// Corresponds to micro_ros_msgs__msg__Node
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    pub entities: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Entity>,

}



impl Default for Node {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !micro_ros_msgs__msg__Node__init(&mut msg as *mut _) {
        panic!("Call to micro_ros_msgs__msg__Node__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Node {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Node__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Node__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Node__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Node {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Node where Self: Sized {
  const TYPE_NAME: &'static str = "micro_ros_msgs/msg/Node";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__micro_ros_msgs__msg__Node() }
  }
}


#[link(name = "micro_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__micro_ros_msgs__msg__Entity() -> *const std::ffi::c_void;
}

#[link(name = "micro_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn micro_ros_msgs__msg__Entity__init(msg: *mut Entity) -> bool;
    fn micro_ros_msgs__msg__Entity__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Entity>, size: usize) -> bool;
    fn micro_ros_msgs__msg__Entity__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Entity>);
    fn micro_ros_msgs__msg__Entity__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Entity>, out_seq: *mut rosidl_runtime_rs::Sequence<Entity>) -> bool;
}

// Corresponds to micro_ros_msgs__msg__Entity
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    pub types: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::BoundedString<256>>,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !micro_ros_msgs__msg__Entity__init(&mut msg as *mut _) {
        panic!("Call to micro_ros_msgs__msg__Entity__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Entity {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Entity__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Entity__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { micro_ros_msgs__msg__Entity__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Entity {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Entity where Self: Sized {
  const TYPE_NAME: &'static str = "micro_ros_msgs/msg/Entity";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__micro_ros_msgs__msg__Entity() }
  }
}



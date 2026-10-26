#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "tdt_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__SendData() -> *const std::ffi::c_void;
}

#[link(name = "tdt_interface__rosidl_generator_c")]
extern "C" {
    fn tdt_interface__msg__SendData__init(msg: *mut SendData) -> bool;
    fn tdt_interface__msg__SendData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SendData>, size: usize) -> bool;
    fn tdt_interface__msg__SendData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SendData>);
    fn tdt_interface__msg__SendData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SendData>, out_seq: *mut rosidl_runtime_rs::Sequence<SendData>) -> bool;
}

// Corresponds to tdt_interface__msg__SendData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SendData {
    /// World optical yaw in degrees: Unity +Z = 0, clockwise positive; chassis-decoupled.
    pub yaw: f32,

    /// World optical pitch in degrees: downward positive, range [-60, 60].
    pub pitch: f32,

    /// Fire permission paired with these angles; actuator alignment gates actual shots.
    pub if_shoot: bool,

}



impl Default for SendData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tdt_interface__msg__SendData__init(&mut msg as *mut _) {
        panic!("Call to tdt_interface__msg__SendData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SendData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__SendData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__SendData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__SendData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SendData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SendData where Self: Sized {
  const TYPE_NAME: &'static str = "tdt_interface/msg/SendData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__SendData() }
  }
}


#[link(name = "tdt_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__AutoAimState() -> *const std::ffi::c_void;
}

#[link(name = "tdt_interface__rosidl_generator_c")]
extern "C" {
    fn tdt_interface__msg__AutoAimState__init(msg: *mut AutoAimState) -> bool;
    fn tdt_interface__msg__AutoAimState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AutoAimState>, size: usize) -> bool;
    fn tdt_interface__msg__AutoAimState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AutoAimState>);
    fn tdt_interface__msg__AutoAimState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AutoAimState>, out_seq: *mut rosidl_runtime_rs::Sequence<AutoAimState>) -> bool;
}

// Corresponds to tdt_interface__msg__AutoAimState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AutoAimState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_sequence: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::rmw::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_degrees: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch_degrees: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub health: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_health: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_weakened: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub projectile_speed: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub remaining_seconds: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub enemy_healths: [i32; 7],


    // This member is not documented.
    #[allow(missing_docs)]
    pub ammo_remaining: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ammo_capacity: i32,

}



impl Default for AutoAimState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tdt_interface__msg__AutoAimState__init(&mut msg as *mut _) {
        panic!("Call to tdt_interface__msg__AutoAimState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AutoAimState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__AutoAimState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__AutoAimState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__AutoAimState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AutoAimState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AutoAimState where Self: Sized {
  const TYPE_NAME: &'static str = "tdt_interface/msg/AutoAimState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__AutoAimState() }
  }
}


#[link(name = "tdt_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__AutoAimFrame() -> *const std::ffi::c_void;
}

#[link(name = "tdt_interface__rosidl_generator_c")]
extern "C" {
    fn tdt_interface__msg__AutoAimFrame__init(msg: *mut AutoAimFrame) -> bool;
    fn tdt_interface__msg__AutoAimFrame__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AutoAimFrame>, size: usize) -> bool;
    fn tdt_interface__msg__AutoAimFrame__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AutoAimFrame>);
    fn tdt_interface__msg__AutoAimFrame__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AutoAimFrame>, out_seq: *mut rosidl_runtime_rs::Sequence<AutoAimFrame>) -> bool;
}

// Corresponds to tdt_interface__msg__AutoAimFrame
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AutoAimFrame {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: super::super::msg::rmw::AutoAimState,


    // This member is not documented.
    #[allow(missing_docs)]
    pub camera_info: sensor_msgs::msg::rmw::CameraInfo,


    // This member is not documented.
    #[allow(missing_docs)]
    pub image: sensor_msgs::msg::rmw::Image,


    // This member is not documented.
    #[allow(missing_docs)]
    pub compressed_image: sensor_msgs::msg::rmw::CompressedImage,


    // This member is not documented.
    #[allow(missing_docs)]
    pub capture_to_publish_ms: f32,

}



impl Default for AutoAimFrame {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tdt_interface__msg__AutoAimFrame__init(&mut msg as *mut _) {
        panic!("Call to tdt_interface__msg__AutoAimFrame__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AutoAimFrame {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__AutoAimFrame__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__AutoAimFrame__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__AutoAimFrame__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AutoAimFrame {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AutoAimFrame where Self: Sized {
  const TYPE_NAME: &'static str = "tdt_interface/msg/AutoAimFrame";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__AutoAimFrame() }
  }
}


#[link(name = "tdt_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__RuneRequest() -> *const std::ffi::c_void;
}

#[link(name = "tdt_interface__rosidl_generator_c")]
extern "C" {
    fn tdt_interface__msg__RuneRequest__init(msg: *mut RuneRequest) -> bool;
    fn tdt_interface__msg__RuneRequest__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RuneRequest>, size: usize) -> bool;
    fn tdt_interface__msg__RuneRequest__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RuneRequest>);
    fn tdt_interface__msg__RuneRequest__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RuneRequest>, out_seq: *mut rosidl_runtime_rs::Sequence<RuneRequest>) -> bool;
}

// Corresponds to tdt_interface__msg__RuneRequest
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RuneRequest {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: u64,

}



impl Default for RuneRequest {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tdt_interface__msg__RuneRequest__init(&mut msg as *mut _) {
        panic!("Call to tdt_interface__msg__RuneRequest__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RuneRequest {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__RuneRequest__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__RuneRequest__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__RuneRequest__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RuneRequest {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RuneRequest where Self: Sized {
  const TYPE_NAME: &'static str = "tdt_interface/msg/RuneRequest";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__RuneRequest() }
  }
}


#[link(name = "tdt_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__RuneResult() -> *const std::ffi::c_void;
}

#[link(name = "tdt_interface__rosidl_generator_c")]
extern "C" {
    fn tdt_interface__msg__RuneResult__init(msg: *mut RuneResult) -> bool;
    fn tdt_interface__msg__RuneResult__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RuneResult>, size: usize) -> bool;
    fn tdt_interface__msg__RuneResult__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RuneResult>);
    fn tdt_interface__msg__RuneResult__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RuneResult>, out_seq: *mut rosidl_runtime_rs::Sequence<RuneResult>) -> bool;
}

// Corresponds to tdt_interface__msg__RuneResult
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RuneResult {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub activated: bool,

}



impl Default for RuneResult {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tdt_interface__msg__RuneResult__init(&mut msg as *mut _) {
        panic!("Call to tdt_interface__msg__RuneResult__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RuneResult {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__RuneResult__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__RuneResult__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tdt_interface__msg__RuneResult__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RuneResult {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RuneResult where Self: Sized {
  const TYPE_NAME: &'static str = "tdt_interface/msg/RuneResult";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tdt_interface__msg__RuneResult() }
  }
}



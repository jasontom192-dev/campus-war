#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to tdt_interface__msg__SendData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SendData::default())
  }
}

impl rosidl_runtime_rs::Message for SendData {
  type RmwMsg = super::msg::rmw::SendData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        yaw: msg.yaw,
        pitch: msg.pitch,
        if_shoot: msg.if_shoot,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      yaw: msg.yaw,
      pitch: msg.pitch,
      if_shoot: msg.if_shoot,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      yaw: msg.yaw,
      pitch: msg.pitch,
      if_shoot: msg.if_shoot,
    }
  }
}


// Corresponds to tdt_interface__msg__AutoAimState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AutoAimState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_sequence: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::Twist,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AutoAimState::default())
  }
}

impl rosidl_runtime_rs::Message for AutoAimState {
  type RmwMsg = super::msg::rmw::AutoAimState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        frame_sequence: msg.frame_sequence,
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.twist)).into_owned(),
        yaw_degrees: msg.yaw_degrees,
        pitch_degrees: msg.pitch_degrees,
        health: msg.health,
        max_health: msg.max_health,
        is_weakened: msg.is_weakened,
        projectile_speed: msg.projectile_speed,
        remaining_seconds: msg.remaining_seconds,
        enemy_healths: msg.enemy_healths,
        ammo_remaining: msg.ammo_remaining,
        ammo_capacity: msg.ammo_capacity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      frame_sequence: msg.frame_sequence,
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.twist)).into_owned(),
      yaw_degrees: msg.yaw_degrees,
      pitch_degrees: msg.pitch_degrees,
      health: msg.health,
      max_health: msg.max_health,
      is_weakened: msg.is_weakened,
      projectile_speed: msg.projectile_speed,
      remaining_seconds: msg.remaining_seconds,
        enemy_healths: msg.enemy_healths,
      ammo_remaining: msg.ammo_remaining,
      ammo_capacity: msg.ammo_capacity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      frame_sequence: msg.frame_sequence,
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      twist: geometry_msgs::msg::Twist::from_rmw_message(msg.twist),
      yaw_degrees: msg.yaw_degrees,
      pitch_degrees: msg.pitch_degrees,
      health: msg.health,
      max_health: msg.max_health,
      is_weakened: msg.is_weakened,
      projectile_speed: msg.projectile_speed,
      remaining_seconds: msg.remaining_seconds,
      enemy_healths: msg.enemy_healths,
      ammo_remaining: msg.ammo_remaining,
      ammo_capacity: msg.ammo_capacity,
    }
  }
}


// Corresponds to tdt_interface__msg__AutoAimFrame

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AutoAimFrame {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: super::msg::AutoAimState,


    // This member is not documented.
    #[allow(missing_docs)]
    pub camera_info: sensor_msgs::msg::CameraInfo,


    // This member is not documented.
    #[allow(missing_docs)]
    pub image: sensor_msgs::msg::Image,


    // This member is not documented.
    #[allow(missing_docs)]
    pub compressed_image: sensor_msgs::msg::CompressedImage,


    // This member is not documented.
    #[allow(missing_docs)]
    pub capture_to_publish_ms: f32,

}



impl Default for AutoAimFrame {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AutoAimFrame::default())
  }
}

impl rosidl_runtime_rs::Message for AutoAimFrame {
  type RmwMsg = super::msg::rmw::AutoAimFrame;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        state: super::msg::AutoAimState::into_rmw_message(std::borrow::Cow::Owned(msg.state)).into_owned(),
        camera_info: sensor_msgs::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Owned(msg.camera_info)).into_owned(),
        image: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Owned(msg.image)).into_owned(),
        compressed_image: sensor_msgs::msg::CompressedImage::into_rmw_message(std::borrow::Cow::Owned(msg.compressed_image)).into_owned(),
        capture_to_publish_ms: msg.capture_to_publish_ms,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        state: super::msg::AutoAimState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.state)).into_owned(),
        camera_info: sensor_msgs::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.camera_info)).into_owned(),
        image: sensor_msgs::msg::Image::into_rmw_message(std::borrow::Cow::Borrowed(&msg.image)).into_owned(),
        compressed_image: sensor_msgs::msg::CompressedImage::into_rmw_message(std::borrow::Cow::Borrowed(&msg.compressed_image)).into_owned(),
      capture_to_publish_ms: msg.capture_to_publish_ms,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      state: super::msg::AutoAimState::from_rmw_message(msg.state),
      camera_info: sensor_msgs::msg::CameraInfo::from_rmw_message(msg.camera_info),
      image: sensor_msgs::msg::Image::from_rmw_message(msg.image),
      compressed_image: sensor_msgs::msg::CompressedImage::from_rmw_message(msg.compressed_image),
      capture_to_publish_ms: msg.capture_to_publish_ms,
    }
  }
}


// Corresponds to tdt_interface__msg__RuneRequest

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RuneRequest {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: u64,

}



impl Default for RuneRequest {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RuneRequest::default())
  }
}

impl rosidl_runtime_rs::Message for RuneRequest {
  type RmwMsg = super::msg::rmw::RuneRequest;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        request_id: msg.request_id,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      request_id: msg.request_id,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      request_id: msg.request_id,
    }
  }
}


// Corresponds to tdt_interface__msg__RuneResult

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RuneResult {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RuneResult::default())
  }
}

impl rosidl_runtime_rs::Message for RuneResult {
  type RmwMsg = super::msg::rmw::RuneResult;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        request_id: msg.request_id,
        accepted: msg.accepted,
        activated: msg.activated,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      request_id: msg.request_id,
      accepted: msg.accepted,
      activated: msg.activated,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      request_id: msg.request_id,
      accepted: msg.accepted,
      activated: msg.activated,
    }
  }
}



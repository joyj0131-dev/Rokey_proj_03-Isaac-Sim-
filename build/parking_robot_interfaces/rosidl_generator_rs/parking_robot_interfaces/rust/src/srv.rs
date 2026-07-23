#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to parking_robot_interfaces__srv__FindEmptySlot_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindEmptySlot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_length: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_width: f64,

}



impl Default for FindEmptySlot_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::FindEmptySlot_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FindEmptySlot_Request {
  type RmwMsg = super::srv::rmw::FindEmptySlot_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        vehicle_length: msg.vehicle_length,
        vehicle_width: msg.vehicle_width,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      vehicle_length: msg.vehicle_length,
      vehicle_width: msg.vehicle_width,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      vehicle_length: msg.vehicle_length,
      vehicle_width: msg.vehicle_width,
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__FindEmptySlot_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindEmptySlot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_pose: geometry_msgs::msg::Pose,

}



impl Default for FindEmptySlot_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::FindEmptySlot_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FindEmptySlot_Response {
  type RmwMsg = super::srv::rmw::FindEmptySlot_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        slot_id: msg.slot_id.as_str().into(),
        slot_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.slot_pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        slot_id: msg.slot_id.as_str().into(),
        slot_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.slot_pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      slot_id: msg.slot_id.to_string(),
      slot_pose: geometry_msgs::msg::Pose::from_rmw_message(msg.slot_pose),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__ParkInSlot_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParkInSlot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_id: std::string::String,

}



impl Default for ParkInSlot_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ParkInSlot_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ParkInSlot_Request {
  type RmwMsg = super::srv::rmw::ParkInSlot_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        slot_id: msg.slot_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        slot_id: msg.slot_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      slot_id: msg.slot_id.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__ParkInSlot_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParkInSlot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for ParkInSlot_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ParkInSlot_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ParkInSlot_Response {
  type RmwMsg = super::srv::rmw::ParkInSlot_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        task_id: msg.task_id.as_str().into(),
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        task_id: msg.task_id.as_str().into(),
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      task_id: msg.task_id.to_string(),
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__GetSlotInfo_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSlotInfo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_id: std::string::String,

}



impl Default for GetSlotInfo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSlotInfo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetSlotInfo_Request {
  type RmwMsg = super::srv::rmw::GetSlotInfo_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        slot_id: msg.slot_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        slot_id: msg.slot_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      slot_id: msg.slot_id.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__GetSlotInfo_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSlotInfo_Response {
    /// false = /parking_slots 캐시 비어있음(runner 미기동)
    pub data_ready: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exists: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub occupied: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_accessible: bool,

    /// map 프레임, orientation = 목표 yaw(quaternion z/w)
    pub pose: geometry_msgs::msg::Pose,

}



impl Default for GetSlotInfo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSlotInfo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetSlotInfo_Response {
  type RmwMsg = super::srv::rmw::GetSlotInfo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data_ready: msg.data_ready,
        exists: msg.exists,
        occupied: msg.occupied,
        is_accessible: msg.is_accessible,
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      data_ready: msg.data_ready,
      exists: msg.exists,
      occupied: msg.occupied,
      is_accessible: msg.is_accessible,
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data_ready: msg.data_ready,
      exists: msg.exists,
      occupied: msg.occupied,
      is_accessible: msg.is_accessible,
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__RequestParkingTask_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RequestParkingTask_Request {
    /// ENTRY, EXIT
    pub request_type: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_id: std::string::String,

}



impl Default for RequestParkingTask_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RequestParkingTask_Request::default())
  }
}

impl rosidl_runtime_rs::Message for RequestParkingTask_Request {
  type RmwMsg = super::srv::rmw::RequestParkingTask_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request_type: msg.request_type.as_str().into(),
        vehicle_id: msg.vehicle_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request_type: msg.request_type.as_str().into(),
        vehicle_id: msg.vehicle_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      request_type: msg.request_type.to_string(),
      vehicle_id: msg.vehicle_id.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__RequestParkingTask_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RequestParkingTask_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for RequestParkingTask_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RequestParkingTask_Response::default())
  }
}

impl rosidl_runtime_rs::Message for RequestParkingTask_Response {
  type RmwMsg = super::srv::rmw::RequestParkingTask_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        task_id: msg.task_id.as_str().into(),
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        task_id: msg.task_id.as_str().into(),
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      task_id: msg.task_id.to_string(),
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__GetTaskStatus_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTaskStatus_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,

}



impl Default for GetTaskStatus_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetTaskStatus_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetTaskStatus_Request {
  type RmwMsg = super::srv::rmw::GetTaskStatus_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_id: msg.task_id.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__GetTaskStatus_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTaskStatus_Response {
    /// WAITING, PROCESSING, DONE, FAILED
    pub state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub eta_seconds: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetTaskStatus_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetTaskStatus_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetTaskStatus_Response {
  type RmwMsg = super::srv::rmw::GetTaskStatus_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        state: msg.state.as_str().into(),
        eta_seconds: msg.eta_seconds,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        state: msg.state.as_str().into(),
      eta_seconds: msg.eta_seconds,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      state: msg.state.to_string(),
      eta_seconds: msg.eta_seconds,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__AcquireZones_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcquireZones_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub zone_ids: Vec<std::string::String>,

}



impl Default for AcquireZones_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::AcquireZones_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AcquireZones_Request {
  type RmwMsg = super::srv::rmw::AcquireZones_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        zone_ids: msg.zone_ids
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        zone_ids: msg.zone_ids
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      robot_id: msg.robot_id.to_string(),
      task_id: msg.task_id.to_string(),
      zone_ids: msg.zone_ids
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__AcquireZones_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcquireZones_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub granted: bool,

    /// 이 로봇이 현재 보유한 존 전체
    pub held_zones: Vec<std::string::String>,

    /// 거부 시 재시도 권고 간격
    pub retry_after_sec: f32,

}



impl Default for AcquireZones_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::AcquireZones_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AcquireZones_Response {
  type RmwMsg = super::srv::rmw::AcquireZones_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        granted: msg.granted,
        held_zones: msg.held_zones
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        retry_after_sec: msg.retry_after_sec,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      granted: msg.granted,
        held_zones: msg.held_zones
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      retry_after_sec: msg.retry_after_sec,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      granted: msg.granted,
      held_zones: msg.held_zones
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      retry_after_sec: msg.retry_after_sec,
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__ReleaseZones_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseZones_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub zone_ids: Vec<std::string::String>,

}



impl Default for ReleaseZones_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ReleaseZones_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ReleaseZones_Request {
  type RmwMsg = super::srv::rmw::ReleaseZones_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        zone_ids: msg.zone_ids
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        zone_ids: msg.zone_ids
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      robot_id: msg.robot_id.to_string(),
      task_id: msg.task_id.to_string(),
      zone_ids: msg.zone_ids
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


// Corresponds to parking_robot_interfaces__srv__ReleaseZones_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseZones_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ReleaseZones_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ReleaseZones_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ReleaseZones_Response {
  type RmwMsg = super::srv::rmw::ReleaseZones_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}






#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__FindEmptySlot() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__FindEmptySlot
#[allow(missing_docs, non_camel_case_types)]
pub struct FindEmptySlot;

impl rosidl_runtime_rs::Service for FindEmptySlot {
    type Request = FindEmptySlot_Request;
    type Response = FindEmptySlot_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__FindEmptySlot() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__ParkInSlot() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__ParkInSlot
#[allow(missing_docs, non_camel_case_types)]
pub struct ParkInSlot;

impl rosidl_runtime_rs::Service for ParkInSlot {
    type Request = ParkInSlot_Request;
    type Response = ParkInSlot_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__ParkInSlot() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__GetSlotInfo() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__GetSlotInfo
#[allow(missing_docs, non_camel_case_types)]
pub struct GetSlotInfo;

impl rosidl_runtime_rs::Service for GetSlotInfo {
    type Request = GetSlotInfo_Request;
    type Response = GetSlotInfo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__GetSlotInfo() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__RequestParkingTask() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__RequestParkingTask
#[allow(missing_docs, non_camel_case_types)]
pub struct RequestParkingTask;

impl rosidl_runtime_rs::Service for RequestParkingTask {
    type Request = RequestParkingTask_Request;
    type Response = RequestParkingTask_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__RequestParkingTask() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__GetTaskStatus() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__GetTaskStatus
#[allow(missing_docs, non_camel_case_types)]
pub struct GetTaskStatus;

impl rosidl_runtime_rs::Service for GetTaskStatus {
    type Request = GetTaskStatus_Request;
    type Response = GetTaskStatus_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__GetTaskStatus() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__AcquireZones() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__AcquireZones
#[allow(missing_docs, non_camel_case_types)]
pub struct AcquireZones;

impl rosidl_runtime_rs::Service for AcquireZones {
    type Request = AcquireZones_Request;
    type Response = AcquireZones_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__AcquireZones() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__ReleaseZones() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__srv__ReleaseZones
#[allow(missing_docs, non_camel_case_types)]
pub struct ReleaseZones;

impl rosidl_runtime_rs::Service for ReleaseZones {
    type Request = ReleaseZones_Request;
    type Response = ReleaseZones_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__srv__ReleaseZones() }
    }
}



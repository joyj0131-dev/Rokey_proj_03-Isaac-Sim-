#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__FindEmptySlot_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__FindEmptySlot_Request__init(msg: *mut FindEmptySlot_Request) -> bool;
    fn parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FindEmptySlot_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FindEmptySlot_Request>);
    fn parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FindEmptySlot_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FindEmptySlot_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__FindEmptySlot_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__FindEmptySlot_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__FindEmptySlot_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FindEmptySlot_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FindEmptySlot_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FindEmptySlot_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/FindEmptySlot_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__FindEmptySlot_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__FindEmptySlot_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__FindEmptySlot_Response__init(msg: *mut FindEmptySlot_Response) -> bool;
    fn parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FindEmptySlot_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FindEmptySlot_Response>);
    fn parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FindEmptySlot_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FindEmptySlot_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__FindEmptySlot_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindEmptySlot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_pose: geometry_msgs::msg::rmw::Pose,

}



impl Default for FindEmptySlot_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__FindEmptySlot_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__FindEmptySlot_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FindEmptySlot_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FindEmptySlot_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FindEmptySlot_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/FindEmptySlot_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__FindEmptySlot_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ParkInSlot_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__ParkInSlot_Request__init(msg: *mut ParkInSlot_Request) -> bool;
    fn parking_robot_interfaces__srv__ParkInSlot_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ParkInSlot_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__ParkInSlot_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ParkInSlot_Request>);
    fn parking_robot_interfaces__srv__ParkInSlot_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ParkInSlot_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ParkInSlot_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__ParkInSlot_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParkInSlot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_id: rosidl_runtime_rs::String,

}



impl Default for ParkInSlot_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__ParkInSlot_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__ParkInSlot_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ParkInSlot_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ParkInSlot_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ParkInSlot_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ParkInSlot_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ParkInSlot_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ParkInSlot_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/ParkInSlot_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ParkInSlot_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ParkInSlot_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__ParkInSlot_Response__init(msg: *mut ParkInSlot_Response) -> bool;
    fn parking_robot_interfaces__srv__ParkInSlot_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ParkInSlot_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__ParkInSlot_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ParkInSlot_Response>);
    fn parking_robot_interfaces__srv__ParkInSlot_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ParkInSlot_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ParkInSlot_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__ParkInSlot_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParkInSlot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for ParkInSlot_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__ParkInSlot_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__ParkInSlot_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ParkInSlot_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ParkInSlot_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ParkInSlot_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ParkInSlot_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ParkInSlot_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ParkInSlot_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/ParkInSlot_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ParkInSlot_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetSlotInfo_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__GetSlotInfo_Request__init(msg: *mut GetSlotInfo_Request) -> bool;
    fn parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSlotInfo_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSlotInfo_Request>);
    fn parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSlotInfo_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSlotInfo_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__GetSlotInfo_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSlotInfo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub slot_id: rosidl_runtime_rs::String,

}



impl Default for GetSlotInfo_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__GetSlotInfo_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__GetSlotInfo_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSlotInfo_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSlotInfo_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSlotInfo_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/GetSlotInfo_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetSlotInfo_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetSlotInfo_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__GetSlotInfo_Response__init(msg: *mut GetSlotInfo_Response) -> bool;
    fn parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSlotInfo_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSlotInfo_Response>);
    fn parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSlotInfo_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSlotInfo_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__GetSlotInfo_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub pose: geometry_msgs::msg::rmw::Pose,

}



impl Default for GetSlotInfo_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__GetSlotInfo_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__GetSlotInfo_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSlotInfo_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSlotInfo_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSlotInfo_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/GetSlotInfo_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetSlotInfo_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__RequestParkingTask_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__RequestParkingTask_Request__init(msg: *mut RequestParkingTask_Request) -> bool;
    fn parking_robot_interfaces__srv__RequestParkingTask_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RequestParkingTask_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__RequestParkingTask_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RequestParkingTask_Request>);
    fn parking_robot_interfaces__srv__RequestParkingTask_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RequestParkingTask_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<RequestParkingTask_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__RequestParkingTask_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RequestParkingTask_Request {
    /// ENTRY, EXIT
    pub request_type: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_id: rosidl_runtime_rs::String,

}



impl Default for RequestParkingTask_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__RequestParkingTask_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__RequestParkingTask_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RequestParkingTask_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__RequestParkingTask_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__RequestParkingTask_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__RequestParkingTask_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RequestParkingTask_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RequestParkingTask_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/RequestParkingTask_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__RequestParkingTask_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__RequestParkingTask_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__RequestParkingTask_Response__init(msg: *mut RequestParkingTask_Response) -> bool;
    fn parking_robot_interfaces__srv__RequestParkingTask_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RequestParkingTask_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__RequestParkingTask_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RequestParkingTask_Response>);
    fn parking_robot_interfaces__srv__RequestParkingTask_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RequestParkingTask_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<RequestParkingTask_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__RequestParkingTask_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RequestParkingTask_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for RequestParkingTask_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__RequestParkingTask_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__RequestParkingTask_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RequestParkingTask_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__RequestParkingTask_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__RequestParkingTask_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__RequestParkingTask_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RequestParkingTask_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RequestParkingTask_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/RequestParkingTask_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__RequestParkingTask_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetTaskStatus_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__GetTaskStatus_Request__init(msg: *mut GetTaskStatus_Request) -> bool;
    fn parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetTaskStatus_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetTaskStatus_Request>);
    fn parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetTaskStatus_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetTaskStatus_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__GetTaskStatus_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTaskStatus_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,

}



impl Default for GetTaskStatus_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__GetTaskStatus_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__GetTaskStatus_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetTaskStatus_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetTaskStatus_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetTaskStatus_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/GetTaskStatus_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetTaskStatus_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetTaskStatus_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__GetTaskStatus_Response__init(msg: *mut GetTaskStatus_Response) -> bool;
    fn parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetTaskStatus_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetTaskStatus_Response>);
    fn parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetTaskStatus_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetTaskStatus_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__GetTaskStatus_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTaskStatus_Response {
    /// WAITING, PROCESSING, DONE, FAILED
    pub state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub eta_seconds: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetTaskStatus_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__GetTaskStatus_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__GetTaskStatus_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetTaskStatus_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetTaskStatus_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetTaskStatus_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/GetTaskStatus_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__GetTaskStatus_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__AcquireZones_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__AcquireZones_Request__init(msg: *mut AcquireZones_Request) -> bool;
    fn parking_robot_interfaces__srv__AcquireZones_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AcquireZones_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__AcquireZones_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AcquireZones_Request>);
    fn parking_robot_interfaces__srv__AcquireZones_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AcquireZones_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AcquireZones_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__AcquireZones_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcquireZones_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub zone_ids: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for AcquireZones_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__AcquireZones_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__AcquireZones_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AcquireZones_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__AcquireZones_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__AcquireZones_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__AcquireZones_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AcquireZones_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AcquireZones_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/AcquireZones_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__AcquireZones_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__AcquireZones_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__AcquireZones_Response__init(msg: *mut AcquireZones_Response) -> bool;
    fn parking_robot_interfaces__srv__AcquireZones_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AcquireZones_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__AcquireZones_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AcquireZones_Response>);
    fn parking_robot_interfaces__srv__AcquireZones_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AcquireZones_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AcquireZones_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__AcquireZones_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcquireZones_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub granted: bool,

    /// 이 로봇이 현재 보유한 존 전체
    pub held_zones: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// 거부 시 재시도 권고 간격
    pub retry_after_sec: f32,

}



impl Default for AcquireZones_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__AcquireZones_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__AcquireZones_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AcquireZones_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__AcquireZones_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__AcquireZones_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__AcquireZones_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AcquireZones_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AcquireZones_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/AcquireZones_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__AcquireZones_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ReleaseZones_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__ReleaseZones_Request__init(msg: *mut ReleaseZones_Request) -> bool;
    fn parking_robot_interfaces__srv__ReleaseZones_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReleaseZones_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__ReleaseZones_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReleaseZones_Request>);
    fn parking_robot_interfaces__srv__ReleaseZones_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReleaseZones_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ReleaseZones_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__ReleaseZones_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseZones_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub zone_ids: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for ReleaseZones_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__ReleaseZones_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__ReleaseZones_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReleaseZones_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ReleaseZones_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ReleaseZones_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ReleaseZones_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReleaseZones_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReleaseZones_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/ReleaseZones_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ReleaseZones_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ReleaseZones_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__srv__ReleaseZones_Response__init(msg: *mut ReleaseZones_Response) -> bool;
    fn parking_robot_interfaces__srv__ReleaseZones_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReleaseZones_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__srv__ReleaseZones_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReleaseZones_Response>);
    fn parking_robot_interfaces__srv__ReleaseZones_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReleaseZones_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ReleaseZones_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__srv__ReleaseZones_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseZones_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ReleaseZones_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__srv__ReleaseZones_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__srv__ReleaseZones_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReleaseZones_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ReleaseZones_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ReleaseZones_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__srv__ReleaseZones_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReleaseZones_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReleaseZones_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/srv/ReleaseZones_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__srv__ReleaseZones_Response() }
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



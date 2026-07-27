
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_Goal() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_Goal__init(msg: *mut ExecuteParkingTask_Goal) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Goal>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Goal>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Goal>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,

    /// ENTRY, EXIT
    pub request_type: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_id: rosidl_runtime_rs::String,

    /// 목표 주차면 (예: 'B1')
    pub slot_id: rosidl_runtime_rs::String,

    /// 목표 주차면 좌표 (map 프레임)
    pub slot_pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub leader_robot_id: rosidl_runtime_rs::String,

    /// 로봇 1대 MVP 흐름에서는 빈 문자열
    pub follower_robot_id: rosidl_runtime_rs::String,

}



impl Default for ExecuteParkingTask_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_Goal__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_Goal() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_Result() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_Result__init(msg: *mut ExecuteParkingTask_Result) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Result>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Result>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Result>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for ExecuteParkingTask_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_Result__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_Result where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_Result() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_Feedback__init(msg: *mut ExecuteParkingTask_Feedback) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Feedback>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Feedback>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_Feedback>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_step: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub progress: f32,

}



impl Default for ExecuteParkingTask_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_Feedback__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_Feedback() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__init(msg: *mut ExecuteParkingTask_FeedbackMessage) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_FeedbackMessage>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_FeedbackMessage>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_FeedbackMessage>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ExecuteParkingTask_Feedback,

}



impl Default for ExecuteParkingTask_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_Goal() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_Goal__init(msg: *mut DetectVehicle_Goal) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Goal>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Goal>);
    fn parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Goal>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub trigger: bool,

}



impl Default for DetectVehicle_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_Goal__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_Goal() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_Result() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_Result__init(msg: *mut DetectVehicle_Result) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Result>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Result>);
    fn parking_robot_interfaces__action__DetectVehicle_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Result>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_info: super::super::msg::rmw::VehicleInfo,

}



impl Default for DetectVehicle_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_Result__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_Result where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_Result() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_Feedback__init(msg: *mut DetectVehicle_Feedback) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Feedback>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Feedback>);
    fn parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_Feedback>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: rosidl_runtime_rs::String,

}



impl Default for DetectVehicle_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_Feedback__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_Feedback() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init(msg: *mut DetectVehicle_FeedbackMessage) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_FeedbackMessage>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_FeedbackMessage>);
    fn parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_FeedbackMessage>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::DetectVehicle_Feedback,

}



impl Default for DetectVehicle_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_FeedbackMessage() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_Goal() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_Goal__init(msg: *mut AlignVehicle_Goal) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Goal>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Goal>);
    fn parking_robot_interfaces__action__AlignVehicle_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Goal>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target_pose: geometry_msgs::msg::rmw::Pose,

}



impl Default for AlignVehicle_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_Goal__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_Goal() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_Result() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_Result__init(msg: *mut AlignVehicle_Result) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Result>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Result>);
    fn parking_robot_interfaces__action__AlignVehicle_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Result>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_error: f32,

}



impl Default for AlignVehicle_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_Result__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_Result where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_Result() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_Feedback__init(msg: *mut AlignVehicle_Feedback) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Feedback>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Feedback>);
    fn parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_Feedback>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_error: f32,

}



impl Default for AlignVehicle_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_Feedback__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_Feedback() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__init(msg: *mut AlignVehicle_FeedbackMessage) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_FeedbackMessage>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_FeedbackMessage>);
    fn parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_FeedbackMessage>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::AlignVehicle_Feedback,

}



impl Default for AlignVehicle_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_FeedbackMessage() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_Goal() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_Goal__init(msg: *mut ControlLift_Goal) -> bool;
    fn parking_robot_interfaces__action__ControlLift_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Goal>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Goal>);
    fn parking_robot_interfaces__action__ControlLift_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Goal>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_Goal {
    /// UP, DOWN
    pub command: rosidl_runtime_rs::String,

}



impl Default for ControlLift_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_Goal__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_Goal() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_Result() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_Result__init(msg: *mut ControlLift_Result) -> bool;
    fn parking_robot_interfaces__action__ControlLift_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Result>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Result>);
    fn parking_robot_interfaces__action__ControlLift_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Result>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub support_state: rosidl_runtime_rs::String,

}



impl Default for ControlLift_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_Result__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_Result where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_Result() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_Feedback__init(msg: *mut ControlLift_Feedback) -> bool;
    fn parking_robot_interfaces__action__ControlLift_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Feedback>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Feedback>);
    fn parking_robot_interfaces__action__ControlLift_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_Feedback>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: rosidl_runtime_rs::String,

}



impl Default for ControlLift_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_Feedback__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_Feedback() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_FeedbackMessage__init(msg: *mut ControlLift_FeedbackMessage) -> bool;
    fn parking_robot_interfaces__action__ControlLift_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_FeedbackMessage>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_FeedbackMessage>);
    fn parking_robot_interfaces__action__ControlLift_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_FeedbackMessage>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ControlLift_Feedback,

}



impl Default for ControlLift_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_FeedbackMessage() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_Goal() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_Goal__init(msg: *mut IngressUnderTruck_Goal) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Goal>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Goal>);
    fn parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Goal>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_Goal {
    /// 0=첫 트로프(후축), 1=둘째 트로프(전축) -- axle_detector_node 의
    ///   /robot_<id>/axle_index 와 동일한 0-based 규약(그 노드가 이
    ///   목표와 같은 세션에서 0부터 새로 세기 시작한다고 가정)
    pub trough_index: i32,

    /// m/s. 0.0(생략) => ingress_node 파라미터 기본값 사용
    pub forward_speed: f32,

    /// m/s. 0.0(생략) => ingress_node 파라미터 기본값 사용
    pub return_speed: f32,

}



impl Default for IngressUnderTruck_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_Goal__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_Goal() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_Result() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_Result__init(msg: *mut IngressUnderTruck_Result) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Result>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Result>);
    fn parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Result>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

    /// midpoint_reached | timeout | pose_stale | depth_lost | canceled
    pub stop_reason: rosidl_runtime_rs::String,

    /// 최종 정지 좌표(주행좌표계 world x)
    pub stop_x: f32,

    /// axle_detector_node 가 보고한 목표 트로프 중심 x(정렬 목표였던 값)
    pub target_axle_x: f32,

    /// 진단용: 명령 vy 의 시간적분 기반 추정치. GT 실측이 아님(이 노드는
    ///   지면진실 좌표를 모른다) -- 실제 횡편차 검증은 외부 스모크가 GT로 한다
    pub est_max_lateral_dev_m: f32,

}



impl Default for IngressUnderTruck_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_Result__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_Result where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_Result() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(msg: *mut IngressUnderTruck_Feedback) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Feedback>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Feedback>);
    fn parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_Feedback>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_Feedback {
    /// SEEK | RETURN | SETTLING
    pub phase: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub troughs_seen: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vy_cmd: f32,

}



impl Default for IngressUnderTruck_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_Feedback() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init(msg: *mut IngressUnderTruck_FeedbackMessage) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_FeedbackMessage>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_FeedbackMessage>);
    fn parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_FeedbackMessage>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::IngressUnderTruck_Feedback,

}



impl Default for IngressUnderTruck_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_Goal() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_Goal__init(msg: *mut CarryToSlot_Goal) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Goal>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Goal>);
    fn parking_robot_interfaces__action__CarryToSlot_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Goal>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub lead_robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub follow_robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_yaw_deg: f64,

}



impl Default for CarryToSlot_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_Goal__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_Goal() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_Result() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_Result__init(msg: *mut CarryToSlot_Result) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Result>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Result>);
    fn parking_robot_interfaces__action__CarryToSlot_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Result>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_yaw_deg: f64,

}



impl Default for CarryToSlot_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_Result__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_Result where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_Result() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_Feedback__init(msg: *mut CarryToSlot_Feedback) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Feedback>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Feedback>);
    fn parking_robot_interfaces__action__CarryToSlot_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_Feedback>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub phase: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dist_remaining: f64,

}



impl Default for CarryToSlot_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_Feedback__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_Feedback() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__init(msg: *mut CarryToSlot_FeedbackMessage) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_FeedbackMessage>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_FeedbackMessage>);
    fn parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_FeedbackMessage>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::CarryToSlot_Feedback,

}



impl Default for CarryToSlot_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_FeedbackMessage() }
  }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__init(msg: *mut ExecuteParkingTask_SendGoal_Request) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Request>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ExecuteParkingTask_Goal,

}



impl Default for ExecuteParkingTask_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__init(msg: *mut ExecuteParkingTask_SendGoal_Response) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Response>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_SendGoal_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ExecuteParkingTask_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__init(msg: *mut ExecuteParkingTask_GetResult_Request) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Request>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ExecuteParkingTask_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__init(msg: *mut ExecuteParkingTask_GetResult_Response) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Response>);
    fn parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteParkingTask_GetResult_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteParkingTask_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ExecuteParkingTask_Result,

}



impl Default for ExecuteParkingTask_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteParkingTask_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteParkingTask_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteParkingTask_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ExecuteParkingTask_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init(msg: *mut DetectVehicle_SendGoal_Request) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Request>);
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::DetectVehicle_Goal,

}



impl Default for DetectVehicle_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_SendGoal_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init(msg: *mut DetectVehicle_SendGoal_Response) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Response>);
    fn parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_SendGoal_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for DetectVehicle_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_SendGoal_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init(msg: *mut DetectVehicle_GetResult_Request) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Request>);
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for DetectVehicle_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_GetResult_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init(msg: *mut DetectVehicle_GetResult_Response) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Response>);
    fn parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectVehicle_GetResult_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectVehicle_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::DetectVehicle_Result,

}



impl Default for DetectVehicle_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectVehicle_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectVehicle_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectVehicle_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/DetectVehicle_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__DetectVehicle_GetResult_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__init(msg: *mut AlignVehicle_SendGoal_Request) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Request>);
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::AlignVehicle_Goal,

}



impl Default for AlignVehicle_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_SendGoal_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__init(msg: *mut AlignVehicle_SendGoal_Response) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Response>);
    fn parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_SendGoal_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for AlignVehicle_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_SendGoal_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Request__init(msg: *mut AlignVehicle_GetResult_Request) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Request>);
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for AlignVehicle_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_GetResult_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Response__init(msg: *mut AlignVehicle_GetResult_Response) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Response>);
    fn parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AlignVehicle_GetResult_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AlignVehicle_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::AlignVehicle_Result,

}



impl Default for AlignVehicle_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__AlignVehicle_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__AlignVehicle_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AlignVehicle_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AlignVehicle_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AlignVehicle_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/AlignVehicle_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__AlignVehicle_GetResult_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Request__init(msg: *mut ControlLift_SendGoal_Request) -> bool;
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Request>);
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ControlLift_Goal,

}



impl Default for ControlLift_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_SendGoal_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Response__init(msg: *mut ControlLift_SendGoal_Response) -> bool;
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Response>);
    fn parking_robot_interfaces__action__ControlLift_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_SendGoal_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ControlLift_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_SendGoal_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_GetResult_Request__init(msg: *mut ControlLift_GetResult_Request) -> bool;
    fn parking_robot_interfaces__action__ControlLift_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_GetResult_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_GetResult_Request>);
    fn parking_robot_interfaces__action__ControlLift_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_GetResult_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ControlLift_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_GetResult_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__ControlLift_GetResult_Response__init(msg: *mut ControlLift_GetResult_Response) -> bool;
    fn parking_robot_interfaces__action__ControlLift_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_GetResult_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__ControlLift_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlLift_GetResult_Response>);
    fn parking_robot_interfaces__action__ControlLift_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlLift_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlLift_GetResult_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlLift_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ControlLift_Result,

}



impl Default for ControlLift_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__ControlLift_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__ControlLift_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlLift_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__ControlLift_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlLift_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlLift_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/ControlLift_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__ControlLift_GetResult_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init(msg: *mut IngressUnderTruck_SendGoal_Request) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Request>);
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::IngressUnderTruck_Goal,

}



impl Default for IngressUnderTruck_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init(msg: *mut IngressUnderTruck_SendGoal_Response) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Response>);
    fn parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_SendGoal_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for IngressUnderTruck_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init(msg: *mut IngressUnderTruck_GetResult_Request) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Request>);
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for IngressUnderTruck_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init(msg: *mut IngressUnderTruck_GetResult_Response) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Response>);
    fn parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<IngressUnderTruck_GetResult_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IngressUnderTruck_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::IngressUnderTruck_Result,

}



impl Default for IngressUnderTruck_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IngressUnderTruck_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IngressUnderTruck_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IngressUnderTruck_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/IngressUnderTruck_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__init(msg: *mut CarryToSlot_SendGoal_Request) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Request>);
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::CarryToSlot_Goal,

}



impl Default for CarryToSlot_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_SendGoal_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__init(msg: *mut CarryToSlot_SendGoal_Response) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Response>);
    fn parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_SendGoal_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for CarryToSlot_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_SendGoal_Response() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Request__init(msg: *mut CarryToSlot_GetResult_Request) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Request>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Request>);
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Request>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for CarryToSlot_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_GetResult_Request() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Response__init(msg: *mut CarryToSlot_GetResult_Response) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Response>, size: usize) -> bool;
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Response>);
    fn parking_robot_interfaces__action__CarryToSlot_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<CarryToSlot_GetResult_Response>) -> bool;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarryToSlot_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::CarryToSlot_Result,

}



impl Default for CarryToSlot_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__action__CarryToSlot_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__action__CarryToSlot_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarryToSlot_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__action__CarryToSlot_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarryToSlot_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarryToSlot_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/action/CarryToSlot_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__action__CarryToSlot_GetResult_Response() }
  }
}






#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteParkingTask_SendGoal;

impl rosidl_runtime_rs::Service for ExecuteParkingTask_SendGoal {
    type Request = ExecuteParkingTask_SendGoal_Request;
    type Response = ExecuteParkingTask_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_SendGoal() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__ExecuteParkingTask_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteParkingTask_GetResult;

impl rosidl_runtime_rs::Service for ExecuteParkingTask_GetResult {
    type Request = ExecuteParkingTask_GetResult_Request;
    type Response = ExecuteParkingTask_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ExecuteParkingTask_GetResult() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__DetectVehicle_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct DetectVehicle_SendGoal;

impl rosidl_runtime_rs::Service for DetectVehicle_SendGoal {
    type Request = DetectVehicle_SendGoal_Request;
    type Response = DetectVehicle_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__DetectVehicle_SendGoal() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__DetectVehicle_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__DetectVehicle_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct DetectVehicle_GetResult;

impl rosidl_runtime_rs::Service for DetectVehicle_GetResult {
    type Request = DetectVehicle_GetResult_Request;
    type Response = DetectVehicle_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__DetectVehicle_GetResult() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__AlignVehicle_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct AlignVehicle_SendGoal;

impl rosidl_runtime_rs::Service for AlignVehicle_SendGoal {
    type Request = AlignVehicle_SendGoal_Request;
    type Response = AlignVehicle_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__AlignVehicle_SendGoal() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__AlignVehicle_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__AlignVehicle_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct AlignVehicle_GetResult;

impl rosidl_runtime_rs::Service for AlignVehicle_GetResult {
    type Request = AlignVehicle_GetResult_Request;
    type Response = AlignVehicle_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__AlignVehicle_GetResult() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ControlLift_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ControlLift_SendGoal;

impl rosidl_runtime_rs::Service for ControlLift_SendGoal {
    type Request = ControlLift_SendGoal_Request;
    type Response = ControlLift_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ControlLift_SendGoal() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ControlLift_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__ControlLift_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ControlLift_GetResult;

impl rosidl_runtime_rs::Service for ControlLift_GetResult {
    type Request = ControlLift_GetResult_Request;
    type Response = ControlLift_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__ControlLift_GetResult() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct IngressUnderTruck_SendGoal;

impl rosidl_runtime_rs::Service for IngressUnderTruck_SendGoal {
    type Request = IngressUnderTruck_SendGoal_Request;
    type Response = IngressUnderTruck_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_SendGoal() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__IngressUnderTruck_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct IngressUnderTruck_GetResult;

impl rosidl_runtime_rs::Service for IngressUnderTruck_GetResult {
    type Request = IngressUnderTruck_GetResult_Request;
    type Response = IngressUnderTruck_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__IngressUnderTruck_GetResult() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__CarryToSlot_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct CarryToSlot_SendGoal;

impl rosidl_runtime_rs::Service for CarryToSlot_SendGoal {
    type Request = CarryToSlot_SendGoal_Request;
    type Response = CarryToSlot_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__CarryToSlot_SendGoal() }
    }
}




#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__CarryToSlot_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to parking_robot_interfaces__action__CarryToSlot_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct CarryToSlot_GetResult;

impl rosidl_runtime_rs::Service for CarryToSlot_GetResult {
    type Request = CarryToSlot_GetResult_Request;
    type Response = CarryToSlot_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__parking_robot_interfaces__action__CarryToSlot_GetResult() }
    }
}



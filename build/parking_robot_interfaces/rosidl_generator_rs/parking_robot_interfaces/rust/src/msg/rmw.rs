#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__VehicleInfo() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__msg__VehicleInfo__init(msg: *mut VehicleInfo) -> bool;
    fn parking_robot_interfaces__msg__VehicleInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleInfo>, size: usize) -> bool;
    fn parking_robot_interfaces__msg__VehicleInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleInfo>);
    fn parking_robot_interfaces__msg__VehicleInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleInfo>) -> bool;
}

// Corresponds to parking_robot_interfaces__msg__VehicleInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// vehicle_detection_node가 인식한 차량 정보. 담당자 확정 전 임시 정의.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub length: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub width: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub height: f64,

}



impl Default for VehicleInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__msg__VehicleInfo__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__msg__VehicleInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__VehicleInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__VehicleInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__VehicleInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleInfo where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/msg/VehicleInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__VehicleInfo() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__TaskState() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__msg__TaskState__init(msg: *mut TaskState) -> bool;
    fn parking_robot_interfaces__msg__TaskState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TaskState>, size: usize) -> bool;
    fn parking_robot_interfaces__msg__TaskState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TaskState>);
    fn parking_robot_interfaces__msg__TaskState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TaskState>, out_seq: *mut rosidl_runtime_rs::Sequence<TaskState>) -> bool;
}

// Corresponds to parking_robot_interfaces__msg__TaskState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// robot_task_orchestrator가 발행하는 진행 상황. task_state 토픽에서 사용.
///
/// state 값 (2026-07-20 세분화, ENTRY/EXIT 공통):
///   SEARCHING    대상 차량을 찾는 중 (입고: 입고 예정 차량 / 출차: 해당 차량)
///   APPROACHING  차량 하부로 진입 중
///   PICKED_UP    차량 픽업 완료
///   MOVING       차량 이동 중
///   ARRIVED      목적지에 도착
///   PARKED       차량 입고 완료 (ENTRY 전용)
///   UNPARKED     차량 출차 완료 (EXIT 전용)
///   RETURNING    대기 장소 또는 충전 도크로 복귀 이동 중
///   DONE / FAILED 작업 종료
///
/// state는 문자열 필드라 새 값 추가가 다른 구독자를 깨뜨리지 않는다(모르는
/// 값은 조용히 무시됨). current_step은 화면에 그대로 보여줄 사람이 읽는 문장.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_step: rosidl_runtime_rs::String,

}



impl Default for TaskState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__msg__TaskState__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__msg__TaskState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TaskState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__TaskState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__TaskState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__TaskState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TaskState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TaskState where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/msg/TaskState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__TaskState() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__ObstacleAlert() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__msg__ObstacleAlert__init(msg: *mut ObstacleAlert) -> bool;
    fn parking_robot_interfaces__msg__ObstacleAlert__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ObstacleAlert>, size: usize) -> bool;
    fn parking_robot_interfaces__msg__ObstacleAlert__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ObstacleAlert>);
    fn parking_robot_interfaces__msg__ObstacleAlert__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ObstacleAlert>, out_seq: *mut rosidl_runtime_rs::Sequence<ObstacleAlert>) -> bool;
}

// Corresponds to parking_robot_interfaces__msg__ObstacleAlert
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// safety_monitor가 발행하는 긴급정지 신호. obstacle_alert 토픽에서 사용.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleAlert {

    // This member is not documented.
    #[allow(missing_docs)]
    pub obstacle_detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub location: geometry_msgs::msg::rmw::Point,

}



impl Default for ObstacleAlert {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__msg__ObstacleAlert__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__msg__ObstacleAlert__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ObstacleAlert {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__ObstacleAlert__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__ObstacleAlert__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__ObstacleAlert__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ObstacleAlert {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ObstacleAlert where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/msg/ObstacleAlert";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__ObstacleAlert() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__FormationStop() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__msg__FormationStop__init(msg: *mut FormationStop) -> bool;
    fn parking_robot_interfaces__msg__FormationStop__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FormationStop>, size: usize) -> bool;
    fn parking_robot_interfaces__msg__FormationStop__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FormationStop>);
    fn parking_robot_interfaces__msg__FormationStop__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FormationStop>, out_seq: *mut rosidl_runtime_rs::Sequence<FormationStop>) -> bool;
}

// Corresponds to parking_robot_interfaces__msg__FormationStop
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// [초안, Team A 2026-07-21] 로봇 2대(leader/follower) 공동 정지 방송.
/// formation_stop 토픽(공용, 네임스페이스 없음)에서 사용한다.
///
/// 배경: 차량 하나를 로봇 2대가 붙잡고 옮기는 중에는 "한쪽만 멈춤"이 가장
/// 위험하다(차가 뒤틀림). 어느 한쪽이 정지해야 하는 상황(자기 이상 감지,
/// 파트너 신호 두절 등)이 생기면 이 메시지로 즉시 상대에게 알려서 같이
/// 멈추게 한다. task_id로 어느 팀(로봇 2대 편성)의 메시지인지 구분한다 —
/// 여러 팀이 동시에 작업 중이어도 서로 신호가 안 섞이도록.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FormationStop {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,

    /// 이 신호를 보낸 로봇
    pub source_robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stop: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: rosidl_runtime_rs::String,

}



impl Default for FormationStop {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__msg__FormationStop__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__msg__FormationStop__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FormationStop {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__FormationStop__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__FormationStop__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__FormationStop__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FormationStop {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FormationStop where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/msg/FormationStop";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__FormationStop() }
  }
}


#[link(name = "parking_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__FormationAssignment() -> *const std::ffi::c_void;
}

#[link(name = "parking_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn parking_robot_interfaces__msg__FormationAssignment__init(msg: *mut FormationAssignment) -> bool;
    fn parking_robot_interfaces__msg__FormationAssignment__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FormationAssignment>, size: usize) -> bool;
    fn parking_robot_interfaces__msg__FormationAssignment__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FormationAssignment>);
    fn parking_robot_interfaces__msg__FormationAssignment__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FormationAssignment>, out_seq: *mut rosidl_runtime_rs::Sequence<FormationAssignment>) -> bool;
}

// Corresponds to parking_robot_interfaces__msg__FormationAssignment
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// [초안, Team A 2026-07-21] task_dispatcher -> 로봇별 formation_gap_controller 배정.
/// formation_assignment 토픽(공용, 네임스페이스 없음)에서 사용한다.
///
/// formation_gap_controller는 로봇마다 하나씩 항상 떠 있는 상시 노드다(실제
/// 로봇의 온보드 소프트웨어처럼). 작업이 없을 때는 idle 상태로 아무것도
/// 발행하지 않고 대기하다가, 이 메시지로 자기 robot_id에 해당하는 배정이
/// 오면 그 순간부터 active=true인 동안만 간격유지/공동정지 로직을 켠다.
///
/// 파트너 odom 토픽 이름은 "/<partner_robot_id>/odom" 컨벤션을 그대로
/// 따른다고 가정한다 — 로봇마다 자기 네임스페이스 아래 odom/cmd_vel을
/// 갖는다는 게 이 프로젝트의 다중로봇 토픽 규칙이다.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FormationAssignment {
    /// 이 배정의 대상 로봇
    pub robot_id: rosidl_runtime_rs::String,

    /// active=false일 때는 의미 없음
    pub task_id: rosidl_runtime_rs::String,

    /// leader | follower
    pub role: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub partner_robot_id: rosidl_runtime_rs::String,

    /// false면 이 로봇은 idle로 복귀
    pub active: bool,

}



impl Default for FormationAssignment {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !parking_robot_interfaces__msg__FormationAssignment__init(&mut msg as *mut _) {
        panic!("Call to parking_robot_interfaces__msg__FormationAssignment__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FormationAssignment {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__FormationAssignment__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__FormationAssignment__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { parking_robot_interfaces__msg__FormationAssignment__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FormationAssignment {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FormationAssignment where Self: Sized {
  const TYPE_NAME: &'static str = "parking_robot_interfaces/msg/FormationAssignment";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__parking_robot_interfaces__msg__FormationAssignment() }
  }
}



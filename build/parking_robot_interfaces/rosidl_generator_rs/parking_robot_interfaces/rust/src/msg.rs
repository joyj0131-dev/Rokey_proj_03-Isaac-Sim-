#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to parking_robot_interfaces__msg__VehicleInfo
/// vehicle_detection_node가 인식한 차량 정보. 담당자 확정 전 임시 정의.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleInfo::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleInfo {
  type RmwMsg = super::msg::rmw::VehicleInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        length: msg.length,
        width: msg.width,
        height: msg.height,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      length: msg.length,
      width: msg.width,
      height: msg.height,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      length: msg.length,
      width: msg.width,
      height: msg.height,
    }
  }
}


// Corresponds to parking_robot_interfaces__msg__TaskState
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_step: std::string::String,

}



impl Default for TaskState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TaskState::default())
  }
}

impl rosidl_runtime_rs::Message for TaskState {
  type RmwMsg = super::msg::rmw::TaskState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        state: msg.state.as_str().into(),
        current_step: msg.current_step.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        state: msg.state.as_str().into(),
        current_step: msg.current_step.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      robot_id: msg.robot_id.to_string(),
      task_id: msg.task_id.to_string(),
      state: msg.state.to_string(),
      current_step: msg.current_step.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__msg__ObstacleAlert
/// safety_monitor가 발행하는 긴급정지 신호. obstacle_alert 토픽에서 사용.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleAlert {

    // This member is not documented.
    #[allow(missing_docs)]
    pub obstacle_detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub location: geometry_msgs::msg::Point,

}



impl Default for ObstacleAlert {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ObstacleAlert::default())
  }
}

impl rosidl_runtime_rs::Message for ObstacleAlert {
  type RmwMsg = super::msg::rmw::ObstacleAlert;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        obstacle_detected: msg.obstacle_detected,
        description: msg.description.as_str().into(),
        location: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.location)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      obstacle_detected: msg.obstacle_detected,
        description: msg.description.as_str().into(),
        location: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.location)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      obstacle_detected: msg.obstacle_detected,
      description: msg.description.to_string(),
      location: geometry_msgs::msg::Point::from_rmw_message(msg.location),
    }
  }
}


// Corresponds to parking_robot_interfaces__msg__FormationStop
/// [초안, Team A 2026-07-21] 로봇 2대(leader/follower) 공동 정지 방송.
/// formation_stop 토픽(공용, 네임스페이스 없음)에서 사용한다.
///
/// 배경: 차량 하나를 로봇 2대가 붙잡고 옮기는 중에는 "한쪽만 멈춤"이 가장
/// 위험하다(차가 뒤틀림). 어느 한쪽이 정지해야 하는 상황(자기 이상 감지,
/// 파트너 신호 두절 등)이 생기면 이 메시지로 즉시 상대에게 알려서 같이
/// 멈추게 한다. task_id로 어느 팀(로봇 2대 편성)의 메시지인지 구분한다 —
/// 여러 팀이 동시에 작업 중이어도 서로 신호가 안 섞이도록.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FormationStop {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,

    /// 이 신호를 보낸 로봇
    pub source_robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stop: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,

}



impl Default for FormationStop {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FormationStop::default())
  }
}

impl rosidl_runtime_rs::Message for FormationStop {
  type RmwMsg = super::msg::rmw::FormationStop;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        source_robot_id: msg.source_robot_id.as_str().into(),
        stop: msg.stop,
        reason: msg.reason.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        source_robot_id: msg.source_robot_id.as_str().into(),
      stop: msg.stop,
        reason: msg.reason.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_id: msg.task_id.to_string(),
      source_robot_id: msg.source_robot_id.to_string(),
      stop: msg.stop,
      reason: msg.reason.to_string(),
    }
  }
}


// Corresponds to parking_robot_interfaces__msg__FormationAssignment
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FormationAssignment {
    /// 이 배정의 대상 로봇
    pub robot_id: std::string::String,

    /// active=false일 때는 의미 없음
    pub task_id: std::string::String,

    /// leader | follower
    pub role: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub partner_robot_id: std::string::String,

    /// false면 이 로봇은 idle로 복귀
    pub active: bool,

}



impl Default for FormationAssignment {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FormationAssignment::default())
  }
}

impl rosidl_runtime_rs::Message for FormationAssignment {
  type RmwMsg = super::msg::rmw::FormationAssignment;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        role: msg.role.as_str().into(),
        partner_robot_id: msg.partner_robot_id.as_str().into(),
        active: msg.active,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        role: msg.role.as_str().into(),
        partner_robot_id: msg.partner_robot_id.as_str().into(),
      active: msg.active,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      robot_id: msg.robot_id.to_string(),
      task_id: msg.task_id.to_string(),
      role: msg.role.to_string(),
      partner_robot_id: msg.partner_robot_id.to_string(),
      active: msg.active,
    }
  }
}



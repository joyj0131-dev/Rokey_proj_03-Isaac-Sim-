// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:action/CarryToSlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CARRY_TO_SLOT__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CARRY_TO_SLOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/action/detail/carry_to_slot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_Goal_target_yaw_deg
{
public:
  explicit Init_CarryToSlot_Goal_target_yaw_deg(::parking_robot_interfaces::action::CarryToSlot_Goal & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_Goal target_yaw_deg(::parking_robot_interfaces::action::CarryToSlot_Goal::_target_yaw_deg_type arg)
  {
    msg_.target_yaw_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Goal msg_;
};

class Init_CarryToSlot_Goal_target_z
{
public:
  explicit Init_CarryToSlot_Goal_target_z(::parking_robot_interfaces::action::CarryToSlot_Goal & msg)
  : msg_(msg)
  {}
  Init_CarryToSlot_Goal_target_yaw_deg target_z(::parking_robot_interfaces::action::CarryToSlot_Goal::_target_z_type arg)
  {
    msg_.target_z = std::move(arg);
    return Init_CarryToSlot_Goal_target_yaw_deg(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Goal msg_;
};

class Init_CarryToSlot_Goal_target_x
{
public:
  explicit Init_CarryToSlot_Goal_target_x(::parking_robot_interfaces::action::CarryToSlot_Goal & msg)
  : msg_(msg)
  {}
  Init_CarryToSlot_Goal_target_z target_x(::parking_robot_interfaces::action::CarryToSlot_Goal::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_CarryToSlot_Goal_target_z(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Goal msg_;
};

class Init_CarryToSlot_Goal_follow_robot_id
{
public:
  explicit Init_CarryToSlot_Goal_follow_robot_id(::parking_robot_interfaces::action::CarryToSlot_Goal & msg)
  : msg_(msg)
  {}
  Init_CarryToSlot_Goal_target_x follow_robot_id(::parking_robot_interfaces::action::CarryToSlot_Goal::_follow_robot_id_type arg)
  {
    msg_.follow_robot_id = std::move(arg);
    return Init_CarryToSlot_Goal_target_x(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Goal msg_;
};

class Init_CarryToSlot_Goal_lead_robot_id
{
public:
  Init_CarryToSlot_Goal_lead_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_Goal_follow_robot_id lead_robot_id(::parking_robot_interfaces::action::CarryToSlot_Goal::_lead_robot_id_type arg)
  {
    msg_.lead_robot_id = std::move(arg);
    return Init_CarryToSlot_Goal_follow_robot_id(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_Goal>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_Goal_lead_robot_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_Result_final_yaw_deg
{
public:
  explicit Init_CarryToSlot_Result_final_yaw_deg(::parking_robot_interfaces::action::CarryToSlot_Result & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_Result final_yaw_deg(::parking_robot_interfaces::action::CarryToSlot_Result::_final_yaw_deg_type arg)
  {
    msg_.final_yaw_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Result msg_;
};

class Init_CarryToSlot_Result_final_z
{
public:
  explicit Init_CarryToSlot_Result_final_z(::parking_robot_interfaces::action::CarryToSlot_Result & msg)
  : msg_(msg)
  {}
  Init_CarryToSlot_Result_final_yaw_deg final_z(::parking_robot_interfaces::action::CarryToSlot_Result::_final_z_type arg)
  {
    msg_.final_z = std::move(arg);
    return Init_CarryToSlot_Result_final_yaw_deg(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Result msg_;
};

class Init_CarryToSlot_Result_final_x
{
public:
  explicit Init_CarryToSlot_Result_final_x(::parking_robot_interfaces::action::CarryToSlot_Result & msg)
  : msg_(msg)
  {}
  Init_CarryToSlot_Result_final_z final_x(::parking_robot_interfaces::action::CarryToSlot_Result::_final_x_type arg)
  {
    msg_.final_x = std::move(arg);
    return Init_CarryToSlot_Result_final_z(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Result msg_;
};

class Init_CarryToSlot_Result_message
{
public:
  explicit Init_CarryToSlot_Result_message(::parking_robot_interfaces::action::CarryToSlot_Result & msg)
  : msg_(msg)
  {}
  Init_CarryToSlot_Result_final_x message(::parking_robot_interfaces::action::CarryToSlot_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CarryToSlot_Result_final_x(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Result msg_;
};

class Init_CarryToSlot_Result_success
{
public:
  Init_CarryToSlot_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_Result_message success(::parking_robot_interfaces::action::CarryToSlot_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CarryToSlot_Result_message(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_Result>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_Result_success();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_Feedback_dist_remaining
{
public:
  explicit Init_CarryToSlot_Feedback_dist_remaining(::parking_robot_interfaces::action::CarryToSlot_Feedback & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_Feedback dist_remaining(::parking_robot_interfaces::action::CarryToSlot_Feedback::_dist_remaining_type arg)
  {
    msg_.dist_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Feedback msg_;
};

class Init_CarryToSlot_Feedback_phase
{
public:
  Init_CarryToSlot_Feedback_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_Feedback_dist_remaining phase(::parking_robot_interfaces::action::CarryToSlot_Feedback::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_CarryToSlot_Feedback_dist_remaining(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_Feedback>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_Feedback_phase();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_SendGoal_Request_goal
{
public:
  explicit Init_CarryToSlot_SendGoal_Request_goal(::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request goal(::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request msg_;
};

class Init_CarryToSlot_SendGoal_Request_goal_id
{
public:
  Init_CarryToSlot_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_SendGoal_Request_goal goal_id(::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CarryToSlot_SendGoal_Request_goal(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_SendGoal_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_SendGoal_Response_stamp
{
public:
  explicit Init_CarryToSlot_SendGoal_Response_stamp(::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response stamp(::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response msg_;
};

class Init_CarryToSlot_SendGoal_Response_accepted
{
public:
  Init_CarryToSlot_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_SendGoal_Response_stamp accepted(::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_CarryToSlot_SendGoal_Response_stamp(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_SendGoal_Response_accepted();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_GetResult_Request_goal_id
{
public:
  Init_CarryToSlot_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_GetResult_Request goal_id(::parking_robot_interfaces::action::CarryToSlot_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_GetResult_Request>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_GetResult_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_GetResult_Response_result
{
public:
  explicit Init_CarryToSlot_GetResult_Response_result(::parking_robot_interfaces::action::CarryToSlot_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_GetResult_Response result(::parking_robot_interfaces::action::CarryToSlot_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_GetResult_Response msg_;
};

class Init_CarryToSlot_GetResult_Response_status
{
public:
  Init_CarryToSlot_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_GetResult_Response_result status(::parking_robot_interfaces::action::CarryToSlot_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_CarryToSlot_GetResult_Response_result(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_GetResult_Response>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_GetResult_Response_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_CarryToSlot_FeedbackMessage_feedback
{
public:
  explicit Init_CarryToSlot_FeedbackMessage_feedback(::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage feedback(::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage msg_;
};

class Init_CarryToSlot_FeedbackMessage_goal_id
{
public:
  Init_CarryToSlot_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarryToSlot_FeedbackMessage_feedback goal_id(::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CarryToSlot_FeedbackMessage_feedback(msg_);
  }

private:
  ::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::CarryToSlot_FeedbackMessage>()
{
  return parking_robot_interfaces::action::builder::Init_CarryToSlot_FeedbackMessage_goal_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CARRY_TO_SLOT__BUILDER_HPP_

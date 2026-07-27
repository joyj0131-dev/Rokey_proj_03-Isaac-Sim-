// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:action/IngressUnderTruck.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__INGRESS_UNDER_TRUCK__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__INGRESS_UNDER_TRUCK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/action/detail/ingress_under_truck__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_Goal_return_speed
{
public:
  explicit Init_IngressUnderTruck_Goal_return_speed(::parking_robot_interfaces::action::IngressUnderTruck_Goal & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_Goal return_speed(::parking_robot_interfaces::action::IngressUnderTruck_Goal::_return_speed_type arg)
  {
    msg_.return_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Goal msg_;
};

class Init_IngressUnderTruck_Goal_forward_speed
{
public:
  explicit Init_IngressUnderTruck_Goal_forward_speed(::parking_robot_interfaces::action::IngressUnderTruck_Goal & msg)
  : msg_(msg)
  {}
  Init_IngressUnderTruck_Goal_return_speed forward_speed(::parking_robot_interfaces::action::IngressUnderTruck_Goal::_forward_speed_type arg)
  {
    msg_.forward_speed = std::move(arg);
    return Init_IngressUnderTruck_Goal_return_speed(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Goal msg_;
};

class Init_IngressUnderTruck_Goal_trough_index
{
public:
  Init_IngressUnderTruck_Goal_trough_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_Goal_forward_speed trough_index(::parking_robot_interfaces::action::IngressUnderTruck_Goal::_trough_index_type arg)
  {
    msg_.trough_index = std::move(arg);
    return Init_IngressUnderTruck_Goal_forward_speed(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_Goal>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_Goal_trough_index();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_Result_est_max_lateral_dev_m
{
public:
  explicit Init_IngressUnderTruck_Result_est_max_lateral_dev_m(::parking_robot_interfaces::action::IngressUnderTruck_Result & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_Result est_max_lateral_dev_m(::parking_robot_interfaces::action::IngressUnderTruck_Result::_est_max_lateral_dev_m_type arg)
  {
    msg_.est_max_lateral_dev_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Result msg_;
};

class Init_IngressUnderTruck_Result_target_axle_x
{
public:
  explicit Init_IngressUnderTruck_Result_target_axle_x(::parking_robot_interfaces::action::IngressUnderTruck_Result & msg)
  : msg_(msg)
  {}
  Init_IngressUnderTruck_Result_est_max_lateral_dev_m target_axle_x(::parking_robot_interfaces::action::IngressUnderTruck_Result::_target_axle_x_type arg)
  {
    msg_.target_axle_x = std::move(arg);
    return Init_IngressUnderTruck_Result_est_max_lateral_dev_m(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Result msg_;
};

class Init_IngressUnderTruck_Result_stop_x
{
public:
  explicit Init_IngressUnderTruck_Result_stop_x(::parking_robot_interfaces::action::IngressUnderTruck_Result & msg)
  : msg_(msg)
  {}
  Init_IngressUnderTruck_Result_target_axle_x stop_x(::parking_robot_interfaces::action::IngressUnderTruck_Result::_stop_x_type arg)
  {
    msg_.stop_x = std::move(arg);
    return Init_IngressUnderTruck_Result_target_axle_x(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Result msg_;
};

class Init_IngressUnderTruck_Result_stop_reason
{
public:
  explicit Init_IngressUnderTruck_Result_stop_reason(::parking_robot_interfaces::action::IngressUnderTruck_Result & msg)
  : msg_(msg)
  {}
  Init_IngressUnderTruck_Result_stop_x stop_reason(::parking_robot_interfaces::action::IngressUnderTruck_Result::_stop_reason_type arg)
  {
    msg_.stop_reason = std::move(arg);
    return Init_IngressUnderTruck_Result_stop_x(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Result msg_;
};

class Init_IngressUnderTruck_Result_success
{
public:
  Init_IngressUnderTruck_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_Result_stop_reason success(::parking_robot_interfaces::action::IngressUnderTruck_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_IngressUnderTruck_Result_stop_reason(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_Result>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_Result_success();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_Feedback_vy_cmd
{
public:
  explicit Init_IngressUnderTruck_Feedback_vy_cmd(::parking_robot_interfaces::action::IngressUnderTruck_Feedback & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_Feedback vy_cmd(::parking_robot_interfaces::action::IngressUnderTruck_Feedback::_vy_cmd_type arg)
  {
    msg_.vy_cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Feedback msg_;
};

class Init_IngressUnderTruck_Feedback_troughs_seen
{
public:
  explicit Init_IngressUnderTruck_Feedback_troughs_seen(::parking_robot_interfaces::action::IngressUnderTruck_Feedback & msg)
  : msg_(msg)
  {}
  Init_IngressUnderTruck_Feedback_vy_cmd troughs_seen(::parking_robot_interfaces::action::IngressUnderTruck_Feedback::_troughs_seen_type arg)
  {
    msg_.troughs_seen = std::move(arg);
    return Init_IngressUnderTruck_Feedback_vy_cmd(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Feedback msg_;
};

class Init_IngressUnderTruck_Feedback_current_x
{
public:
  explicit Init_IngressUnderTruck_Feedback_current_x(::parking_robot_interfaces::action::IngressUnderTruck_Feedback & msg)
  : msg_(msg)
  {}
  Init_IngressUnderTruck_Feedback_troughs_seen current_x(::parking_robot_interfaces::action::IngressUnderTruck_Feedback::_current_x_type arg)
  {
    msg_.current_x = std::move(arg);
    return Init_IngressUnderTruck_Feedback_troughs_seen(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Feedback msg_;
};

class Init_IngressUnderTruck_Feedback_phase
{
public:
  Init_IngressUnderTruck_Feedback_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_Feedback_current_x phase(::parking_robot_interfaces::action::IngressUnderTruck_Feedback::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_IngressUnderTruck_Feedback_current_x(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_Feedback>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_Feedback_phase();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_SendGoal_Request_goal
{
public:
  explicit Init_IngressUnderTruck_SendGoal_Request_goal(::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request goal(::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request msg_;
};

class Init_IngressUnderTruck_SendGoal_Request_goal_id
{
public:
  Init_IngressUnderTruck_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_SendGoal_Request_goal goal_id(::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_IngressUnderTruck_SendGoal_Request_goal(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Request>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_SendGoal_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_SendGoal_Response_stamp
{
public:
  explicit Init_IngressUnderTruck_SendGoal_Response_stamp(::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response stamp(::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response msg_;
};

class Init_IngressUnderTruck_SendGoal_Response_accepted
{
public:
  Init_IngressUnderTruck_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_SendGoal_Response_stamp accepted(::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_IngressUnderTruck_SendGoal_Response_stamp(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_SendGoal_Response>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_SendGoal_Response_accepted();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_GetResult_Request_goal_id
{
public:
  Init_IngressUnderTruck_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Request goal_id(::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Request>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_GetResult_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_GetResult_Response_result
{
public:
  explicit Init_IngressUnderTruck_GetResult_Response_result(::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response result(::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response msg_;
};

class Init_IngressUnderTruck_GetResult_Response_status
{
public:
  Init_IngressUnderTruck_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_GetResult_Response_result status(::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_IngressUnderTruck_GetResult_Response_result(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_GetResult_Response>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_GetResult_Response_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_IngressUnderTruck_FeedbackMessage_feedback
{
public:
  explicit Init_IngressUnderTruck_FeedbackMessage_feedback(::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage feedback(::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage msg_;
};

class Init_IngressUnderTruck_FeedbackMessage_goal_id
{
public:
  Init_IngressUnderTruck_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IngressUnderTruck_FeedbackMessage_feedback goal_id(::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_IngressUnderTruck_FeedbackMessage_feedback(msg_);
  }

private:
  ::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::IngressUnderTruck_FeedbackMessage>()
{
  return parking_robot_interfaces::action::builder::Init_IngressUnderTruck_FeedbackMessage_goal_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__INGRESS_UNDER_TRUCK__BUILDER_HPP_

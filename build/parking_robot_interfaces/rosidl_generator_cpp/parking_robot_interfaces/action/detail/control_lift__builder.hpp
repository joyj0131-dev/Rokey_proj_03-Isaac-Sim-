// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:action/ControlLift.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CONTROL_LIFT__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CONTROL_LIFT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/action/detail/control_lift__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_Goal_command
{
public:
  Init_ControlLift_Goal_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::ControlLift_Goal command(::parking_robot_interfaces::action::ControlLift_Goal::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_Goal>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_Goal_command();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_Result_support_state
{
public:
  explicit Init_ControlLift_Result_support_state(::parking_robot_interfaces::action::ControlLift_Result & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ControlLift_Result support_state(::parking_robot_interfaces::action::ControlLift_Result::_support_state_type arg)
  {
    msg_.support_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_Result msg_;
};

class Init_ControlLift_Result_success
{
public:
  Init_ControlLift_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlLift_Result_support_state success(::parking_robot_interfaces::action::ControlLift_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ControlLift_Result_support_state(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_Result>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_Result_success();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_Feedback_status
{
public:
  Init_ControlLift_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::ControlLift_Feedback status(::parking_robot_interfaces::action::ControlLift_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_Feedback>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_Feedback_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_SendGoal_Request_goal
{
public:
  explicit Init_ControlLift_SendGoal_Request_goal(::parking_robot_interfaces::action::ControlLift_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ControlLift_SendGoal_Request goal(::parking_robot_interfaces::action::ControlLift_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_SendGoal_Request msg_;
};

class Init_ControlLift_SendGoal_Request_goal_id
{
public:
  Init_ControlLift_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlLift_SendGoal_Request_goal goal_id(::parking_robot_interfaces::action::ControlLift_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ControlLift_SendGoal_Request_goal(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_SendGoal_Request>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_SendGoal_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_SendGoal_Response_stamp
{
public:
  explicit Init_ControlLift_SendGoal_Response_stamp(::parking_robot_interfaces::action::ControlLift_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ControlLift_SendGoal_Response stamp(::parking_robot_interfaces::action::ControlLift_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_SendGoal_Response msg_;
};

class Init_ControlLift_SendGoal_Response_accepted
{
public:
  Init_ControlLift_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlLift_SendGoal_Response_stamp accepted(::parking_robot_interfaces::action::ControlLift_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ControlLift_SendGoal_Response_stamp(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_SendGoal_Response>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_SendGoal_Response_accepted();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_GetResult_Request_goal_id
{
public:
  Init_ControlLift_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::ControlLift_GetResult_Request goal_id(::parking_robot_interfaces::action::ControlLift_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_GetResult_Request>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_GetResult_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_GetResult_Response_result
{
public:
  explicit Init_ControlLift_GetResult_Response_result(::parking_robot_interfaces::action::ControlLift_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ControlLift_GetResult_Response result(::parking_robot_interfaces::action::ControlLift_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_GetResult_Response msg_;
};

class Init_ControlLift_GetResult_Response_status
{
public:
  Init_ControlLift_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlLift_GetResult_Response_result status(::parking_robot_interfaces::action::ControlLift_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ControlLift_GetResult_Response_result(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_GetResult_Response>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_GetResult_Response_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ControlLift_FeedbackMessage_feedback
{
public:
  explicit Init_ControlLift_FeedbackMessage_feedback(::parking_robot_interfaces::action::ControlLift_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ControlLift_FeedbackMessage feedback(::parking_robot_interfaces::action::ControlLift_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_FeedbackMessage msg_;
};

class Init_ControlLift_FeedbackMessage_goal_id
{
public:
  Init_ControlLift_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlLift_FeedbackMessage_feedback goal_id(::parking_robot_interfaces::action::ControlLift_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ControlLift_FeedbackMessage_feedback(msg_);
  }

private:
  ::parking_robot_interfaces::action::ControlLift_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ControlLift_FeedbackMessage>()
{
  return parking_robot_interfaces::action::builder::Init_ControlLift_FeedbackMessage_goal_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CONTROL_LIFT__BUILDER_HPP_

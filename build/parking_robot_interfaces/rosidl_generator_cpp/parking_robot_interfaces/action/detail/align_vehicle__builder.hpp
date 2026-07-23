// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:action/AlignVehicle.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/action/detail/align_vehicle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_Goal_target_pose
{
public:
  Init_AlignVehicle_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_Goal target_pose(::parking_robot_interfaces::action::AlignVehicle_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_Goal>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_Goal_target_pose();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_Result_final_error
{
public:
  explicit Init_AlignVehicle_Result_final_error(::parking_robot_interfaces::action::AlignVehicle_Result & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_Result final_error(::parking_robot_interfaces::action::AlignVehicle_Result::_final_error_type arg)
  {
    msg_.final_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_Result msg_;
};

class Init_AlignVehicle_Result_success
{
public:
  Init_AlignVehicle_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AlignVehicle_Result_final_error success(::parking_robot_interfaces::action::AlignVehicle_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_AlignVehicle_Result_final_error(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_Result>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_Result_success();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_Feedback_current_error
{
public:
  Init_AlignVehicle_Feedback_current_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_Feedback current_error(::parking_robot_interfaces::action::AlignVehicle_Feedback::_current_error_type arg)
  {
    msg_.current_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_Feedback>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_Feedback_current_error();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_SendGoal_Request_goal
{
public:
  explicit Init_AlignVehicle_SendGoal_Request_goal(::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request goal(::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request msg_;
};

class Init_AlignVehicle_SendGoal_Request_goal_id
{
public:
  Init_AlignVehicle_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AlignVehicle_SendGoal_Request_goal goal_id(::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_AlignVehicle_SendGoal_Request_goal(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_SendGoal_Request>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_SendGoal_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_SendGoal_Response_stamp
{
public:
  explicit Init_AlignVehicle_SendGoal_Response_stamp(::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response stamp(::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response msg_;
};

class Init_AlignVehicle_SendGoal_Response_accepted
{
public:
  Init_AlignVehicle_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AlignVehicle_SendGoal_Response_stamp accepted(::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_AlignVehicle_SendGoal_Response_stamp(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_SendGoal_Response>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_SendGoal_Response_accepted();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_GetResult_Request_goal_id
{
public:
  Init_AlignVehicle_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_GetResult_Request goal_id(::parking_robot_interfaces::action::AlignVehicle_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_GetResult_Request>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_GetResult_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_GetResult_Response_result
{
public:
  explicit Init_AlignVehicle_GetResult_Response_result(::parking_robot_interfaces::action::AlignVehicle_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_GetResult_Response result(::parking_robot_interfaces::action::AlignVehicle_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_GetResult_Response msg_;
};

class Init_AlignVehicle_GetResult_Response_status
{
public:
  Init_AlignVehicle_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AlignVehicle_GetResult_Response_result status(::parking_robot_interfaces::action::AlignVehicle_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_AlignVehicle_GetResult_Response_result(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_GetResult_Response>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_GetResult_Response_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_AlignVehicle_FeedbackMessage_feedback
{
public:
  explicit Init_AlignVehicle_FeedbackMessage_feedback(::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage feedback(::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage msg_;
};

class Init_AlignVehicle_FeedbackMessage_goal_id
{
public:
  Init_AlignVehicle_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AlignVehicle_FeedbackMessage_feedback goal_id(::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_AlignVehicle_FeedbackMessage_feedback(msg_);
  }

private:
  ::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::AlignVehicle_FeedbackMessage>()
{
  return parking_robot_interfaces::action::builder::Init_AlignVehicle_FeedbackMessage_goal_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__BUILDER_HPP_

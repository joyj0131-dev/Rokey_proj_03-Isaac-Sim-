// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:action/DetectVehicle.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__DETECT_VEHICLE__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__DETECT_VEHICLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/action/detail/detect_vehicle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_Goal_trigger
{
public:
  Init_DetectVehicle_Goal_trigger()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_Goal trigger(::parking_robot_interfaces::action::DetectVehicle_Goal::_trigger_type arg)
  {
    msg_.trigger = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_Goal>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_Goal_trigger();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_Result_vehicle_info
{
public:
  explicit Init_DetectVehicle_Result_vehicle_info(::parking_robot_interfaces::action::DetectVehicle_Result & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_Result vehicle_info(::parking_robot_interfaces::action::DetectVehicle_Result::_vehicle_info_type arg)
  {
    msg_.vehicle_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_Result msg_;
};

class Init_DetectVehicle_Result_success
{
public:
  Init_DetectVehicle_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectVehicle_Result_vehicle_info success(::parking_robot_interfaces::action::DetectVehicle_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DetectVehicle_Result_vehicle_info(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_Result>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_Result_success();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_Feedback_status
{
public:
  Init_DetectVehicle_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_Feedback status(::parking_robot_interfaces::action::DetectVehicle_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_Feedback>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_Feedback_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_SendGoal_Request_goal
{
public:
  explicit Init_DetectVehicle_SendGoal_Request_goal(::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request goal(::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request msg_;
};

class Init_DetectVehicle_SendGoal_Request_goal_id
{
public:
  Init_DetectVehicle_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectVehicle_SendGoal_Request_goal goal_id(::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_DetectVehicle_SendGoal_Request_goal(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_SendGoal_Request>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_SendGoal_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_SendGoal_Response_stamp
{
public:
  explicit Init_DetectVehicle_SendGoal_Response_stamp(::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response stamp(::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response msg_;
};

class Init_DetectVehicle_SendGoal_Response_accepted
{
public:
  Init_DetectVehicle_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectVehicle_SendGoal_Response_stamp accepted(::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_DetectVehicle_SendGoal_Response_stamp(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_SendGoal_Response>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_SendGoal_Response_accepted();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_GetResult_Request_goal_id
{
public:
  Init_DetectVehicle_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_GetResult_Request goal_id(::parking_robot_interfaces::action::DetectVehicle_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_GetResult_Request>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_GetResult_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_GetResult_Response_result
{
public:
  explicit Init_DetectVehicle_GetResult_Response_result(::parking_robot_interfaces::action::DetectVehicle_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_GetResult_Response result(::parking_robot_interfaces::action::DetectVehicle_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_GetResult_Response msg_;
};

class Init_DetectVehicle_GetResult_Response_status
{
public:
  Init_DetectVehicle_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectVehicle_GetResult_Response_result status(::parking_robot_interfaces::action::DetectVehicle_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_DetectVehicle_GetResult_Response_result(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_GetResult_Response>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_GetResult_Response_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_DetectVehicle_FeedbackMessage_feedback
{
public:
  explicit Init_DetectVehicle_FeedbackMessage_feedback(::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage feedback(::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage msg_;
};

class Init_DetectVehicle_FeedbackMessage_goal_id
{
public:
  Init_DetectVehicle_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectVehicle_FeedbackMessage_feedback goal_id(::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_DetectVehicle_FeedbackMessage_feedback(msg_);
  }

private:
  ::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::DetectVehicle_FeedbackMessage>()
{
  return parking_robot_interfaces::action::builder::Init_DetectVehicle_FeedbackMessage_goal_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__DETECT_VEHICLE__BUILDER_HPP_

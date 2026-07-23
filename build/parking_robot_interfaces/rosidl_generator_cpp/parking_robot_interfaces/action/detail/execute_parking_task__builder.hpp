// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:action/ExecuteParkingTask.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__EXECUTE_PARKING_TASK__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__EXECUTE_PARKING_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/action/detail/execute_parking_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_Goal_follower_robot_id
{
public:
  explicit Init_ExecuteParkingTask_Goal_follower_robot_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal follower_robot_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_follower_robot_id_type arg)
  {
    msg_.follower_robot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

class Init_ExecuteParkingTask_Goal_leader_robot_id
{
public:
  explicit Init_ExecuteParkingTask_Goal_leader_robot_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteParkingTask_Goal_follower_robot_id leader_robot_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_leader_robot_id_type arg)
  {
    msg_.leader_robot_id = std::move(arg);
    return Init_ExecuteParkingTask_Goal_follower_robot_id(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

class Init_ExecuteParkingTask_Goal_slot_pose
{
public:
  explicit Init_ExecuteParkingTask_Goal_slot_pose(::parking_robot_interfaces::action::ExecuteParkingTask_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteParkingTask_Goal_leader_robot_id slot_pose(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_slot_pose_type arg)
  {
    msg_.slot_pose = std::move(arg);
    return Init_ExecuteParkingTask_Goal_leader_robot_id(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

class Init_ExecuteParkingTask_Goal_slot_id
{
public:
  explicit Init_ExecuteParkingTask_Goal_slot_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteParkingTask_Goal_slot_pose slot_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_slot_id_type arg)
  {
    msg_.slot_id = std::move(arg);
    return Init_ExecuteParkingTask_Goal_slot_pose(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

class Init_ExecuteParkingTask_Goal_vehicle_id
{
public:
  explicit Init_ExecuteParkingTask_Goal_vehicle_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteParkingTask_Goal_slot_id vehicle_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_vehicle_id_type arg)
  {
    msg_.vehicle_id = std::move(arg);
    return Init_ExecuteParkingTask_Goal_slot_id(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

class Init_ExecuteParkingTask_Goal_request_type
{
public:
  explicit Init_ExecuteParkingTask_Goal_request_type(::parking_robot_interfaces::action::ExecuteParkingTask_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteParkingTask_Goal_vehicle_id request_type(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_request_type_type arg)
  {
    msg_.request_type = std::move(arg);
    return Init_ExecuteParkingTask_Goal_vehicle_id(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

class Init_ExecuteParkingTask_Goal_task_id
{
public:
  Init_ExecuteParkingTask_Goal_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_Goal_request_type task_id(::parking_robot_interfaces::action::ExecuteParkingTask_Goal::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_ExecuteParkingTask_Goal_request_type(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_Goal>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_Goal_task_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_Result_message
{
public:
  explicit Init_ExecuteParkingTask_Result_message(::parking_robot_interfaces::action::ExecuteParkingTask_Result & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_Result message(::parking_robot_interfaces::action::ExecuteParkingTask_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Result msg_;
};

class Init_ExecuteParkingTask_Result_success
{
public:
  Init_ExecuteParkingTask_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_Result_message success(::parking_robot_interfaces::action::ExecuteParkingTask_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ExecuteParkingTask_Result_message(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_Result>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_Result_success();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_Feedback_progress
{
public:
  explicit Init_ExecuteParkingTask_Feedback_progress(::parking_robot_interfaces::action::ExecuteParkingTask_Feedback & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_Feedback progress(::parking_robot_interfaces::action::ExecuteParkingTask_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Feedback msg_;
};

class Init_ExecuteParkingTask_Feedback_current_step
{
public:
  Init_ExecuteParkingTask_Feedback_current_step()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_Feedback_progress current_step(::parking_robot_interfaces::action::ExecuteParkingTask_Feedback::_current_step_type arg)
  {
    msg_.current_step = std::move(arg);
    return Init_ExecuteParkingTask_Feedback_progress(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_Feedback>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_Feedback_current_step();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_SendGoal_Request_goal
{
public:
  explicit Init_ExecuteParkingTask_SendGoal_Request_goal(::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request goal(::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request msg_;
};

class Init_ExecuteParkingTask_SendGoal_Request_goal_id
{
public:
  Init_ExecuteParkingTask_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_SendGoal_Request_goal goal_id(::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteParkingTask_SendGoal_Request_goal(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Request>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_SendGoal_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_SendGoal_Response_stamp
{
public:
  explicit Init_ExecuteParkingTask_SendGoal_Response_stamp(::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response stamp(::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response msg_;
};

class Init_ExecuteParkingTask_SendGoal_Response_accepted
{
public:
  Init_ExecuteParkingTask_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_SendGoal_Response_stamp accepted(::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ExecuteParkingTask_SendGoal_Response_stamp(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_SendGoal_Response>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_SendGoal_Response_accepted();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_GetResult_Request_goal_id
{
public:
  Init_ExecuteParkingTask_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Request goal_id(::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Request>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_GetResult_Request_goal_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_GetResult_Response_result
{
public:
  explicit Init_ExecuteParkingTask_GetResult_Response_result(::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response result(::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response msg_;
};

class Init_ExecuteParkingTask_GetResult_Response_status
{
public:
  Init_ExecuteParkingTask_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_GetResult_Response_result status(::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ExecuteParkingTask_GetResult_Response_result(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_GetResult_Response>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_GetResult_Response_status();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteParkingTask_FeedbackMessage_feedback
{
public:
  explicit Init_ExecuteParkingTask_FeedbackMessage_feedback(::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage feedback(::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage msg_;
};

class Init_ExecuteParkingTask_FeedbackMessage_goal_id
{
public:
  Init_ExecuteParkingTask_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteParkingTask_FeedbackMessage_feedback goal_id(::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteParkingTask_FeedbackMessage_feedback(msg_);
  }

private:
  ::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::action::ExecuteParkingTask_FeedbackMessage>()
{
  return parking_robot_interfaces::action::builder::Init_ExecuteParkingTask_FeedbackMessage_goal_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__EXECUTE_PARKING_TASK__BUILDER_HPP_

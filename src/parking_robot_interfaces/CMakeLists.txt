cmake_minimum_required(VERSION 3.8)
project(parking_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleInfo.msg"
  "msg/TaskState.msg"
  "msg/ObstacleAlert.msg"
  "msg/FormationStop.msg"
  "msg/FormationAssignment.msg"
  "msg/SafetyState.msg"
  "srv/FindEmptySlot.srv"
  "srv/ParkInSlot.srv"
  "srv/GetSlotInfo.srv"
  "srv/RequestParkingTask.srv"
  "srv/GetTaskStatus.srv"
  "srv/AcquireZones.srv"
  "srv/ReleaseZones.srv"
  "srv/ActivateEmergencyStop.srv"
  "srv/RequestSafetyReset.srv"
  "srv/ApproveOperation.srv"
  "action/ExecuteParkingTask.action"
  "action/DetectVehicle.action"
  "action/AlignVehicle.action"
  "action/ControlLift.action"
  "action/IngressUnderTruck.action"
  "action/CarryToSlot.action"
  DEPENDENCIES geometry_msgs
)

ament_package()

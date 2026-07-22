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
  "srv/FindEmptySlot.srv"
  "srv/ParkInSlot.srv"
  "srv/GetSlotInfo.srv"
  "srv/RequestParkingTask.srv"
  "srv/GetTaskStatus.srv"
  "srv/AcquireZones.srv"
  "srv/ReleaseZones.srv"
  "action/ExecuteParkingTask.action"
  "action/DetectVehicle.action"
  "action/AlignVehicle.action"
  "action/ControlLift.action"
  DEPENDENCIES geometry_msgs
)

ament_package()

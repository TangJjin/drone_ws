cmake_minimum_required(VERSION 3.8)
project(drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneStatus.msg"
  "msg/TaskStatus.msg"
  "msg/ReadyStatus.msg"
  "msg/BarcodeCapture.msg"
  "msg/AnimalDetection.msg"
  "msg/AnimalDetections.msg"
  "msg/IndustrialCameraParams.msg"
  "msg/K230AnimalCenter.msg"
  "msg/K230AnimalTarget.msg"
  "msg/K230AnimalTargets.msg"
  "msg/K230CaptureReady.msg"
  "msg/K230RecordResult.msg"
  "msg/K230ScanPointDone.msg"
  "msg/GridPoint.msg"
  "msg/GridGroup.msg"
  "msg/WorldPoint.msg"
  "msg/WorldGroup.msg"
  "msg/MissionSummary.msg"
  "srv/StartTask.srv"
  "srv/StartOffboard.srv"
  "srv/UploadMissionYaml.srv"
  "srv/UploadMissionSummary.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

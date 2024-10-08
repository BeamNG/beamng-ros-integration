cmake_minimum_required(VERSION 3.0.2)
project(beamng_msgs)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  DamageSensor.msg
  ElectricsSensor.msg
  DeformGroupInfo.msg
  GForceSensor.msg
  PartDamageInfo.msg
  ScenarioState.msg
  StateSensor.msg
  TimeSensor.msg
  VehicleControl.msg
  VehicleInfo.msg
  MeshSensor.msg
  MeshSensorBeam.msg
  MeshSensorNode.msg
  IdealRadarSensor.msg
  IdealRadarSensorVehicle.msg
  PowertrainSensor.msg
  PowertrainSensorDevice.msg
  RadarScan.msg
  RadarReturn.msg
  RoadsSensor.msg
  CubicPolynomial.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  ChangeSimulationState.srv
  GetCurrentVehiclesInfo.srv
  GetScenarioState.srv
  SpawnVehicle.srv
  StartScenario.srv
  TeleportVehicle.srv
  PublishRoadNetwork.srv
  AiControl.srv
  NavigateToPosition.srv
)
add_action_files(
  DIRECTORY action
  FILES
  Step.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs 
  geometry_msgs 
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

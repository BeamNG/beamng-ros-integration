cmake_minimum_required(VERSION 3.0.2)
project(beamng_msgs)
find_package(catkin REQUIRED COMPONENTS
message_generation
std_msgs
actionlib_msgs
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
)
add_action_files(
  DIRECTORY action
  FILES
  Step.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
catkin_package(
 CATKIN_DEPENDS 
 std_msgs 
 actionlib_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

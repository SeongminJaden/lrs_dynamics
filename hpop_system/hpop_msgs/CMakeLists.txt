cmake_minimum_required(VERSION 3.8)
project(hpop_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/OrbitalElements.msg"
  "msg/TLEData.msg"
  "msg/Acceleration.msg"
  "msg/SatelliteState.msg"
  "msg/SpacecraftPose.msg"
  "msg/GroundStation.msg"
  "msg/ContactWindow.msg"
  "msg/ProximityEvent.msg"
  "msg/ManeuverPlan.msg"
  "msg/PropagatorStatus.msg"
)

set(srv_files
  "srv/FetchTLE.srv"
  "srv/AddSatellite.srv"
  "srv/RemoveSatellite.srv"
  "srv/PlanManeuver.srv"
  "srv/PredictContacts.srv"
  "srv/AnalyzeProximity.srv"
  "srv/ExportEphemeris.srv"
  "srv/SetPerturbations.srv"
)

set(action_files
  "action/PropagateOrbit.action"
  "action/BatchPropagate.action"
  "action/ExecuteManeuver.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

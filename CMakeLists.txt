cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# CRITICAL: Generate interfaces FIRST, before any executables
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PrepareMission.srv"
  "srv/InitiateTakeoff.srv"
  "srv/StartMission.srv"
  "srv/ConnectAgent.srv"
  "srv/DisconnectAgent.srv"
  "msg/ConnectedAgents.msg"
  "msg/Heartbeat.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

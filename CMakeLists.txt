cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geodesy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConnectAgent.srv"
  "srv/DisconnectAgent.srv"
  "srv/StartExperiment.srv"
  "msg/ConnectedAgents.msg"
  "msg/Heartbeat.msg"
  "msg/PrepareExperimentCommand.msg"
  "msg/PrepareExperimentResponse.msg"
  "msg/InitiateTakeoffCommand.msg"
  "msg/InitiateTakeoffResponse.msg"
  "msg/StartExperimentCommand.msg"
  "msg/InitiateLandCommand.msg"
  "msg/InitiateLandResponse.msg"
  DEPENDENCIES geometry_msgs geographic_msgs
)

# Actual library target for your C++ utility headers/source
add_library(${PROJECT_NAME}_lib
    src/frame_conversions.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
    rclcpp
    geometry_msgs
    geodesy
    geographic_msgs
    tf2
    tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}_libTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs geodesy geographic_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

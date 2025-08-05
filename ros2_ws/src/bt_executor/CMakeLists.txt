cmake_minimum_required(VERSION 3.8)
project(bt_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ardupilot_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(custom_msgs REQUIRED)


include_directories(include)

# --- gimbal_helpers ---
add_library(
  gimbal_helpers
    src/gimbal/helpers.cpp
)
ament_target_dependencies(
  gimbal_helpers
    rclcpp
    sensor_msgs
    OpenCV
    cv_bridge
)

# --- gimbal_action_helpers ---
add_library(
  gimbal_action_helpers
    src/gimbal_action/helpers.cpp
)

# --- gimbal_action ---
add_library(
  gimbal_action
    # src/gimbal_action/lock_on.cpp
    src/gimbal_action/look_at.cpp
    src/gimbal_action/reset_pitch.cpp
    src/gimbal_action/reset_zoom.cpp
    src/gimbal_action/set_zoom.cpp
    src/gimbal_action/scan.cpp
    # src/gimbal_action/dynamic_scan.cpp
    # src/gimbal_action/sequence.cpp
    # src/gimbal_action/zoomin_max.cpp
    # src/gimbal_action/lock_on_while_zoomin.cpp
    # src/gimbal_action/look_at_while_zoomin.cpp
    src/gimbal/gimbal_controller.cpp
)
ament_target_dependencies(
  gimbal_action
    rclcpp
    behaviortree_cpp
    geographic_msgs
    OpenCV
    cv_bridge
    ardupilot_msgs
)
target_link_libraries(
  gimbal_action
    gimbal_action_helpers
)

# --- bt_action_helpers ---
add_library(
  bt_action_helpers
    src/bt_action/helpers.cpp
)
ament_target_dependencies(
  bt_action_helpers
    rclcpp
    rclcpp_action
    behaviortree_cpp
    sensor_msgs
    geometry_msgs
    ardupilot_msgs
    geographic_msgs
    OpenCV
    cv_bridge
)

# --- bt_action ---
add_library(
  bt_action
    src/bt_action/helpers.cpp
    src/bt_action/bt_node.cpp
    src/bt_action/arm.cpp
    src/bt_action/disarm.cpp
    src/bt_action/guided.cpp
    src/bt_action/hold.cpp
    src/bt_action/navigate.cpp
    src/bt_action/register_target.cpp
    src/bt_action/search.cpp
    src/bt_action/stop_gimbal_control.cpp
    src/bt_action/resume_gimbal_control.cpp
)
ament_target_dependencies(
  bt_action
    rclcpp
    rclcpp_action
    behaviortree_cpp
    sensor_msgs
    geometry_msgs
    ardupilot_msgs
    geographic_msgs
    OpenCV
    cv_bridge
)
target_link_libraries(
  bt_action
    bt_action_helpers
    gimbal_action
)

# --- gimbal_streamer ---
add_library(
  gimbal_streamer
    src/gimbal/gimbal_streamer.cpp
)
pkg_check_modules(FFMPEG REQUIRED
  libavformat
  libavcodec
  libavutil
  libswscale
)
ament_target_dependencies(
  gimbal_streamer
    OpenCV
    cv_bridge
    rclcpp
    sensor_msgs
)
target_include_directories(
  gimbal_streamer
  PRIVATE
    ${FFMPEG_INCLUDE_DIRS}
)
target_link_libraries(
  gimbal_streamer
    gimbal_helpers
    ${FFMPEG_LIBRARIES}
)

# --- gimbal_controller ---
add_library(
  gimbal_controller
    src/gimbal/gimbal_controller.cpp
)
ament_target_dependencies(
  gimbal_controller
    OpenCV
)
target_link_libraries(
  gimbal_controller
    gimbal_helpers
)

# --- system_component_helpers ---
add_library(
  system_component_helpers
    src/system_component/helpers.cpp
)
ament_target_dependencies(
  system_component_helpers
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    ardupilot_msgs
    geographic_msgs
    std_srvs
    std_msgs
    OpenCV
    cv_bridge
)

# --- target_registry ---
add_library(
  target_registry
    src/system_component/target_registry.cpp
)
ament_target_dependencies(
  target_registry
    OpenCV
)

# --- mission_server ---
add_executable(
  mission_server
    src/system_component/mission_server.cpp
)
ament_target_dependencies(
  mission_server
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    ardupilot_msgs
    geographic_msgs
    std_srvs
    std_msgs
    OpenCV
    cv_bridge
    custom_msgs
)
target_link_libraries(
  mission_server
    system_component_helpers
    bt_action
    target_registry
    gimbal_streamer
    gimbal_controller
)

# ----------------------------
install(
  TARGETS
    mission_server
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.15)
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
find_package(gimbal REQUIRED)

include_directories(include)

# --- bt_action_helpers ---
add_library(
  bt_action_helpers
    src/bt_action/common_helpers.cpp
)

# --- navigate_helpers ---
add_library(
  navigate_helpers
    src/bt_action/navigate_helpers.cpp
)
ament_target_dependencies(
  navigate_helpers
    sensor_msgs
    geometry_msgs
    ardupilot_msgs
    geographic_msgs
)

# --- search_helpers ---
add_library(
  search_helpers
    src/bt_action/search_helpers.cpp
)
ament_target_dependencies(
  search_helpers
    rclcpp
    sensor_msgs
    ardupilot_msgs
    OpenCV
    cv_bridge
)

# --- bt_action ---
add_library(
  bt_action
    src/bt_action/bt_node.cpp
    src/bt_action/arm.cpp
    src/bt_action/disarm.cpp
    src/bt_action/guided.cpp
    src/bt_action/hold.cpp
    src/bt_action/navigate.cpp
    src/bt_action/register_target.cpp
    src/bt_action/search.cpp
    src/bt_action/navigate_while_search.cpp
    src/bt_action/stop_gimbal_control.cpp
    src/bt_action/resume_gimbal_control.cpp
    src/bt_action/take_photo.cpp
    src/bt_action/zigzag.cpp
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
    custom_msgs
    OpenCV
    cv_bridge
)
target_link_libraries(
  bt_action
    bt_action_helpers
    navigate_helpers
    search_helpers
    gimbal::gimbal_action
    gimbal::usv_gimbal_streamer
    gimbal::usv_gimbal_controller
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
    gimbal::usv_gimbal_streamer
    gimbal::usv_gimbal_controller
)

# ----------------------------
install(
  TARGETS
    mission_server
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()

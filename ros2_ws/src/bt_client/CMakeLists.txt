cmake_minimum_required(VERSION 3.16)
project(bt_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Qt6 packages
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Quick
    Qml
    Gui
    Xml
)

# Debug print
# message(STATUS "Qt6_DIR = ${Qt6_DIR}")
# message(STATUS "Qt6 Quick include dir: ${Qt6Quick_INCLUDE_DIRS}")
# message(STATUS "Qt6 Quick libraries: ${Qt6Quick_LIBRARIES}")

qt_standard_project_setup()

# Find ROS 2 and OpenCV packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(gimbal REQUIRED)

include_directories(include)

# --- BT Visualizer ---
qt_add_executable(
    bviz
        src/main.cpp
        src/parser.cpp
        src/btnode.cpp
        src/ros_bridge.cpp
        src/ros2_topic_subscriber.cpp
        src/ros2_image_provider.cpp
        include/btnode.h
        include/parser.h
        include/ros_bridge.h
        include/ros2_topic_subscriber.hpp
        include/ros2_image_provider.hpp
)
target_include_directories(
    bviz PUBLIC
        ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(
    bviz PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Qml
        Qt6::Quick
        Qt6::Xml
        ${rclcpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        gimbal::gcs_gimbal_controller
        gimbal::gcs_gimbal_streamer
        gimbal::gimbal_image_provider
)
qt_add_resources(QML_RESOURCES qml.qrc)
target_sources(
    bviz PRIVATE
        ${QML_RESOURCES}
)

# --- Command Line Interface ---
add_executable(
    cli
    src/cli.cpp
)
ament_target_dependencies(
  cli
    rclcpp
    rclcpp_action
    behaviortree_cpp
    ament_index_cpp
    custom_msgs
    geometry_msgs
    std_srvs
)
target_link_libraries(
  cli
    readline
)

install(
    TARGETS
        bviz
        cli
    DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.15)
project(gimbal)

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
find_package(Qt6 REQUIRED COMPONENTS Core Quick Qml Gui Xml)

# Find FFmpeg for GimbalStreamer
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavdevice
    libavformat
    libavfilter
    libavcodec
    libswresample
    libswscale
    libavutil
)

qt_standard_project_setup()

# --- gimbal_helpers library ---
add_library(gimbal_helpers
    src/gimbal/helpers.cpp
)
target_include_directories(gimbal_helpers
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gimbal_helpers
    rclcpp
    sensor_msgs
    OpenCV
    cv_bridge
)
target_link_libraries(gimbal_helpers
    Qt6::Gui
)

# --- gcs_gimbal_controller library ---
# Use qt_add_library to automatically handle MOC
qt_add_library(gcs_gimbal_controller STATIC 
    src/gimbal/gcs/gimbal_controller.cpp
    include/gimbal/gcs/gimbal_controller.hpp
)
target_include_directories(gcs_gimbal_controller
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(gcs_gimbal_controller
    gimbal_helpers
    Qt6::Core
)
ament_target_dependencies(gcs_gimbal_controller
    rclcpp
    OpenCV
)

# --- gcs_gimbal_streamer library ---
# Use qt_add_library to automatically handle MOC
qt_add_library(gcs_gimbal_streamer STATIC
    src/gimbal/gcs/gimbal_streamer.cpp
    include/gimbal/gcs/gimbal_streamer.hpp
)
target_include_directories(gcs_gimbal_streamer
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
pkg_check_modules(FFMPEG REQUIRED
  libavformat
  libavcodec
  libavutil
  libswscale
)
ament_target_dependencies(
  gcs_gimbal_streamer
    OpenCV
    cv_bridge
    rclcpp
    sensor_msgs
)
target_include_directories(
  gcs_gimbal_streamer
  PRIVATE
    ${FFMPEG_INCLUDE_DIRS}
)
target_link_libraries(
  gcs_gimbal_streamer
    gimbal_helpers
    ${FFMPEG_LIBRARIES}
    Qt6::Core
    Qt6::Quick
)

# --- gimbal_image_provider ---
qt_add_library(gimbal_image_provider STATIC
    src/gimbal/gcs/gimbal_image_provider.cpp
)
target_include_directories(gimbal_image_provider
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  gimbal_image_provider
    Qt6::Core
    Qt6::Quick
)

# --- usv_gimbal_controller library ---
add_library(usv_gimbal_controller
    src/gimbal/usv/gimbal_controller.cpp
)
target_include_directories(usv_gimbal_controller
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(usv_gimbal_controller
    gimbal_helpers
)
ament_target_dependencies(usv_gimbal_controller
    rclcpp
    OpenCV
)

# --- gimbal_action library --- (Used by mission server to enqueue gimbal actions)
add_library(gimbal_action
    src/gimbal_action/lock_on.cpp
    src/gimbal_action/look_at.cpp
    src/gimbal_action/reset_pitch.cpp
    src/gimbal_action/reset_zoom.cpp
    src/gimbal_action/scan.cpp
    src/gimbal_action/set_zoom.cpp
)
target_include_directories(gimbal_action
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(gimbal_action
    gimbal_helpers
    usv_gimbal_controller
)

# --- Install all libraries ---
install(
  TARGETS 
    gimbal_helpers
    gcs_gimbal_controller
    gcs_gimbal_streamer
    gimbal_image_provider
    usv_gimbal_controller
    gimbal_action
  EXPORT export_gimbal
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# --- Install headers ---
install(
  DIRECTORY include/
  DESTINATION include
)

# --- Export targets and dependencies ---
ament_export_targets(export_gimbal HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    OpenCV
    cv_bridge
    Qt6
)

# --- Generate and install config files ---
include(CMakePackageConfigHelpers)

# Generate the config file
configure_package_config_file(
    cmake/gimbalConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/gimbalConfig.cmake
    INSTALL_DESTINATION lib/cmake/gimbal
)

# Generate version file (optional but recommended)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/gimbalConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

# Install the config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/gimbalConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/gimbalConfigVersion.cmake
    DESTINATION lib/cmake/gimbal
)

# Install the targets file (this is what was missing!)
install(EXPORT export_gimbal
    FILE gimbalTargets.cmake
    NAMESPACE gimbal::
    DESTINATION lib/cmake/gimbal
)

ament_package()

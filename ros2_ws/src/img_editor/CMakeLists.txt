cmake_minimum_required(VERSION 3.8)
project(img_editor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(
  crop
    src/crop.cpp
)

ament_target_dependencies(
  crop
    OpenCV
)

install(
  TARGETS
    crop
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()

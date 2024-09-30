cmake_minimum_required(VERSION 3.8)
project(uv-vision)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(serial_util REQUIRED)

file(GLOB FUNCTIONS_SOURCE src/*.cpp)

# BUILD EXE
add_executable(
  ${PROJECT_NAME}
  ${FUNCTIONS_SOURCE}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs serial_util OpenCV)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



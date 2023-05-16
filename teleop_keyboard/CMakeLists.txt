cmake_minimum_required(VERSION 3.5)
project(teleop_keyboard)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  #${EXTERNAL_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(teleop_keyboard_node src/main.cpp src/teleop_keyboard.cpp)
ament_target_dependencies(teleop_keyboard_node rclcpp geometry_msgs std_msgs)

install(TARGETS
teleop_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

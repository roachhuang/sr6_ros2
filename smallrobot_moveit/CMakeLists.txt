cmake_minimum_required(VERSION 3.22)
project(smallrobot_moveit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

add_executable(move_program src/move_program.cpp)

target_include_directories(move_program PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(move_program
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS move_program DESTINATION lib/${PROJECT_NAME})

ament_package()

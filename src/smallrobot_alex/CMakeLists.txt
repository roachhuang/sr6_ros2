cmake_minimum_required(VERSION 3.8)
project(smallrobot_alex)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanMovement.action"
  DEPENDENCIES action_msgs
)

ament_package()

install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/planner_action_server.py
  DESTINATION lib/${PROJECT_NAME}
#   PERMISSIONS EXECUTABLE
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install action files
install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)


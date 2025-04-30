cmake_minimum_required(VERSION 3.8)
project(smallrobot_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_fndm_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# set dependencies
set(dependencies
rclcpp
rclcpp_action
rclcpp_components
ros2_fndm_interface
moveit_ros_planning_interface
)

#include directories
include_directories(include/smallrobot_remote)

# add executables for scripts
add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server ${dependencies})

# target_include_directories(task_server
#   PRIVATE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# target_compile_definitions(task_server
#   PRIVATE "TASK_SERVER_CPP_BUILDING_DLL"
# )

# ament_target_dependencies(task_server
#   "ros2_fndm_interface"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "moveit_ros_planning_interface")

# Register the component
# rclcpp_components_register_nodes(task_server
#   PLUGIN "smallrobot_remote::TaskServer"
#   EXECUTABLE "task_server_node"
# )

# Install the library
install(TARGETS task_server
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(task_server_node src/task_server.cpp)
# Install other files
# install(DIRECTORY include/ DESTINATION include)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)

# export dependencies
ament_export_dependencies(${dependencies})

ament_package()

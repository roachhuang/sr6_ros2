cmake_minimum_required(VERSION 3.8)
project(smallrobot_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
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
include_directories(include/task_server)

# add executables for scripts
add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server ${dependencies})

# install the scripts
install(TARGETS
  task_server
  # fibonacci_client
  DESTINATION lib/${PROJECT_NAME})

#############
# add_library(task_server SHARED src/task_server.cpp)
# target_include_directories(task_server
#   PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_compile_definitions(task_server
#   PRIVATE
#   "TASK_SERVER_CPP_BUILDING_DLL"
# )

# ament_target_dependencies(task_server
#   "ros2_fndm_interface"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "moveit_ros_planning_interface")
# rclcpp_components_register_node(task_server
#   PLUGIN "smallrobot_remote::TaskServer"
#   EXECUTABLE task_server_node
# )

# install(
#   TARGETS task_server
#   EXPORT export_task_server
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION bin/${PROJECT_NAME}
# )
###############

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# export dependencies
ament_export_dependencies(${dependencies})
ament_package()

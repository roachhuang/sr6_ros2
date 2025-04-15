cmake_minimum_required(VERSION 3.8)
project(smallrobot_description)

# Enable compiler warnings (optional, not strictly needed for description packages)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)  # Add xacro dependency
find_package(urdf REQUIRED)   # Add urdf dependency (optional)

# Install directories
install(
  DIRECTORY meshes urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Optional: Generate URDF from Xacro
#add_custom_target(generate_urdf ALL
#  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/a.urdf.xacro -o ${CMAKE_CURRENT_BINARY_DIR}/a.urdf
#  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/a.urdf.xacro
#)

#install(
#  FILES ${CMAKE_CURRENT_BINARY_DIR}/urdf/smallrobot.urdf.xacro
#  DESTINATION share/${PROJECT_NAME}/urdf
#)

# Enable testing
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  set(ament_cmake_copyright_FOUND TRUE)
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

# Mark the package as an ament package
ament_package()

cmake_minimum_required(VERSION 3.12)
project(base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find chassis library (built from components/control/base)
find_path(CHASSIS_INCLUDE_DIR chassis.h PATH_SUFFIXES include)
find_library(CHASSIS_LIB chassis)
if(NOT CHASSIS_INCLUDE_DIR OR NOT CHASSIS_LIB)
  message(FATAL_ERROR "chassis.h or libchassis not found. Build/install components/control/base first (PREFIX must be in CMAKE_PREFIX_PATH).")
endif()

# Create executable
add_executable(esos_base_control_node src/esos_base_control_node.cpp)
target_include_directories(esos_base_control_node PRIVATE ${CHASSIS_INCLUDE_DIR})
ament_target_dependencies(esos_base_control_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# chassis is a shared lib with driver registration
target_link_libraries(esos_base_control_node
  ${CHASSIS_LIB}
  pthread
)

install(TARGETS esos_base_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

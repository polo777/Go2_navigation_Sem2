cmake_minimum_required(VERSION 3.8)
project(go2_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executables & Link libraries
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster
  rclcpp
  nav_msgs
  tf2
  tf2_ros
)

add_executable(transform_remapper src/transform_remapper.cpp)
ament_target_dependencies(transform_remapper
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(footprint_transform_publisher src/footprint_transform_publisher.cpp)
ament_target_dependencies(footprint_transform_publisher
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(tf_tree_publisher_node src/tf_tree_publisher_node.cpp)
ament_target_dependencies(tf_tree_publisher_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

# Install executables
install(TARGETS
  odom_tf_broadcaster
  transform_remapper
  footprint_transform_publisher
  tf_tree_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
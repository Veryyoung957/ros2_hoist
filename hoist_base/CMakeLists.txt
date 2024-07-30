cmake_minimum_required(VERSION 3.8)
project(hoist_base)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hoist_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(hoist_base src/hoist_base.cpp)
ament_target_dependencies(hoist_base rclcpp std_msgs geometry_msgs sensor_msgs std_srvs tf2_ros serial nav_msgs tf2_geometry_msgs hoist_msgs)
install(TARGETS
  hoist_base
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY
    launch
    config
    world
    urdf
    map
  DESTINATION share/${PROJECT_NAME}/
)
install(
  PROGRAMS
  launch/gazebo_hoist.launch.py
  launch/hoist_base.launch.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(serial REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

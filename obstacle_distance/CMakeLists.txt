cmake_minimum_required(VERSION 3.5)
project(obstacle_distance)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(obstacle_distance_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(rosidl_default_generators REQUIRED)


include_directories(
  include
)


add_executable(obstacle_distance_node src/obstacle_distance_node.cpp src/obstacle_distance.cpp)
ament_target_dependencies(obstacle_distance_node rclcpp nav_msgs obstacle_distance_msgs visualization_msgs OpenCV)

install(TARGETS
  obstacle_distance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/launch
)
#install  a python script
install(PROGRAMS
  scripts/obstacle_distance_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(ros2_sinus_talker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(apex_cmake REQUIRED)  # Required on all Apex.OS packages!
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sinus_talker src/ros2_sinus_talker.cpp)
apex_set_compile_options(sinus_talker)
ament_target_dependencies(sinus_talker rclcpp std_msgs)

add_executable(sinus_listener src/ros2_sinus_listener.cpp)
apex_set_compile_options(sinus_listener)
ament_target_dependencies(sinus_listener rclcpp std_msgs)

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

install(TARGETS
  sinus_talker
  sinus_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

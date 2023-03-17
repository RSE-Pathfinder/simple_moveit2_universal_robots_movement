cmake_minimum_required(VERSION 3.8)
project(hello_moveit_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_moveit_ur src/hello_moveit_ur.cpp)
add_executable(counter_withdraw src/x0y1_up.cpp)
add_executable(counter_return src/x0y1_down.cpp)
add_executable(shelf_top_withdraw src/x1y0_up.cpp)
add_executable(shelf_top_return src/x1y0_down.cpp)
add_executable(shelf_bottom_withdraw src/x2y0_up.cpp)
add_executable(shelf_bottom_return src/x2y0_down.cpp)

target_include_directories(hello_moveit_ur PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit_ur PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit_ur
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
hello_moveit_ur
DESTINATION lib/${PROJECT_NAME})

target_compile_features(counter_withdraw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  counter_withdraw
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
counter_withdraw
DESTINATION lib/${PROJECT_NAME})

target_compile_features(counter_return PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  counter_return
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
counter_return
DESTINATION lib/${PROJECT_NAME})

target_compile_features(shelf_top_withdraw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shelf_top_withdraw
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
shelf_top_withdraw
DESTINATION lib/${PROJECT_NAME})

target_compile_features(shelf_top_return PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shelf_top_return
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
shelf_top_return
DESTINATION lib/${PROJECT_NAME})

target_compile_features(shelf_bottom_withdraw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shelf_bottom_withdraw
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
shelf_bottom_withdraw
DESTINATION lib/${PROJECT_NAME})

target_compile_features(shelf_bottom_return PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shelf_bottom_return
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS 
shelf_bottom_return
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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

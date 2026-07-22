cmake_minimum_required(VERSION 3.8)
project(drone_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets SerialPort)

add_executable(ground_station
  src/main.cpp
  src/mainwindow.cpp
  src/ros_manager.cpp
  src/image_preview_dialog.cpp
  src/position_view_widget.cpp
  include/drone_qt/mainwindow.hpp
  include/drone_qt/ros_manager.hpp
  include/drone_qt/image_preview_dialog.hpp
  include/drone_qt/position_view_widget.hpp
)

add_executable(ground_link_bridge
  src/ground_link_bridge_main.cpp
  src/ground_link_bridge.cpp
  include/drone_qt/ground_link_bridge.hpp
)

target_include_directories(ground_station PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ground_link_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ground_station
  Qt5::Widgets
  Qt5::SerialPort
)

target_link_libraries(ground_link_bridge
  Qt5::Core
  Qt5::SerialPort
)

ament_target_dependencies(ground_station
  rclcpp
  std_msgs
  std_srvs
  drone_msgs
  mavros_msgs
  geometry_msgs
)

ament_target_dependencies(ground_link_bridge
  rclcpp
  std_msgs
  std_srvs
  drone_msgs
  geometry_msgs
)

install(TARGETS
  ground_station
  ground_link_bridge
  DESTINATION lib/${PROJECT_NAME}
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

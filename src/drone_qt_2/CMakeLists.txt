cmake_minimum_required(VERSION 3.8)
project(drone_qt_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core SerialPort)

add_executable(airborne_node
  src/main.cpp
  src/airborne_node.cpp
  src/mission_yaml_builder.cpp
  include/drone_qt_2/airborne_node.hpp
  include/drone_qt_2/mission_yaml_builder.hpp
)

add_executable(airborne_link_bridge
  src/airborne_link_bridge_main.cpp
  src/airborne_link_bridge.cpp
  include/drone_qt_2/airborne_link_bridge.hpp
)

add_executable(car_link_bridge
  src/car_link_bridge_main.cpp
  src/car_link_bridge.cpp
  include/drone_qt_2/car_link_bridge.hpp
)

add_executable(barcode_test_sender
  src/barcode_text_sender.cpp
)

target_include_directories(airborne_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(airborne_link_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(car_link_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(airborne_node
  Qt5::Core
)

target_link_libraries(airborne_link_bridge
  Qt5::Core
  Qt5::SerialPort
)

target_link_libraries(car_link_bridge
  Qt5::Core
  Qt5::SerialPort
)

ament_target_dependencies(airborne_node
  rclcpp
  std_msgs
  std_srvs
  drone_msgs
  mavros_msgs
  geometry_msgs
)

ament_target_dependencies(airborne_link_bridge
  rclcpp
  std_msgs
  std_srvs
  drone_msgs
  geometry_msgs
)

ament_target_dependencies(car_link_bridge
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(barcode_test_sender
  rclcpp
  drone_msgs
)

install(TARGETS
  airborne_node
  airborne_link_bridge
  car_link_bridge
  barcode_test_sender
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

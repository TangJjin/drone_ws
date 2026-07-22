cmake_minimum_required(VERSION 3.8)
project(drone_warehouse)

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
find_package(PkgConfig REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets SerialPort)
pkg_check_modules(GPIOD REQUIRED libgpiod)

add_executable(warehouse_gcs
  src/main.cpp
  src/mainwindow.cpp
  src/scene_view.cpp
  src/ros_manager.cpp
  src/gpio_output.cpp
  src/top_status_bar.cpp
  src/ai_diff_analyzer.cpp
  src/shelf_info_dialog.cpp
  src/shelf_panel_storage.cpp
  include/drone_warehouse/models.hpp
  include/drone_warehouse/mainwindow.hpp
  include/drone_warehouse/scene_view.hpp
  include/drone_warehouse/ros_manager.hpp
  include/drone_warehouse/gpio_output.hpp
  include/drone_warehouse/top_status_bar.hpp
  include/drone_warehouse/ai_diff_analyzer.hpp
  include/drone_warehouse/shelf_info_dialog.hpp
  include/drone_warehouse/shelf_panel_storage.hpp
)

add_executable(ground_link_bridge
  src/ground_link_bridge_main.cpp  
  src/ground_link_bridge.cpp
  include/drone_warehouse/ground_link_bridge.hpp
)

target_include_directories(warehouse_gcs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)

target_include_directories(ground_link_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(warehouse_gcs
  Qt5::Widgets
  Qt5::SerialPort
  ${GPIOD_LIBRARIES}
)

target_link_libraries(ground_link_bridge
  Qt5::Core
  Qt5::SerialPort
)

ament_target_dependencies(warehouse_gcs
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
  warehouse_gcs
  ground_link_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/warehouse_ai_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

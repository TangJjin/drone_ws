cmake_minimum_required(VERSION 3.8)
project(drone_line_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

add_executable(line_vision_node
  src/main.cpp
  src/line_vision_node.cpp
  src/line_detector.cpp
  src/frame_statistics.cpp
)
target_include_directories(line_vision_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_vision_node
  ${OpenCV_LIBS}
  yaml-cpp
  PkgConfig::GSTREAMER
)
ament_target_dependencies(line_vision_node rclcpp std_msgs drone_msgs ament_index_cpp)

install(TARGETS line_vision_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()

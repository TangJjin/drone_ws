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
find_package(hbm_img_msgs QUIET)
find_path(HBM_IMG_MSGS_INCLUDE_DIR NAMES hbm_img_msgs/msg/hbm_msg1080p.hpp
  PATHS /opt/tros/humble/include /opt/ros/humble/include)

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
)
ament_target_dependencies(line_vision_node rclcpp std_msgs drone_msgs ament_index_cpp)
if(hbm_img_msgs_FOUND)
  ament_target_dependencies(line_vision_node hbm_img_msgs)
  target_include_directories(line_vision_node PRIVATE ${hbm_img_msgs_INCLUDE_DIRS} ${HBM_IMG_MSGS_INCLUDE_DIR})
  target_compile_definitions(line_vision_node PRIVATE DRONE_LINE_VISION_HAS_HBM=1)
else()
  message(WARNING "hbm_img_msgs not found; line_vision_node will build without camera subscription")
endif()

install(TARGETS line_vision_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()

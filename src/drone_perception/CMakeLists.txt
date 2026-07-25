cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# K230 bridge nodes remain independent from the Orange Pi RKNN production path.
add_executable(k230_tcp_bridge src/k230_tcp_bridge.cpp)
target_compile_features(k230_tcp_bridge PUBLIC cxx_std_17)
target_include_directories(k230_tcp_bridge PRIVATE include)
ament_target_dependencies(k230_tcp_bridge rclcpp std_msgs drone_msgs)
target_link_libraries(k230_tcp_bridge Threads::Threads)

add_executable(k230_animals_uart_ros2_node src/k230_animals_uart_ros2_node.cpp)
target_compile_features(k230_animals_uart_ros2_node PUBLIC cxx_std_17)
target_include_directories(k230_animals_uart_ros2_node PRIVATE include)
ament_target_dependencies(
  k230_animals_uart_ros2_node rclcpp std_msgs drone_msgs geometry_msgs)

add_executable(rdk_nv12_display_cpp scripts/rdk_nv12_display.cpp)
target_compile_features(rdk_nv12_display_cpp PUBLIC cxx_std_17)
target_include_directories(rdk_nv12_display_cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rdk_nv12_display_cpp ${OpenCV_LIBS} Threads::Threads)

install(TARGETS
  k230_tcp_bridge
  k230_animals_uart_ros2_node
  rdk_nv12_display_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/k230_animals_image_saver.sh
  scripts/k230_snapshot_decode.sh
  scripts/rdk_udp_vpu_bridge.sh
  scripts/rdk_nv12_display.py
  scripts/rdk_udp_vpu_display.sh
  scripts/rdk_realsense_vpu_display.sh
  scripts/orangepi_mjpeg_h264_sender.sh
  scripts/orangepi_mjpeg_rtp_sender.sh
  scripts/orangepi_realsense_h264_sender.sh
  scripts/rdk_mjpeg_rtp_display.sh
  scripts/rdk_mjpeg_rtp_cv_display.py
  DESTINATION lib/${PROJECT_NAME}
)

# Orange Pi RK3588 production path: industrial MJPEG camera + MPP + 3 RKNN contexts.
set(RKNN_INCLUDE $ENV{RKNN_INCLUDE})
set(RKNN_LIB $ENV{RKNN_LIB})
set(RKNN_RUNTIME_SO "${RKNN_LIB}/librknnrt.so")
pkg_check_modules(GSTREAMER QUIET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)
pkg_check_modules(RGA QUIET IMPORTED_TARGET librga)

if(RKNN_INCLUDE AND RKNN_LIB AND EXISTS "${RKNN_RUNTIME_SO}" AND GSTREAMER_FOUND)
  add_library(rknn_yolo_detector STATIC src/rknn_yolo_detector.cpp)
  target_compile_features(rknn_yolo_detector PUBLIC cxx_std_17)
  target_include_directories(rknn_yolo_detector
    PUBLIC include ${RKNN_INCLUDE}
    PRIVATE ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(rknn_yolo_detector PUBLIC ${OpenCV_LIBS} ${RKNN_RUNTIME_SO})
  if(RGA_FOUND)
    target_compile_definitions(rknn_yolo_detector PRIVATE DRONE_PERCEPTION_HAS_RGA=1)
    target_link_libraries(rknn_yolo_detector PRIVATE PkgConfig::RGA)
    message(STATUS "Industrial detector RGA preprocessing enabled")
  else()
    message(STATUS "librga not found; industrial detector will use OpenCV preprocessing")
  endif()

  add_executable(industrial_animal_vision_node
    src/main.cpp
    src/industrial_animal_vision_node.cpp
  )
  target_compile_features(industrial_animal_vision_node PUBLIC cxx_std_17)
  target_include_directories(industrial_animal_vision_node PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_INCLUDE}
    ${GSTREAMER_INCLUDE_DIRS}
  )
  target_link_directories(industrial_animal_vision_node PRIVATE
    ${GSTREAMER_LIBRARY_DIRS}
  )
  target_compile_options(industrial_animal_vision_node PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
  )
  ament_target_dependencies(industrial_animal_vision_node rclcpp ament_index_cpp drone_msgs)
  target_link_libraries(industrial_animal_vision_node
    rknn_yolo_detector
    ${OpenCV_LIBS}
    ${RKNN_RUNTIME_SO}
    ${GSTREAMER_LIBRARIES}
    Threads::Threads
  )

  install(TARGETS
    rknn_yolo_detector
    industrial_animal_vision_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS
    "Skipping industrial_animal_vision_node: need RKNN_INCLUDE, RKNN_LIB/librknnrt.so "
    "and GStreamer development modules (gstreamer-1.0, app, video).")
endif()

# Keep the standalone RDK X5 model probe available when its SDK is installed.
find_path(RDK_DNN_INCLUDE_DIR NAMES dnn/hb_dnn.h PATHS /usr/include)
find_library(RDK_DNN_LIBRARY NAMES dnn PATHS /usr/lib /usr/lib/aarch64-linux-gnu)
if(RDK_DNN_INCLUDE_DIR AND RDK_DNN_LIBRARY)
  add_executable(bpu_model_probe
    src/bpu_model_probe.cpp
    src/bpu_yolo_detector.cpp
  )
  target_compile_features(bpu_model_probe PUBLIC cxx_std_17)
  target_include_directories(bpu_model_probe PRIVATE include ${RDK_DNN_INCLUDE_DIR})
  target_link_libraries(bpu_model_probe ${RDK_DNN_LIBRARY})
  install(TARGETS bpu_model_probe DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Skipping bpu_model_probe because the RDK BPU SDK is unavailable.")
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

set(ANIMAL_MODELS
  "${CMAKE_CURRENT_SOURCE_DIR}/models/animal_clean200_yolo11n_fp16.rknn"
  "${CMAKE_CURRENT_SOURCE_DIR}/models/animal_clean200_yolo11n_int8.rknn"
)
foreach(ANIMAL_MODEL IN LISTS ANIMAL_MODELS)
  if(EXISTS "${ANIMAL_MODEL}")
    install(FILES "${ANIMAL_MODEL}" DESTINATION share/${PROJECT_NAME}/models)
  else()
    message(WARNING "Animal RKNN model is missing: ${ANIMAL_MODEL}")
  endif()
endforeach()

ament_package()

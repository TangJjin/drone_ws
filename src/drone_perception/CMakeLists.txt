cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(ZBAR REQUIRED zbar)

# ── k230_tcp_bridge ──────────────────────────────────────
add_executable(k230_tcp_bridge
  src/k230_tcp_bridge.cpp
)

target_compile_features(k230_tcp_bridge PUBLIC cxx_std_17)

target_include_directories(k230_tcp_bridge
  PRIVATE
  include
)

ament_target_dependencies(k230_tcp_bridge
  rclcpp
  std_msgs
  drone_msgs
)

target_link_libraries(k230_tcp_bridge Threads::Threads)

install(TARGETS
  k230_tcp_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ── k230_animals_uart_ros2_node ──────────────────────────
add_executable(k230_animals_uart_ros2_node
  src/k230_animals_uart_ros2_node.cpp
)

target_compile_features(k230_animals_uart_ros2_node PUBLIC cxx_std_17)

target_include_directories(k230_animals_uart_ros2_node
  PRIVATE
  include
)

ament_target_dependencies(k230_animals_uart_ros2_node
  rclcpp
  std_msgs
  drone_msgs
  geometry_msgs
)

install(TARGETS
  k230_animals_uart_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/k230_animals_image_saver.sh
  scripts/k230_snapshot_decode.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ── qr_vision_node ───────────────────────────────────────
add_executable(qr_vision_node
  src/main.cpp
  src/qr_vision_node.cpp
  src/depth_processor.cpp
)

target_compile_features(qr_vision_node PUBLIC cxx_std_17)

target_include_directories(qr_vision_node
  PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

target_link_libraries(qr_vision_node
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
)

target_link_directories(qr_vision_node
  PRIVATE
  ${ZBAR_LIBRARY_DIRS}
)

target_compile_options(qr_vision_node
  PRIVATE
  ${ZBAR_CFLAGS_OTHER}
)

ament_target_dependencies(qr_vision_node
  ament_index_cpp
  cv_bridge
  drone_msgs
  image_geometry
  message_filters
  rcl_interfaces
  rclcpp
  realsense2_camera_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS
  qr_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── optional standalone RKNN detector library ─────────────
set(RKNN_INCLUDE $ENV{RKNN_INCLUDE})
set(RKNN_LIB $ENV{RKNN_LIB})
set(RKNN_RUNTIME_SO "${RKNN_LIB}/librknnrt.so")

if(RKNN_INCLUDE AND RKNN_LIB AND EXISTS "${RKNN_RUNTIME_SO}")
  add_executable(rknn_model_probe
    src/rknn_model_probe.cpp
    src/depth_processor.cpp
  )

  target_compile_features(rknn_model_probe PUBLIC cxx_std_17)

  target_include_directories(rknn_model_probe
    PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_INCLUDE}
  )

  ament_target_dependencies(rknn_model_probe
    ament_index_cpp
    cv_bridge
    image_geometry
    rclcpp
    realsense2_camera_msgs
    sensor_msgs
  )

  # 路径 B：9 输出 YOLO11（实现在 rknn_yolo_detector.*）
  add_library(rknn_yolo_detector STATIC
    src/rknn_yolo_detector.cpp
  )

  target_compile_features(rknn_yolo_detector PUBLIC cxx_std_17)

  target_include_directories(rknn_yolo_detector
    PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_INCLUDE}
  )

  target_link_libraries(rknn_yolo_detector
    ${OpenCV_LIBS}
    ${RKNN_RUNTIME_SO}
  )

  target_link_libraries(rknn_model_probe
    rknn_yolo_detector
    ${OpenCV_LIBS}
    ${RKNN_RUNTIME_SO}
    realsense2::realsense2
  )

  add_executable(package_qr_shelf_rknn_probe
    src/package_qr_shelf_rknn_probe.cpp
    src/depth_processor.cpp
  )

  target_compile_features(package_qr_shelf_rknn_probe PUBLIC cxx_std_17)

  target_include_directories(package_qr_shelf_rknn_probe
    PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_INCLUDE}
  )

  ament_target_dependencies(package_qr_shelf_rknn_probe
    ament_index_cpp
    cv_bridge
    image_geometry
    rclcpp
    realsense2_camera_msgs
    sensor_msgs
  )

  target_link_libraries(package_qr_shelf_rknn_probe
    rknn_yolo_detector
    ${OpenCV_LIBS}
    ${RKNN_RUNTIME_SO}
    realsense2::realsense2
  )

  # 业务节点接入路径 B RKNN（仅 ZBar 解二维码，不接 OCR）
  target_include_directories(qr_vision_node
    PRIVATE
    ${RKNN_INCLUDE}
  )
  target_link_libraries(qr_vision_node
    rknn_yolo_detector
    ${RKNN_RUNTIME_SO}
  )
  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_RKNN=1
  )

  install(TARGETS
    rknn_model_probe
    rknn_yolo_detector
    package_qr_shelf_rknn_probe
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    qr_rk3588_FP16.rknn
    qr_rk3588_hybrid_bbox_fp16.rknn
    package_qrcode_shelf_tag_fp16.rknn
    DESTINATION share/${PROJECT_NAME}/models
  )
else()
  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_RKNN=0
  )
  message(STATUS
    "Skipping standalone RKNN detector library because RKNN runtime is unavailable. "
    "Set RKNN_INCLUDE and RKNN_LIB so that ${RKNN_RUNTIME_SO} exists to enable it.")
endif()

# ── optional RDK X5 BPU model probe ──────────────────────
find_path(RDK_DNN_INCLUDE_DIR
  NAMES dnn/hb_dnn.h
  PATHS /usr/include
)

find_library(RDK_DNN_LIBRARY
  NAMES dnn
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu
)

if(RDK_DNN_INCLUDE_DIR AND RDK_DNN_LIBRARY)
  target_sources(qr_vision_node
    PRIVATE
    src/bpu_yolo_detector.cpp
    src/bpu_ocr_pipeline.cpp
  )

  target_include_directories(qr_vision_node
    PRIVATE
    ${RDK_DNN_INCLUDE_DIR}
  )

  target_link_libraries(qr_vision_node
    ${RDK_DNN_LIBRARY}
  )

  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_BPU=1
  )

  add_executable(bpu_model_probe
    src/bpu_model_probe.cpp
    src/bpu_yolo_detector.cpp
  )

  target_compile_features(bpu_model_probe PUBLIC cxx_std_17)

  target_include_directories(bpu_model_probe
    PRIVATE
    include
    ${RDK_DNN_INCLUDE_DIR}
  )

  target_link_libraries(bpu_model_probe
    ${RDK_DNN_LIBRARY}
  )

  install(TARGETS
    bpu_model_probe
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_BPU=0
  )

  message(STATUS
    "Skipping bpu_model_probe because RDK BPU SDK is unavailable. "
    "Need dnn/hb_dnn.h and libdnn.so.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

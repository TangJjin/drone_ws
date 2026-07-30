cmake_minimum_required(VERSION 3.8)
project(drone_perception)

# 未显式指定时 CMake 不加任何 -O 优化参数，CPU 侧热点（letterbox 拷贝、
# INT8 填充、后处理扫描）会以无优化版本运行，故默认 Release。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# aarch64 GCC 对 std::pair 等小聚合类型的返回打印 psABI 变更注记，
# 仅在与 GCC<10.1 编译的旧目标文件混链时才有意义；全工程同版本编译，静音之。
if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge QUIET)
find_package(sensor_msgs QUIET)
find_package(realsense2_camera_msgs QUIET)
find_package(hbm_img_msgs QUIET)
find_path(HBM_IMG_MSGS_INCLUDE_DIR NAMES hbm_img_msgs/msg/hbm_msg1080_p.hpp
  PATHS /opt/tros/humble/include/hbm_img_msgs /opt/ros/humble/include/hbm_img_msgs)

add_executable(line_vision_node
  src/line_vision_main.cpp
  src/line_vision_node.cpp
  src/line_detector.cpp
  src/frame_statistics.cpp
)
target_compile_features(line_vision_node PUBLIC cxx_std_17)
target_include_directories(line_vision_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_vision_node ${OpenCV_LIBS} yaml-cpp)
ament_target_dependencies(line_vision_node rclcpp std_msgs drone_msgs ament_index_cpp)
if(hbm_img_msgs_FOUND AND HBM_IMG_MSGS_INCLUDE_DIR)
  ament_target_dependencies(line_vision_node hbm_img_msgs)
  target_include_directories(line_vision_node PRIVATE ${HBM_IMG_MSGS_INCLUDE_DIR})
  target_compile_definitions(line_vision_node PRIVATE DRONE_LINE_VISION_HAS_HBM=1)
else()
  message(WARNING "hbm_img_msgs not found; line_vision_node will not subscribe to HBM")
endif()

add_executable(rdk_nv12_display_cpp scripts/rdk_nv12_display.cpp)
target_compile_features(rdk_nv12_display_cpp PUBLIC cxx_std_17)
target_include_directories(rdk_nv12_display_cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rdk_nv12_display_cpp ${OpenCV_LIBS} Threads::Threads)

install(TARGETS
  rdk_nv12_display_cpp
  line_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(air_ground_servo_depth STATIC src/air_ground_servo_depth.cpp)
target_compile_features(air_ground_servo_depth PUBLIC cxx_std_17)
target_include_directories(air_ground_servo_depth
  PUBLIC include
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(air_ground_servo_depth ${OpenCV_LIBS})
ament_target_dependencies(air_ground_servo_depth sensor_msgs)

add_library(air_ground_target_geometry STATIC src/air_ground_target_geometry.cpp)
target_compile_features(air_ground_target_geometry PUBLIC cxx_std_17)
target_include_directories(air_ground_target_geometry
  PUBLIC include
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(air_ground_target_geometry ${OpenCV_LIBS})

if(cv_bridge_FOUND AND sensor_msgs_FOUND AND realsense2_camera_msgs_FOUND)
  add_executable(air_ground_servo_node src/air_ground_servo_node.cpp)
  target_compile_features(air_ground_servo_node PUBLIC cxx_std_17)
  target_include_directories(air_ground_servo_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(
    air_ground_servo_node air_ground_servo_depth air_ground_target_geometry ${OpenCV_LIBS})
  ament_target_dependencies(
    air_ground_servo_node rclcpp cv_bridge sensor_msgs geometry_msgs realsense2_camera_msgs)
  install(TARGETS air_ground_servo_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "Skipping air_ground_servo_node: need cv_bridge, sensor_msgs, and "
    "realsense2_camera_msgs development packages.")
endif()

install(PROGRAMS
  scripts/rdk_udp_vpu_bridge.sh
  scripts/rdk_nv12_display.py
  scripts/rdk_udp_vpu_display.sh
  scripts/rdk_realsense_vpu_display.sh
  scripts/orangepi_mjpeg_h264_sender.sh
  scripts/orangepi_mjpeg_rtp_sender.sh
  scripts/orangepi_realsense_h264_sender.sh
  scripts/rdk_mjpeg_rtp_display.sh
  scripts/rdk_mjpeg_rtp_cv_display.py
  scripts/rdk_d435i_mediamtx_sender.sh
  scripts/start_rdk_d435i_sender.sh
  scripts/camera_param_tuner.py
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

# RDK X5 standalone D435i H.264 sender. Keep this conditional so non-RDK
# workstations and the RK3588 target can still build the package.
find_path(RDK_MEDIA_CODEC_INCLUDE_DIR NAMES hb_media_codec.h PATHS /usr/include)
find_library(RDK_MULTIMEDIA_LIBRARY NAMES multimedia PATHS /usr/hobot/lib /usr/lib)
pkg_check_modules(RDK_FFMPEG QUIET IMPORTED_TARGET libswscale libavutil)
find_path(RDK_SP_INCLUDE_DIR NAMES sp_codec.h sp_display.h PATHS /usr/include)
find_library(RDK_SPCDEV_LIBRARY NAMES spcdev PATHS /lib /usr/lib)
pkg_check_modules(RDK_AVFORMAT QUIET IMPORTED_TARGET libavformat libavcodec libavutil)

if(RDK_MEDIA_CODEC_INCLUDE_DIR AND RDK_MULTIMEDIA_LIBRARY AND RDK_FFMPEG_FOUND)
  add_executable(rdk_d435i_h264_sender src/rdk_d435i_h264_sender.cpp)
  target_compile_features(rdk_d435i_h264_sender PUBLIC cxx_std_17)
  target_include_directories(rdk_d435i_h264_sender PRIVATE
    ${RDK_MEDIA_CODEC_INCLUDE_DIR}
  )
  target_link_libraries(rdk_d435i_h264_sender
    ${RDK_MULTIMEDIA_LIBRARY}
    PkgConfig::RDK_FFMPEG
  )
  install(TARGETS rdk_d435i_h264_sender DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "Skipping rdk_d435i_h264_sender: need RDK hb_media_codec/libmultimedia "
    "and libswscale/libavutil development files.")
endif()

if(RDK_SP_INCLUDE_DIR AND RDK_SPCDEV_LIBRARY AND RDK_AVFORMAT_FOUND)
  add_executable(rdk_rtsp_display_receiver src/rdk_rtsp_display_receiver.cpp)
  target_compile_features(rdk_rtsp_display_receiver PUBLIC cxx_std_17)
  target_include_directories(rdk_rtsp_display_receiver PRIVATE ${RDK_SP_INCLUDE_DIR})
  target_link_libraries(rdk_rtsp_display_receiver
    ${RDK_SPCDEV_LIBRARY}
    PkgConfig::RDK_AVFORMAT
  )
  install(TARGETS rdk_rtsp_display_receiver DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "Skipping rdk_rtsp_display_receiver: need RDK spcdev and FFmpeg development files.")
endif()

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
  ament_target_dependencies(
    industrial_animal_vision_node rclcpp ament_index_cpp geometry_msgs drone_msgs)
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(
      air_ground_servo_depth_test
      test/air_ground_servo_depth_test.cpp
    )
    target_include_directories(
      air_ground_servo_depth_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(
      air_ground_servo_depth_test air_ground_servo_depth ${OpenCV_LIBS})
    ament_add_gtest(
      air_ground_target_geometry_test
      test/air_ground_target_geometry_test.cpp
    )
    target_include_directories(
      air_ground_target_geometry_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(
      air_ground_target_geometry_test air_ground_target_geometry ${OpenCV_LIBS})
  else()
    message(WARNING "ament_cmake_gtest not found; RGBD depth tests are unavailable.")
  endif()
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(
      air_ground_servo_launch_test
      test/air_ground_servo_launch_test.py
    )
  else()
    message(WARNING "ament_cmake_pytest not found; RGBD launch tests are unavailable.")
  endif()
endif()

ament_package()

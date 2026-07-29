#pragma once

#include <string>

namespace drone_line_vision
{
struct CameraConfig {
  std::string source_topic{"/line_vision/nv12"};
  std::string device;
  int width{1280};
  int height{720};
  int fps{60};
  std::string pixel_format{"mjpeg"};
  std::string io_method{"mmap"};
  bool zero_copy{true};
  std::string source_encoding{"nv12"};
};

struct RoiConfig { bool enabled{false}; int x{0}; int y{0}; int width{0}; int height{0}; };
struct ThresholdConfig {
  std::string mode{"fixed"}; int gray_threshold{70}; bool invert{false};
  int min_candidate_pixels{100}; int max_candidate_pixels{100000};
};
struct MorphologyConfig {
  bool enabled{true}; int kernel_size{3}; int open_iterations{0}; int close_iterations{1};
};
struct LineFitConfig {
  int min_component_area{80}; double max_fit_residual_px{15.0};
  double min_line_length_px{80.0}; double near_scan_ratio{0.75}; double far_scan_ratio{0.25};
};
struct DisplayConfig {
  bool enabled{true}; int window_width{1600}; int window_height{900}; int display_fps_limit{30};
  bool show_debug_text{true}; bool show_binary_mask{true}; bool show_data_panel{true};
};
struct LoggingConfig {
  bool enabled{true}; std::string output_directory{"/tmp/line_vision"};
  bool csv_enabled{true}; bool json_enabled{true}; bool save_images_on_invalid{false};
};
struct RuntimeConfig { char reload_key{'r'}; char save_key{'s'}; char pause_key{'p'}; char quit_key{'q'}; };
struct LineVisionConfig {
  CameraConfig camera; RoiConfig roi; ThresholdConfig threshold; MorphologyConfig morphology;
  LineFitConfig line_fit; DisplayConfig display; LoggingConfig logging; RuntimeConfig runtime;
  std::string config_file;
};
}

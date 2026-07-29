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
  int adaptive_block_size{31}; double adaptive_c{5.0};
  int min_candidate_pixels{100}; int max_candidate_pixels{100000};
};
struct MorphologyConfig {
  bool enabled{true}; int kernel_size{3}; int open_iterations{0}; int close_iterations{1};
};
struct LineFitConfig {
  int min_component_area{80}; double max_fit_residual_px{15.0};
  double min_line_length_px{80.0}; double near_scan_ratio{0.75}; double far_scan_ratio{0.25};
};
struct CurveConfig {
  int min_fit_points{40}; double max_fit_residual_px{15.0};
  double straight_curvature_threshold_px_inv{0.0005};
  double reference_row_ratio{0.75};
  int direction_sign{1};
};
struct CenterlineConfig {
  int row_step_px{4}; int min_valid_rows{40}; int min_band_width_px{4};
  int max_band_width_px{500}; int max_center_jump_px{80}; int max_gap_rows{16};
  double bottom_search_ratio{0.75};
};
struct DisplayConfig {
  bool enabled{true}; int window_width{1600}; int window_height{900}; int display_fps_limit{60};
  bool show_debug_text{true}; bool show_binary_mask{true}; bool show_data_panel{true};
};
struct LoggingConfig {
  bool enabled{true}; std::string output_directory{"/tmp/line_vision"};
  bool csv_enabled{true}; bool json_enabled{true}; bool save_images_on_invalid{false};
};
struct RuntimeConfig { char reload_key{'r'}; char save_key{'s'}; char pause_key{'p'}; char quit_key{'q'}; };
struct LineVisionConfig {
  CameraConfig camera; RoiConfig roi; ThresholdConfig threshold; MorphologyConfig morphology;
  LineFitConfig line_fit; CurveConfig curve; CenterlineConfig centerline;
  DisplayConfig display; LoggingConfig logging; RuntimeConfig runtime;
  std::string config_file;
};
}

std_msgs/Header header

# Frame validity and geometry source health.
bool valid
bool decode_ok
bool roi_valid
bool curve_valid

# Image geometry, in pixels.
uint32 image_width_px
uint32 image_height_px
float32 line_center_u_px
float32 line_center_v_px
float32 image_center_u_px
float32 error_u_px

# Line orientation and curve geometry in the image plane.
# heading_error_rad > 0 means the line heads to the right in the forward image direction.
float32 heading_error_rad
# Raw undirected fit-line angle retained for diagnostics only.
float32 line_angle_rad
# Curvature magnitude in 1/pixel. It is zero for a straight line.
float32 curvature_px_inv
# -1: left curve, 0: straight, 1: right curve. Valid only when curve_valid is true.
int8 curve_direction

# Detection quality and runtime diagnostics.
float32 confidence
uint32 candidate_pixel_count
uint32 lost_frame_count
uint32 processing_time_us
float32 capture_fps
float32 observation_fps

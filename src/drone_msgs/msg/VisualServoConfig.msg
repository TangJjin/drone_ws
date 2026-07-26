# Target identifier. An empty value locks the first eligible target.
string target_id

# Require the vision source to mark a target as stably confirmed.
bool require_confirmed

# Body axes driven by horizontal and vertical image error: x, y, or z.
string image_x_axis
string image_y_axis

# Direction multipliers. Valid values are -1.0 and 1.0.
float64 image_x_sign
float64 image_y_sign

# PID gains for normalized horizontal and vertical image error.
float64 kp_x
float64 ki_x
float64 kd_x
float64 kp_y
float64 ki_y
float64 kd_y

# Integral anti-windup limit and low-pass filter coefficient.
float64 integral_limit
float64 filter_alpha

# Alignment hysteresis and required stable duration.
float64 enter_tolerance_x
float64 enter_tolerance_y
float64 exit_tolerance_x
float64 exit_tolerance_y
float64 settle_time_s

# Target acquisition, target loss, and whole-action timeout limits.
float64 acquire_timeout_s
float64 lost_timeout_s
float64 overall_timeout_s

# Per-axis body-frame speed limit and timeout handling policy.
float64 max_body_speed_mps
bool continue_on_timeout

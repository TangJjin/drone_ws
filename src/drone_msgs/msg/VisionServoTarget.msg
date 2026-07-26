builtin_interfaces/Time stamp
uint32 sequence

# Stable identifier selected by the vision side. It may be empty when only one target exists.
string target_id
bool valid
bool confirmed
float64 confidence

# Normalized image error in [-1, 1]: target center minus image center.
# Positive error_x points right; positive error_y points down.
float64 error_x
float64 error_y

# Pixel fields are diagnostic only. Control uses normalized error_x/error_y.
uint32 image_width
uint32 image_height
float64 center_x
float64 center_y


builtin_interfaces/Time stamp

# request_id is zero for a startup status publication.
uint64 request_id
bool success
uint64 applied_mask
uint64 rejected_mask
uint64 available_mask
uint64 writable_mask
uint64 active_mask
string message

# Current values read back from the V4L2 driver where possible.
int32 exposure_auto
int32 exposure_absolute
int32 exposure_auto_priority
int32 gain
int32 brightness
int32 contrast
int32 saturation
int32 gamma
int32 sharpness
int32 backlight_compensation
int32 white_balance_auto
int32 white_balance_temperature
int32 power_line_frequency
int32 focus_auto
int32 focus_absolute
int32 zoom_absolute

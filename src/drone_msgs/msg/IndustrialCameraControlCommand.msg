# Correlates a command with IndustrialCameraControlState.request_id.
uint64 request_id

# Command type.
uint8 COMMAND_APPLY=0
uint8 COMMAND_SAVE_CURRENT=1
uint8 COMMAND_RESTORE_PROJECT_DEFAULTS=2
uint8 command

# Only fields whose bits are set are applied for COMMAND_APPLY.
uint64 UPDATE_EXPOSURE_AUTO=1
uint64 UPDATE_EXPOSURE_ABSOLUTE=2
uint64 UPDATE_EXPOSURE_AUTO_PRIORITY=4
uint64 UPDATE_GAIN=8
uint64 UPDATE_BRIGHTNESS=16
uint64 UPDATE_CONTRAST=32
uint64 UPDATE_SATURATION=64
uint64 UPDATE_GAMMA=128
uint64 UPDATE_SHARPNESS=256
uint64 UPDATE_BACKLIGHT_COMPENSATION=512
uint64 UPDATE_WHITE_BALANCE_AUTO=1024
uint64 UPDATE_WHITE_BALANCE_TEMPERATURE=2048
uint64 UPDATE_POWER_LINE_FREQUENCY=4096
uint64 UPDATE_FOCUS_AUTO=8192
uint64 UPDATE_FOCUS_ABSOLUTE=16384
uint64 UPDATE_ZOOM_ABSOLUTE=32768
uint64 update_mask

# Values use the native integer representation reported by V4L2.
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

# A V4L2 control advertised by IndustrialCameraControlCapabilities.
uint8 CONTROL_TYPE_INTEGER=0
uint8 CONTROL_TYPE_BOOLEAN=1
uint8 CONTROL_TYPE_MENU=2

string name
uint64 update_mask
uint8 control_type
bool available
bool writable
bool active
int32 minimum
int32 maximum
int32 step
int32 default_value
int32 current_value

# Populated for CONTROL_TYPE_MENU. Values and labels have matching indexes.
int32[] menu_values
string[] menu_labels

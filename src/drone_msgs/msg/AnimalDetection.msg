# YOLO class label, for example "elephant" or "tiger".
string label

# Stable target identifier. It is allocated by the perception node and is not
# reused until that node restarts.
uint64 track_id

# Detector confidence in the range [0.0, 1.0].
float64 score

# Target center in pixels in the source image.
int32 cx
int32 cy

# Target center relative to the image center, in pixels.
# Positive err_x means right of center; positive err_y means below center.
int32 err_x
int32 err_y

# Normalized center error. Positive directions match err_x and err_y.
float64 norm_x
float64 norm_y

# Bounding box in source-image pixels. x2/y2 are the exclusive right/bottom
# edge, equivalent to x1 + bbox_w and y1 + bbox_h.
int32 x1
int32 y1
int32 x2
int32 y2
int32 bbox_w
int32 bbox_h
int32 bbox_area

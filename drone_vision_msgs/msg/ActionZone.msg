std_msgs/Header header

# Zone type
string zone_type           # "landing" | "takeoff" | "drop_payload"
bool   zone_detected

# Pixel location in camera frame
float32[2] center_px       # [cx, cy]
float32[4] bbox_xyxy       # Bounding region
float32    area_ratio      # How much of frame the zone covers

# Confidence and safety
float32 detection_confidence   # YOLO confidence if marker detected
float32 gemma_confidence       # Gemma confidence score [0.0-1.0]
float32 clearance_score        # Open space score [0.0-1.0]
string  safety_assessment      # "safe" | "caution" | "unsafe"

# Gemma reasoning
string description             # "Flat concrete with H marker visible"
string reasoning               # Why Gemma says safe/unsafe
string recommended_action      # "proceed" | "hold" | "abort"

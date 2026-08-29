std_msgs/Header header

# Class and category info
string class_name          # YOLO class: "person", "car", "tree", etc.
string category            # "target" | "landing_zone" | "takeoff_zone" | "drop_zone" | "obstacle"
int32  track_id            # Unique tracker ID (-1 if not tracked)
float32 confidence         # YOLO confidence score [0.0-1.0]

# Bounding box in pixel coordinates [x1, y1, x2, y2]
float32[4] bbox_xyxy
float32[2] center_px       # [cx, cy] pixel center

# Spatial info
float32 area_ratio         # bbox area / total frame area [0.0-1.0]
string  placement_h        # "left" | "center" | "right"
string  placement_v        # "top" | "middle" | "bottom"
string  depth_estimate     # "near" | "mid" | "far" (from area_ratio)

# Gemma enrichment (filled every N frames)
string  gemma_description  # Gemma textual description of this object
string  safety_note        # "clear" | "caution" | "danger"

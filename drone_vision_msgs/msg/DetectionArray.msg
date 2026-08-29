std_msgs/Header header

# All detections in this frame
drone_vision_msgs/DetectedObject[] objects

# Counts by category
int32 total_count
int32 target_count
int32 obstacle_count
int32 landing_zone_count
int32 drop_zone_count

# Performance
float32 fps
float32 yolo_inference_ms
int32   frame_id

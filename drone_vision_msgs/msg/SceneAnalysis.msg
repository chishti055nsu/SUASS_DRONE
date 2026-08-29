std_msgs/Header header

# Raw Gemma output
string raw_json                # Full structured JSON from Gemma

# Scene description
string scene_description       # "Open field with H-marker at center-left"
string[] detected_features     # List of notable features Gemma saw

# Zone analysis
drone_vision_msgs/ActionZone landing_zone
drone_vision_msgs/ActionZone takeoff_zone
drone_vision_msgs/ActionZone drop_zone

# Obstacle summary
float32 obstacle_density       # 0.0-1.0
string  obstacle_summary       # "3 trees on left, clear center"

# Mission recommendation
string recommended_action      # "land" | "takeoff" | "drop" | "hold" | "avoid" | "search"
string action_direction        # "left" | "right" | "forward" | "back" | "descend" | "ascend"
string reasoning               # Full Gemma reasoning text
float32 gemma_inference_ms     # Time taken for Gemma to respond
int32   frame_id               # Which frame was analyzed

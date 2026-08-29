std_msgs/Header header

# All detected obstacles
drone_vision_msgs/DetectedObject[] obstacles

# Aggregated risk assessment
float32 obstacle_density       # 0.0 = clear, 1.0 = fully blocked
string  risk_level             # "low" | "medium" | "high" | "critical"
string  primary_threat         # Description of nearest/biggest obstacle
bool    path_clear             # True if no obstacle in center region

# Directional clearance (useful for avoidance)
bool left_clear
bool right_clear
bool center_clear
bool top_clear
bool bottom_clear

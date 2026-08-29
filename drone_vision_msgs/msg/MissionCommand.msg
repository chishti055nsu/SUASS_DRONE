std_msgs/Header header

# Command to mission planner
string command                 # "start" | "abort" | "hold" | "resume" | "land_now" | "rtl" | "terminate"

# Optional override parameters
float32 target_altitude_m      # Override target altitude
float32[3] goto_ned            # Override position [N, E, D]
bool    force_land             # Force landing regardless of zone assessment
string  mission_file           # Path to mission YAML file to load

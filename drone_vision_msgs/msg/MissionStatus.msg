std_msgs/Header header

# Mission state machine state
string state                   # See MissionStateMachine states below
# States: IDLE | ARMING | TAKEOFF | SEARCH | LOITER |
#         APPROACH_TARGET | DROP_PAYLOAD | RETURN_HOME | LAND | ABORT | COMPLETE

# Current waypoint info
int32  current_waypoint_index
int32  total_waypoints
float32[3] current_target_ned  # [North, East, Down] meters from home

# Flight data
float32 altitude_m             # Current altitude in meters
float32 battery_percent        # Battery level [0-100]
float32 groundspeed_ms         # Ground speed m/s
float32 distance_to_target_m   # Distance to current waypoint

# Mission progress
float32 mission_progress       # 0.0-1.0
bool    payload_dropped
bool    target_acquired
bool    landing_zone_confirmed

# Timestamps
float32 mission_elapsed_sec    # Seconds since mission start
string  status_message         # Human-readable status

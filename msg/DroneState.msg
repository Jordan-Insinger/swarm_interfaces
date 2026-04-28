# DroneState.msg
std_msgs/Header header
string agent_id

# flight state
bool armed
bool offboard_mode
string flight_mode       # raw MAVROS mode string
uint8 landed_state

# local position (Autonomy Park Frame)
geometry_msgs/PoseStamped apark_pose

# global position (lat/lon)
geographic_msgs/GeoPoseStamped global_pose

# local velocity (Autonomy Park Frame)
geometry_msgs/TwistStamped apark_vel

# body velocity
geometry_msgs/Twist body_vel

# health
float32 battery_percent
float32 battery_voltage


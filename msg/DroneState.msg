# DroneState.msg
std_msgs/Header header
string agent_id

# flight state
uint8 agent_state       # Mavros Extended State
string flight_mode      # Mavros State
bool armed
bool offboard_mode

# position (Autonomy Park frame)
geometry_msgs/PoseStamped local_pose

# velocity
geometry_msgs/TwistStamped velocity

# global
geographic_msgs/GeoPoseStamped global_pose

# health
float32 battery_percentage
float32 battery_voltage



std_msgs/Header header
string state
string owner
string active_goal_id
bool ready

# Seconds remaining on the current ownership lease.
float64 lease_remaining
bool joint_goals_supported
bool cartesian_goals_supported
bool gravity_compensation_supported
bool feedforward_supported
string message

# Wrist availability is independent of body ready/fault status.
bool wrist_enabled
string[] wrist_joint_names
bool[] wrist_ready
string wrist_message

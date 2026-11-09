# Simulation-only, one leased injection stream per robot. Body axes use the
# same convention as the vehicle dynamics; torque is about the body origin.
std_msgs/Header header
string session_id
uint64 sequence
# Acquisition must contain only zeros. Release invalidates this session.
bool acquire
bool release
geometry_msgs/Wrench body_wrench
string[4] joint_names
float64[4] joint_torques

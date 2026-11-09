# Heartbeat plus the load supplied to the last COMPLETED dynamics step.
# A locked joint can have an applied external load without moving.
std_msgs/Header header
string component
string lease_owner
bool lease_active
bool paused
uint64 generation
uint64 step
float64 dynamics_time
float64 step_duration
string applied_session
uint64 applied_sequence
geometry_msgs/Wrench contact_body_wrench
geometry_msgs/Wrench applied_body_wrench
string[4] joint_names
float64[4] applied_joint_torques
string last_rejection

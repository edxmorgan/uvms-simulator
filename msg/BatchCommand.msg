# Batched whole-body command/action stream for UVMS batch simulation.

std_msgs/Header header

# Monotonic training/control tick. The simulator echoes the applied tick in
# BatchObservation so external RL nodes can stay synchronized.
uint64 tick_id

uint32 robot_count
uint32 action_dim
uint32 vehicle_action_dim
uint32 arm_action_dim
uint32 arm_joint_count

# Flattened row-major [robot_count, action_dim].
# The mock backend interprets:
#   actions[:, 0:6] as vehicle body acceleration commands
#   actions[:, vehicle_action_dim:vehicle_action_dim+arm_action_dim] as arm joint accelerations
float32[] actions

# Batched whole-body observation/state stream for UVMS batch simulation.

std_msgs/Header header
uint64 tick_id

uint32 robot_count
uint32 observation_dim
uint32 vehicle_state_dim
uint32 arm_state_dim
uint32 arm_joint_count
uint32 action_dim
uint32 vehicle_action_dim
uint32 arm_action_dim

# Flattened row-major [robot_count, observation_dim].
# The UVMS dynamics backend observation is:
# [vehicle_state(12), arm_position(arm_joint_count), arm_velocity(arm_joint_count)].
float32[] observations

# RL-oriented fields. BatchUvmsCore fills rewards with 0 and dones with 0
# unless a C++ task is attached.
float32[] rewards
uint8[] dones

# Selected robot state is repeated here for low-cost debug consumers.
uint32 selected_robot_index
float32[] selected_state

# Batch simulator timing diagnostics in milliseconds.
float32 gpu_step_ms
float32 host_to_device_ms
float32 device_to_host_ms
float32 publish_ms

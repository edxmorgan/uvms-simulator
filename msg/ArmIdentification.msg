builtin_interfaces/Time stamp
string joint
string subsystem
string status
string detail
uint64 epoch
float64 retained_accepted_time
float64 observation_time
float64[15] model_context
uint64 source_generation
bool measurements_fresh
bool retained_valid
bool applied
uint32 rows
uint32 information_rank
uint32 information_dimensions
float64 min_information
# Link-major [m,mx,my,mz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,fv,fc] for e,d,c,b.
# Gripper order: mass, damping, stiffness. Link parameters are not individually unique.
float64[] candidate_parameters
float64[] retained_parameters
float64 validation_normalized_rms
float64 retained_normalized_rms

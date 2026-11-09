# Identification diagnostics. Only validated retained parameters may feed a
# simulation EKF; candidate parameters are never applied directly.
builtin_interfaces/Time stamp
string joint
string status
string detail
# True only when simulation readback acknowledges this retained model in its EKF.
# Does not imply controller parameter application or current model validity.
bool applied
uint64 epoch
uint64 source_generation
bool hardware_measurements
uint32 window_samples
float64 window_start
float64 window_end
float64[2] weight_bounds
# Shared vehicle regressor order: inertia(10), W,B, restoring moments(3),
# linear drag(6), quadratic drag(6). NED/FRD.
float64[27] parameters
float64[6] residual_rms
uint32 numerical_rank
uint32 robust_iterations
float64 relative_objective_change
float64 solver_seconds
float64 fit_seconds

# Retention is never controller approval. Validity expires on
# discontinuity, stopped ingestion or persistent independent-validation failure.
bool retained_available
bool retained_valid
# Retained W/B are representatives valid only where the regressor uses W-B.
bool retained_net_weight_only
float64[27] retained_parameters
float64 retained_accepted_time
float64 retained_min_information
# Engineering-scaled average sensitivity information; NOT Fisher covariance.
uint32 information_rank
uint32 information_dimensions
float64 min_information
float64 information_condition
# True: only W-B was identifiable, individual W and B remain representatives.
bool net_weight_only
# Maximum per-axis RMS divided by configured residual scale; -1 = not evaluated.
float64 validation_error
float64 retained_validation_error
uint32 contradiction_blocks
# Per-axis retained-model RMS on recent measurements after model acceptance.
# Fx,Fy,Fz in N; Tx,Ty,Tz in Nm. Window uses retention_validation_seconds.
bool retained_rms_available
float64[6] retained_residual_rms
uint32 retained_rms_samples
float64 retained_rms_window_start
float64 retained_rms_window_end

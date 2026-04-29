# Exported diff_uv lumped vehicle model parameters.

# Inertia/Coriolis lumped parameters.
float64 m_x_du
float64 m_y_dv
float64 m_z_dw
float64 mz_g_x_dq
float64 mz_g_y_dp
float64 mz_g_k_dv
float64 mz_g_m_du
float64 i_x_k_dp
float64 i_y_m_dq
float64 i_z_n_dr

# Hydrostatic restoring lumped parameters.
float64 weight
float64 buoyancy
float64 x_g_weight_minus_x_b_buoyancy
float64 y_g_weight_minus_y_b_buoyancy
float64 z_g_weight_minus_z_b_buoyancy

# Linear damping lumped parameters.
float64 x_u
float64 y_v
float64 z_w
float64 k_p
float64 m_q
float64 n_r

# Quadratic damping lumped parameters.
float64 x_uu
float64 y_vv
float64 z_ww
float64 k_pp
float64 m_qq
float64 n_rr

# Body-frame current velocity [u_c, v_c, w_c, p_c, q_c, r_c].
float64[6] current_velocity

# Row-major 6x8 thrust configuration/allocation matrix.
# Rows are [Fx, Fy, Fz, Tx, Ty, Tz], columns are thruster channels 0..7.
float64[48] thrust_configuration_matrix

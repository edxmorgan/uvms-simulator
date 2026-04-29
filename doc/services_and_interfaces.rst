Services, Actions, and Topics
=============================

This page lists the runtime ROS interfaces that are useful when operating,
debugging, or integrating with the UVMS stack.

Inspecting Runtime Interfaces
-----------------------------

Use standard ROS 2 tools to inspect the active launch:

.. code-block:: shell

   ros2 service list
   ros2 action list
   ros2 topic list
   ros2 control list_controllers
   ros2 control list_hardware_interfaces

Most robot-specific names include the robot prefix. Simulated robot instances
use prefixes such as ``robot_1_``. The ``robot_real_`` prefix is used for the
hardware namespace, including mixed hardware/simulation launches.

Simulation Reset and Dynamics Services
--------------------------------------

The simulator exposes per-robot reset, release, and dynamics services. For a
simulated robot prefix such as ``robot_1_``:

- ``/robot_1_reset_sim_uvms``: reset manipulator and/or vehicle through
  ``sim_reset_coordinator``.
- ``/robot_1_release_sim_uvms``: release commands after a held combined reset.
- ``/robot_1_reset_sim_manipulator``: reset only the simulated manipulator.
- ``/robot_1_release_sim_manipulator``: release manipulator commands after a
  held reset.
- ``/robot_1_reset_sim_vehicle``: reset only the simulated vehicle.
- ``/robot_1_release_sim_vehicle``: release vehicle commands after a held
  reset.
- ``/robot_1_set_sim_uvms_dynamics``: update typed manipulator and/or vehicle
  dynamics parameters through ``sim_reset_coordinator``.
- ``/robot_1_set_sim_manipulator_dynamics``: update only manipulator dynamics.
- ``/robot_1_set_sim_vehicle_dynamics``: update only vehicle dynamics.

``ResetSimUvms`` request fields:

.. code-block:: text

   bool reset_manipulator
   bool reset_vehicle
   bool hold_commands

   bool use_manipulator_state
   float64[5] manipulator_position
   float64[5] manipulator_velocity

   bool use_vehicle_state
   float64[6] vehicle_pose    # [x, y, z, roll, pitch, yaw]
   float64[6] vehicle_twist   # [u, v, w, p, q, r]
   float64[6] vehicle_wrench  # [Fx, Fy, Fz, Tx, Ty, Tz]

   bool use_coupled_dynamics
   bool set_vehicle_dynamics
   SimVehicleDynamics vehicle_dynamics
   bool set_manipulator_dynamics
   SimManipulatorDynamics manipulator_dynamics

``hold_commands`` keeps simulated manipulator and vehicle commands blocked
after reset until the matching release service is called.

Use ``hold_commands=true`` when you want to inspect the reset state before
controllers drive again. Use ``hold_commands=false`` when a replay/profile
manager should reset and then start immediately.

``use_coupled_dynamics`` tells the backend how to interpret the supplied
vehicle and manipulator parameter sets. Use ``false`` for independent vehicle
and manipulator subsystem parameters. Use ``true`` when the parameters belong
to one coupled UVMS dynamics model.

``SetSimDynamics`` request fields:

.. code-block:: text

   bool use_coupled_dynamics
   bool set_vehicle_dynamics
   SimVehicleDynamics vehicle
   bool set_manipulator_dynamics
   SimManipulatorDynamics manipulator

The response reports ``success`` and ``message``.

``SimVehicleDynamics`` contains the lumped vehicle model parameters used by
the simulated vehicle backend:

.. code-block:: text

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

   float64 weight
   float64 buoyancy
   float64 x_g_weight_minus_x_b_buoyancy
   float64 y_g_weight_minus_y_b_buoyancy
   float64 z_g_weight_minus_z_b_buoyancy

   float64 x_u
   float64 y_v
   float64 z_w
   float64 k_p
   float64 m_q
   float64 n_r

   float64 x_uu
   float64 y_vv
   float64 z_ww
   float64 k_pp
   float64 m_qq
   float64 n_rr

   float64[6] current_velocity
   float64[48] thrust_configuration_matrix  # row-major 6x8, rows [Fx,Fy,Fz,Tx,Ty,Tz], columns thrusters 0..7

``SimManipulatorDynamics`` contains the manipulator model parameters used by
the simulated manipulator backend:

.. code-block:: text

   float64[4] link_masses
   float64[12] link_first_moments
   float64[24] link_inertias
   float64[4] viscous_friction
   float64[4] coulomb_friction
   float64[4] static_friction
   float64[4] stribeck_velocity
   float64[3] gravity_vector
   float64[3] payload_com
   float64 payload_mass
   float64[3] payload_inertia
   float64[6] base_pose
   float64[6] world_pose
   float64[6] tip_offset_pose

   float64 joint_lock_on_deadband
   float64 joint_lock_off_deadband
   float64 baumgarte_alpha
   float64 endeffector_mass
   float64 endeffector_damping
   float64 endeffector_stiffness

Planner Action
--------------

The planner action server is ``/planner`` and uses
``simlab_msgs/action/PlanVehicle``.

Goal:

.. code-block:: text

   float64[] start_xyz
   float64[] start_quat_wxyz
   float64[] goal_xyz
   float64[] goal_quat_wxyz
   string planner_name
   float64 time_limit
   float64 robot_collision_radius

Result:

.. code-block:: text

   bool success
   bool is_success
   float64[] xyz
   float64[] quat_wxyz
   int32 count
   float64 path_length_cost
   float64 geom_length
   string message

Feedback:

.. code-block:: text

   string stage

Supported planner names include ``Bitstar`` and ``RRTstar``. The interactive
controller wraps this action through ``PlannerActionClient`` and converts the
result into Ruckig trajectory execution.

State Topic
-----------

The central state topic is ``/dynamic_joint_states``. The stack reads:

- Manipulator position, velocity, acceleration, and effort for
  ``axis_e``, ``axis_d``, ``axis_c``, ``axis_b``, and grasper ``axis_a``.
- Vehicle NED pose: ``position.x``, ``position.y``, ``position.z``, ``roll``,
  ``pitch``, ``yaw``.
- Vehicle body velocity: ``u``, ``v``, ``w``, ``p``, ``q``, ``r``.
- Vehicle body acceleration: ``du``, ``dv``, ``dw``, ``dp``, ``dq``, ``dr``.
- Vehicle force/wrench state through the floating-base IO interfaces.

Inspect the live state stream with:

.. code-block:: shell

   ros2 topic echo /dynamic_joint_states

Mocap Interfaces
----------------

When launched with ``use_mocap:=true``:

- ``mocap4r2_optitrack_driver`` connects to the OptiTrack NatNet server.
- ``mocap_publisher`` converts rigid body poses into ROS pose/path topics.
- Main topics include ``/mocap_pose``, ``/mocap_path``, ``/map_mocap_pose``,
  and ``/map_mocap_path``.

The default launch value is ``use_mocap:=false`` so a missing OptiTrack server
does not spam startup logs.

Related Guides
--------------

- Use :doc:`controls_and_menus` for RViz menu, joystick, and task behavior.
- Use :doc:`replay_and_experiments` for command replay profiles and replay
  logging.
- Use :doc:`camera_and_perception` for sensor topics, camera launch options,
  and perception utilities.

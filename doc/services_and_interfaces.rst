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

Named dynamics profiles are documented in :doc:`replay_and_experiments`.
The ``Dynamics Profile`` RViz menu and command replay metadata use these
messages to apply complete robot parameter sets.

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

Simulated Camera Dynamics Service
---------------------------------

The simulator camera renderer exposes a typed camera profile service:

- ``/sim_camera_renderer_node/set_sim_camera_dynamics``
  (``ros2_control_blue_reach_5/srv/SetSimCameraDynamics``)

``SetSimCameraDynamics`` carries ``SimCameraDynamics``:

.. code-block:: text

   bool set_underwater_effect
   bool underwater_effect

   bool set_underwater_haze
   float64 underwater_haze

   bool set_underwater_tint
   float64 underwater_tint

   bool set_underwater_blur
   float64 underwater_blur

   bool set_underwater_noise
   float64 underwater_noise

   bool set_underwater_vignette
   float64 underwater_vignette

Each ``set_*`` flag controls whether the corresponding value is applied. The
same fields are also available as dynamic ROS parameters on
``/sim_camera_renderer_node``. SimLab dynamics profiles use this service for
camera-only and combined robot/camera profile updates.

Dynamic Obstacle Services
-------------------------

Dynamic obstacles are owned by the simulator. The low-level simulator service
sets the exact live obstacle snapshot:

- ``/dynamic_obstacle_sim_node/set_dynamic_obstacles``
  (``ros2_control_blue_reach_5/srv/SetDynamicObstacles``)

The request carries ``DynamicObstacleArray``:

.. code-block:: text

   std_msgs/Header header          # frame_id must match the simulator world frame
   DynamicObstacle[] obstacles

Each ``DynamicObstacle`` contains:

.. code-block:: text

   string id
   geometry_msgs/Pose pose
   geometry_msgs/Twist twist
   uint8 collision_type
   float64[] collision_dimensions
   uint8 visual_type
   float64[] visual_dimensions
   string visual_mesh_resource
   std_msgs/ColorRGBA color

Supported geometry types are ``sphere``, ``box``, ``cylinder``, and ``mesh``
through the message constants. Dimension conventions are:

- Sphere: ``[radius]``.
- Box: ``[x, y, z]``.
- Cylinder: ``[radius, height]``.
- Mesh: use ``visual_mesh_resource`` for rendering and collision dimensions as
  a simple proxy.

Obstacle IDs must be unique within each update. Empty IDs are normalized to
``obstacle_N`` by index. Duplicate effective IDs are rejected before the
snapshot is published.

The simulator publishes the accepted snapshot and RViz markers on:

- ``/dynamic_obstacles`` (``ros2_control_blue_reach_5/msg/DynamicObstacleArray``)
- ``/dynamic_obstacle_markers`` (``visualization_msgs/msg/MarkerArray``)

Static obstacles are published once after an update. Obstacles with nonzero
linear or angular velocity continue to update at the simulator obstacle publish
rate.

SimLab Backend API
------------------

Interactive mode exposes one backend API surface for RViz menus and non-RViz
clients. RViz menu callbacks call the same backend methods used by the ROS
services, so controller selection, planning, replay, grasper commands, dynamics
profile selection, and waypoint actions follow one behavior path.

The services live on the interactive controller node and use interfaces from
``simlab``:

- ``/backend/robot_command``
  (``simlab/srv/BackendRobotCommand``)
- ``/backend/world_command``
  (``simlab/srv/BackendWorldCommand``)
- ``/backend/pose_command``
  (``simlab/srv/BackendPoseCommand``)
- ``/backend/waypoint_command``
  (``simlab/srv/BackendWaypointCommand``)

``BackendRobotCommand`` covers robot-scoped actions such as selecting the active
robot, selecting a controller, selecting a planner, selecting a dynamics
profile, starting or stopping command replay, commanding the grasper, and
requesting Plan & Execute. Session recording is also exposed here so RViz and
headless clients use the same backend path.

``BackendWorldCommand`` covers world-level actions that are not robot-scoped.
The backend service is a convenience layer for named profiles; the simulator
service remains the source of truth for the live obstacle state.

Request fields:

.. code-block:: text

   string command
   string name
   int32 robot_index          # zero-based robot index; used by path-aware helpers

   bool enabled              # used by set_dynamic_replanning
   float64 rate              # Hz; <= 0 keeps current value
   float64 cooldown          # seconds; <= 0 keeps current value
   float64 lookahead_time    # seconds; <= 0 keeps current value
   float64 safety_margin     # meters; <= 0 keeps current value
   float64 replan_hysteresis # meters; <= 0 keeps current value

   float64 distance_ahead    # meters; used by spawn_path_obstacle
   float64 radius            # meters; used by spawn_path_obstacle

Supported ``BackendWorldCommand.command`` values:

- ``set_world_profile``: load the world profile named by ``name`` from
  ``simlab/resource/world_profiles`` and forward it to
  ``/dynamic_obstacle_sim_node/set_dynamic_obstacles``.
- ``clear_dynamic_obstacles``: clear all simulator dynamic obstacles.
- ``spawn_path_obstacle``: create one spherical obstacle on the active planned
  path for ``robot_index``. ``distance_ahead`` places it along the remaining
  path and ``radius`` sets the sphere radius. This is mainly for deterministic
  replanning tests and frontend-created benchmark scenarios.
- ``enable_dynamic_replanning``: enable the per-robot dynamic replanning
  supervisor. Optional numeric fields tune the monitor rate, cooldown,
  lookahead time, clearance margin, and hysteresis.
- ``disable_dynamic_replanning``: disable dynamic replanning and release its
  dynamic-obstacle subscription and timer.
- ``set_dynamic_replanning``: set ``enabled`` and optionally tune numeric
  dynamic replanning parameters in one request.
- ``dynamic_replanning_status``: return the current dynamic replanning state
  and tuning values, including per-robot replan count, last obstacle, last
  clearance, and last trigger reason.

Available world-profile examples include ``clear_world``,
``obstacle_crossing_sphere``, ``static_sphere_field``, and
``moving_box_corridor``.

Dynamic replanning is event-triggered. Each robot has its own replanning
supervisor, but the obstacle world is shared. The supervisor samples the
remaining active path over ``lookahead_time`` and asks the selected planner for
a replacement path when predicted clearance to a dynamic obstacle drops below
``safety_margin``. Replanning is non-destructive: the active trajectory remains
valid while the planner request is pending, and a failed replacement plan does
not erase the current trajectory. The replan request passes a nominal vehicle
speed so planners can time-index dynamic obstacle predictions while checking
candidate states. Repeated replans against the same unchanged blocked path are
suppressed by hysteresis; if an unresolved blocked path becomes imminent, the
robot stops the mission and holds its current state.

Request fields:

.. code-block:: text

   int32 robot_index       # zero-based robot index
   string command          # command name
   string name             # controller/planner/profile/grasper argument
   int32 index             # reserved for indexed commands
   float64 scalar          # scalar argument, for example IK weight
   float64[3] vector3      # vector argument, for example IK tool axis

Supported ``BackendRobotCommand.command`` values:

- ``select_robot``: select ``robot_index`` as the active robot.
- ``set_controller``: set controller named by ``name``.
- ``set_planner``: set planner named by ``name``.
- ``set_control_space``: set control space named by ``name``.
- ``set_dynamics_profile``: apply dynamics profile named by ``name``.
- ``plan_execute``: run Plan & Execute.
- ``reset_simulation``: reset the selected simulated robot.
- ``release_simulation``: release held commands after simulation reset and
  immediately hold the current vehicle pose with the selected feedback
  controller. This is the safe API default for headless/frontend clients.
- ``release_simulation_raw``: release the low-level simulator hold without
  installing a feedback hold target. This is intended for debugging only.
- ``hold_current_state`` or ``hold_current_vehicle_pose``: switch to feedback
  control and hold the current measured vehicle pose and arm state.
- ``release_and_hold``: install a feedback hold target and release the
  simulator if it is currently held.
- ``replay_select_profile``: select CmdReplay profile named by ``name``.
- ``replay_start``: reset and start CmdReplay. Requires CmdReplay and a
  selected replay profile.
- ``replay_stop``: stop CmdReplay.
- ``grasper``: use ``name: open`` or ``name: close``.
- ``set_ik_tool_axis``: set the task-space tool axis from ``vector3``.
- ``set_ik_base_align_weight``: set the IK base-alignment weight from
  ``scalar``.
- ``start_mcap_recording``: start the MCAP recorder.
- ``stop_mcap_recording``: stop the active MCAP recording.

``BackendPoseCommand`` carries a ``geometry_msgs/Pose`` for target updates and
waypoint creation.

Supported ``BackendPoseCommand.command`` values:

- ``set_vehicle_target``: set the vehicle planning target.
- ``set_task_target_world``: set the end-effector task target in world frame.
- ``set_task_target_arm_base``: set the end-effector task target in arm-base
  frame.
- ``add_waypoint``: add a vehicle waypoint. If ``use_current_target`` is
  ``true``, the current backend vehicle target is used. If it is ``false``,
  ``pose`` is copied into the vehicle target before adding the waypoint.
- ``reset_vehicle_world``: reset the simulated vehicle to ``pose`` expressed in
  the backend ``world`` frame. The backend converts the pose to the simulator
  vehicle/map NED convention, clears stale waypoint/path state, installs a
  feedback hold target at the requested pose, and releases the low-level
  simulator hold after reset. Frontends should use this command instead of
  calling ``/robot_N_reset_sim_uvms`` directly with world coordinates.

``BackendWaypointCommand`` manages vehicle waypoint missions: delete, clear,
stop, and execute.

Supported ``BackendWaypointCommand.command`` values:

- ``delete``: delete ``waypoint_index``.
- ``clear``: clear all waypoints for the robot.
- ``stop``: stop the active waypoint mission.
- ``execute``: execute the waypoint mission. If the robot is still held after a
  reset, the backend installs a feedback hold target, releases the simulator,
  and dispatches the waypoint mission from the release callback.

Examples:

.. code-block:: shell

   ros2 service call /backend/robot_command \
     simlab/srv/BackendRobotCommand \
     "{robot_index: 0, command: set_controller, name: PID}"

   ros2 service call /backend/robot_command \
     simlab/srv/BackendRobotCommand \
     "{robot_index: 0, command: plan_execute}"

   ros2 service call /backend/world_command \
     simlab/srv/BackendWorldCommand \
     "{command: set_world_profile, name: obstacle_crossing_sphere}"

   ros2 service call /backend/pose_command \
     simlab/srv/BackendPoseCommand \
     "{robot_index: 0, command: set_vehicle_target, pose: {position: {x: 1.0, y: 0.0, z: -1.0}, orientation: {w: 1.0}}}"

   ros2 service call /backend/pose_command \
     simlab/srv/BackendPoseCommand \
     "{robot_index: 0, command: reset_vehicle_world, pose: {position: {x: 0.0, y: 0.0, z: -2.0}, orientation: {w: 1.0}}}"

   ros2 service call /backend/waypoint_command \
     simlab/srv/BackendWaypointCommand \
     "{robot_index: 0, command: execute}"

   ros2 service call /backend/robot_command \
     simlab/srv/BackendRobotCommand \
     "{robot_index: 0, command: start_mcap_recording}"

These backend services are SimLab interfaces. Simulator-owned services remain
in ``ros2_control_blue_reach_5`` and cover reset, release, dynamics
parameters, simulated camera configuration, dynamic obstacles, and
``ros2_control`` hardware plugins.

Path-obstacle test flow
~~~~~~~~~~~~~~~~~~~~~~~

For a deterministic dynamic-obstacle/replanning check, start an interactive
launch, create a long vehicle target or waypoint mission, then place one or
more path obstacles through ``/backend/world_command``. The obstacle is placed
on the selected robot's active path, so the same command can be used from RViz,
scripts, or frontend clients.

.. code-block:: shell

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: clear_dynamic_obstacles}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: set_dynamic_replanning, enabled: true, rate: 5.0, cooldown: 0.5, lookahead_time: 8.0, safety_margin: 0.6, replan_hysteresis: 0.05}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: spawn_path_obstacle, name: blocker_1, robot_index: 0, distance_ahead: 4.0, radius: 0.75}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: dynamic_replanning_status}"

Expected behavior is that the obstacle appears in RViz, the active path is
checked against the updated obstacle set, the selected planner searches for a
replacement path, and the selected trajectory generator starts a replacement
trajectory if planning succeeds. A failed replacement plan leaves the current
trajectory intact until the conflict becomes imminent, at which point the
mission is stopped and the robot holds state.

Planner Action
--------------

The planner action server is ``/planner`` and uses
``simlab/action/PlanVehicle``.

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

Supported planner names include ``Bitstar``, ``RRTstar``, and ``RRTConnect``. The interactive
controller wraps this action through ``PlannerActionClient`` and converts the
result into vehicle trajectory-generator execution.

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

Controller Performance Topic
----------------------------

Each robot publishes a normalized controller-performance stream:

- ``/<prefix>/performance/controller``
  (``simlab/msg/ControllerPerformance``)

The topic is updated from the same command-publish loop that sends vehicle and
manipulator commands. It compares the measured robot state against the active
trajectory/reference commands. During vehicle planning, those references are
the active trajectory-generator samples. During replay in reference-tracking mode, they
come from the replay profile desired-state columns.

The raw vehicle path errors are:

- ``vehicle_cross_track_m`` or ``XTE``: cross-track error in meters. This is
  the component of vehicle position error perpendicular to the current
  trajectory direction.
- ``vehicle_along_track_m`` or ``ATE``: along-track error in meters. This is
  the component of vehicle position error parallel to the current trajectory
  direction.

The normalized vehicle metrics use the ``n`` prefix in overlays because they
are unitless:

- ``vehicle_n_cross_track`` or ``nXTE``:
  ``vehicle_cross_track_m / 0.4 m``.
- ``vehicle_n_along_track`` or ``nATE``:
  ``abs(vehicle_along_track_m) / max(target_speed * 1.0 s, 0.4 m)``.
- ``vehicle_n_position`` or ``nPos``: full 3D position-error norm divided by
  ``0.4 m``.
- ``vehicle_n_attitude`` or ``nAtt``: roll/pitch/yaw attitude-error norm
  divided by ``pi``.
- ``vehicle_n_linear_velocity`` or ``nVel``: body linear velocity-error norm
  divided by ``max(target_linear_speed, 0.1 m/s)``.
- ``vehicle_n_linear_acceleration`` or ``nAcc``: body linear
  acceleration-error norm divided by
  ``max(target_linear_acceleration_norm, 0.1 m/s^2)``.
- ``vehicle_n_angular_velocity`` and ``vehicle_n_angular_acceleration``:
  normalized body angular velocity and angular acceleration errors.

The normalized manipulator metrics are:

- ``arm_n_position``: RMS joint-position error divided by ``pi``.
- ``arm_n_velocity``: RMS joint-velocity error divided by
  ``max(reference_joint_velocity_rms, 0.1 rad/s)``.
- ``arm_n_acceleration``: RMS joint-acceleration error divided by
  ``max(reference_joint_acceleration_rms, 0.1 rad/s^2)``.

Aggregate behavior metrics are computed over the current active behavior
window:

- ``tracking_score``: per-sample RMS aggregate of ``nXTE``, ``nATE``,
  ``nAtt``, ``nVel``, ``nAcc``, ``arm_n_position``, ``arm_n_velocity``, and
  ``arm_n_acceleration``.
- ``tracking_score_rms``: RMS of ``tracking_score`` over the active behavior
  window.
- ``normalized_control_effort``: normalized vehicle wrench effort plus
  normalized manipulator effort.
- ``effort_per_tracking_score``: ``normalized_control_effort`` divided by
  ``tracking_score``.
- ``energy_per_meter`` and ``energy_per_second``: active-window control energy
  normalized by distance traveled and elapsed time.
- ``time_to_tolerance_sec``: seconds from behavior activation until
  ``tracking_score`` first reaches the configured tolerance. The value is
  ``-1`` until tolerance has been reached.
- ``peak_tracking_score``: largest ``tracking_score`` observed in the active
  behavior window.
- ``sample_count``: number of samples accumulated in the active behavior
  window.

Inspect one robot's live metric stream with:

.. code-block:: shell

   ros2 topic echo /robot_1_/performance/controller

Related Guides
--------------

- Use :doc:`controls_and_menus` for RViz menu, joystick, and task behavior.
- Use :doc:`replay_and_experiments` for command replay profiles and replay
  logging.
- Use :doc:`sensors_and_perception` for sensor topics, camera launch options,
  and perception-facing camera streams.

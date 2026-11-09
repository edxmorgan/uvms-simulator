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
- ``/robot_1_get_sim_uvms_dynamics``: read both active dynamics configurations.
- ``/robot_1_get_sim_vehicle_dynamics``: read the vehicle configuration.
- ``/robot_1_get_sim_manipulator_dynamics``: read the manipulator configuration.

These read-only endpoints use ``GetSimDynamics`` (empty request). The response
contains ``success``, ``message``, typed ``vehicle`` and ``manipulator``
messages, and per-component ``vehicle_valid`` / ``manipulator_valid`` and
``vehicle_use_coupled_dynamics`` / ``manipulator_use_coupled_dynamics`` flags.
An individual endpoint populates only its own component. Never interpret a
component's default-valued message when its validity flag is false.

Hardware plugins reconstruct snapshots from their active dynamics vectors and
runtime settings under their simulation-state mutex. Reads do not reset, hold,
release, or modify anything. The coordinator reads both components separately:
the combined response is **not an atomic snapshot**. It fails if either read
fails or the coupling modes disagree; successfully read components remain
marked valid for diagnosis.

For experiments, hold the robot, apply the profile, then read back and compare
all requested fields before starting a trial. Prevent other clients from
changing dynamics during this sequence and the trial. Save requested and
returned configurations separately, and verify controller nominal-model
configuration independently. Existing payload/gravity state telemetry remains
available; this service does not replace it. This snapshot describes configured
parameters, not a guarantee that every model mode uses every parameter.

``ResetSimUvms`` request fields:

.. code-block:: text

   bool reset_manipulator
   bool reset_vehicle
   bool hold_commands
   bool pause_dynamics      # opt-in; requires hold_commands=true

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
after reset until the matching release service is called. By itself this does
not freeze physics: gravity, buoyancy and other passive dynamics still act.

For deterministic experiment initialization, use both ``hold_commands=true``
and ``pause_dynamics=true``. The simulated vehicle and manipulator retain their
reset physical and estimator states until their matching release services are
called. State timestamps continue advancing; release resumes normal per-cycle
integration without integrating the elapsed pause duration. A pause without
command hold is rejected before applying the reset. The new flag defaults to
false, preserving ordinary startup and reset behavior; it is not enabled by
normal replay profiles. Rebuild service clients after updating this interface.

Use ``hold_commands=false`` when a replay/profile
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

Dynamic obstacles are owned by the SimLab source runtime. All edits use:

- ``/dynamic_obstacle_sim_node/edit_dynamic_obstacles``
  (``simlab/srv/EditDynamicObstacles``)

The request selects ``get``, ``add``, ``update``, ``remove``, ``clear``,
``replace``, or ``reset``, carries ``DynamicObstacleArray`` or IDs, and optionally
checks the expected scene revision. Responses contain the completed result and
authoritative scene. Creating an obstacle does not require a vehicle path.

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
   string collision_mesh_resource
   float64[3] collision_mesh_scale
   float64[3] visual_mesh_scale
   std_msgs/ColorRGBA color

Supported geometry types are ``sphere``, ``box``, ``cylinder``, and ``mesh``
through the message constants. Dimension conventions are:

- Sphere: ``[radius]``.
- Box: ``[x, y, z]``.
- Cylinder: ``[radius, height]``.
- Mesh: collision_mesh_resource and collision_mesh_scale define collision geometry;
  visual_mesh_resource and visual_mesh_scale define rendering.

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

   bool enabled              # used by set_dynamic_replanning
   float64 rate              # Hz; <= 0 keeps current value
   float64 cooldown          # seconds; <= 0 keeps current value
   float64 lookahead_time    # seconds; <= 0 keeps current value
   float64 safety_margin     # meters; <= 0 keeps current value
   float64 replan_hysteresis # meters; <= 0 keeps current value


Supported ``BackendWorldCommand.command`` values:

- ``set_world_profile``: load the world profile named by ``name`` from
  ``simlab/resource/world_profiles`` and forward it to
  ``/dynamic_obstacle_sim_node/edit_dynamic_obstacles`` using ``replace``.
- ``clear_dynamic_obstacles``: clear all simulator dynamic obstacles.
- ``enable_dynamic_replanning``: enable the per-robot dynamic replanning
  supervisor. Optional numeric fields tune the monitor rate, cooldown,
  lookahead time, clearance margin, and hysteresis.
- ``disable_dynamic_replanning``: disable the replanning supervisor timer;
  collision checking retains its dynamic-obstacle subscription.
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
  control and hold the current measured vehicle pose and arm state. The arm
  hold takes precedence over the background arm-marker IK timer until a new
  valid arm-base target is explicitly supplied. Marker keep-alives and menu
  events do not replace the hold target.
- ``release_and_hold``: install a feedback hold target and release the
  simulator if it is currently held.
- ``replay_select_profile``: select CmdReplay profile named by ``name``.
- ``replay_start``: reset and start CmdReplay. Requires CmdReplay and a
  selected replay profile.
- ``replay_stop``: stop CmdReplay.
- ``replay_status``: return schema-versioned JSON containing execution ID,
  lifecycle state, profile, feedback controller, subsystem modes, zero-based
  pass index, repeat count, elapsed/duration seconds, command-hold state and
  whether a reset response is pending. States include ``stopped``, ``resetting``,
  ``armed``, ``running``, ``complete``, ``error`` and ``no_csv``.
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

Replay execution identity
~~~~~~~~~~~~~~~~~~~~~~~~~

An accepted ``replay_start`` creates an execution ID and includes it in the
response message. Query ``replay_status`` for structured execution readback.
An accepted start is not a completed replay. Concurrent starts are rejected
while preparation/execution or a previous reset response is pending.

For a conditional stop, set ``BackendRobotCommand.name`` to the execution ID
when sending ``replay_stop``. A mismatched ID rejects the stop without affecting
the current run. An empty name stops the current run (the operator/menu behavior).
Stopping during reset prevents its late response from rearming playback; it
does not cancel the low-level physical reset. Wait for ``reset_pending=false``
before starting again. Stop still means zero replay commands, not feedback hold
or frozen physics. This status interface is not a trajectory action or a complete
experiment transaction/ownership protocol.

Controller model configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``/backend/controller_configuration`` uses
``simlab/srv/ControllerConfiguration``:

.. code-block:: text

   int32 robot_index
   string controller_name
   string operation          # get or set_model
   string model_profile      # named profile or absolute JSON path, set_model only
   ---
   bool success
   string message
   string configuration_json

``set_model`` configures the controller's estimated model, not the simulator
plant. It preserves gains, controller equations and internal actuator bounds.
It is allowed only for simulated robots that are inactive or command-held,
outside replay preparation/execution and outstanding replay resets. It does not
reset controller memory. ``get`` is read-only and remains available during execution.

PID, InvDyn, Ours and DR-Ours support the existing uncoupled arm/vehicle profile
layouts. Unsupported controller plugins reject configuration. Coupled profiles
and nonzero payload inertia are rejected because these controller model
signatures do not represent them.

The returned JSON has ``schema_version``, controller/implementation identity,
``model_profile_sha256`` and named effective ``parameters`` (model arrays,
gains/weights, limits and declared settings). An empty profile hash means no
profile was explicitly applied. The hash identifies the input profile; it is
not a source-code hash or a substitute for comparing effective parameter values.
DR readback also validates that its projectors consume the controller's residual
collectors. This endpoint provides controller readback, not plant readback.

.. code-block:: shell

   ros2 service call /backend/controller_configuration simlab/srv/ControllerConfiguration \
     "{robot_index: 0, controller_name: PID, operation: set_model, model_profile: dory_alpha}"

   ros2 service call /backend/robot_command simlab/srv/BackendRobotCommand \
     "{robot_index: 0, command: replay_status}"

Obstacle/replanning test flow
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For a dynamic-obstacle/replanning check, start an interactive launch and author
obstacles through ``/dynamic_obstacle_sim_node/edit_dynamic_obstacles`` (or its
source-owned RViz editor). Supply explicit poses and geometry; the source does
not require an active vehicle path. Read back the accepted scene and revision.
Then create a vehicle target or waypoint mission and enable replanning through
the separate backend world command interface.

.. code-block:: shell

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: clear_dynamic_obstacles}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: set_dynamic_replanning, enabled: true, rate: 5.0, cooldown: 0.5, lookahead_time: 8.0, safety_margin: 0.6, replan_hysteresis: 0.05}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: dynamic_replanning_status}"

Accepted obstacles appear in RViz and enter the shared collision world.
The selected planner searches for a replacement path, and the trajectory
generator's timed output is validated before execution. Urgent braking checks
take priority over replanning cooldown and suppression. A safety hold retains
the goal and waypoint progress. Simulation retries after clearance returns and
the vehicle settles; hardware requires operator control. Automatic retreat from
inside the clearance margin is not implemented.


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
   float64 dynamic_obstacle_prediction_speed # <= 0 uses current snapshot

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

Completed simulation steps and DR observations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The simulated arm GPIO (``<prefix>_arm_IOs``) and vehicle GPIO
(``<prefix>IOs``) export ``sim_step`` and ``sim_generation`` through the
existing ``dynamic_joint_states`` stream. These simulation-only counters are
independent for each subsystem. A reset increments the generation and sets the
step to zero; a completed integration increments the step. Paused dynamics do
not increment it. Existing ``sim_time`` remains the hardware timestamp, and
``sim_period`` is the integration duration, not the Python control-call period.

For a completed step, arm ``effort`` is the limited torque used for integration;
vehicle ``force.*``/``torque.*`` is the wrench passed to vehicle integration.
These fields describe the transition into the reported state. They are not
acknowledgements of every published command and do not include constraint
reaction forces or external contact forces.

The vehicle C++ EKF publishes additive body-FRD wrench estimates as
``estimated_disturbance.force.*`` / ``estimated_disturbance.torque.*``, diagonal
covariance as ``estimated_disturbance.variance.*``, and ``estimator_valid`` /
``estimator_step``. ``estimator_nominal_parameter.0`` through ``.32`` expose its
frozen 27 lumped coefficients followed by six nominal current components;
these are estimator settings, not simulator ground truth.

The simulated arm uses the same generated-CasADi/C++ architecture. Its
12-state augmented EKF estimates ``[q(4), dq(4), disturbance_torque(4)]`` in
joint order ``e, d, c, b``. On ``<prefix>_arm_IOs`` it publishes
``estimated_disturbance.torque.e`` through ``.b`` (Nm),
``estimated_disturbance.variance.torque.e`` through ``.b`` (Nm squared),
``estimator_valid`` and ``estimator_step``.
``estimator_nominal_parameter.0`` through ``.80`` expose the fixed ``dory_alpha``
nominal arm model (including gravity ``[0, 0, 9.81]``).
The default plant uses this same nominal gravity. Plant parameter changes do
not overwrite the observer's nominal model. The gripper retains its
acceleration filter. The vehicle and arm estimators currently use separate
dynamic models; this does not introduce vehicle–arm inertial coupling.

For unlocked-arm simulation experiments, pass ``arm_joint_locks:=false`` to
``robot_system_multi_interface.launch.py``. The default is ``true``.
This startup setting disables artificial lock hysteresis and forces the shared
plant/EKF lock mask to zero, including after resets or plant profile changes.
It does not remove gravity, friction, actuator limits, or joint position limits,
and is not passed to the physical manipulator plugin. Lock-free operation does
not guarantee observability when a joint is held against a position limit.

DR-Ours consumes each subsystem's native estimate only when valid, fresh (within 0.5 s),
and consistent with its nominal model. Missing/stale estimates or model mismatch
clear that subsystem's evidence; the nominal controller remains active.
Arm evidence is also cleared when any ``estimator_joint_locked.e/d/c/b`` flag
is one, or those flags are absent: a held load estimate is not fresh evidence.
In simulation DR-Ours starts with the same ``dory_alpha`` nominal profile as
the native observers. Explicit controller model changes remain subject to the
model-consistency check.
DR publishes ``/<prefix>/diagnostics/dr``
using ``diagnostic_msgs/msg/DiagnosticArray``, including the native variance.
EKF posterior means are correlated; empirical DR statistics are not certified
confidence bounds for the actual disturbance.

The arm estimator source and export cells are in
``Floating-KinDyn-Graph/usage/joint_ekf.ipynb``. They generate
``libmDisturbanceEKF_next.so``, installed from ``uvms-simulator/casadi_lib``.
The update signature follows ``diff_uv/usage/uv_ekf.ipynb``:
``(x, P, u_applied, p_nominal, dt, y, Q_d, R)``; the native lock-aware arm
entry point appends ``lock_mask, alpha, lower, upper, completed_contact``.
Bounds are read from the existing Xacro joint limits through the hardware
joint structures; there is no second limit configuration. Both estimators use analytic
Jacobians, a random-walk disturbance, and a Joseph covariance update.
The arm read loop consumes a completed integration step once, using its actual
limited actuator torque, duration, and the lock mask/gain used for that same
transition. Known lock reactions are predicted by the same constrained
mass-matrix solve as the plant. The Kalman-gain rows for locked disturbance
states are zero: means are held, and their variance grows by process noise,
including through lock transitions. Unlocking restores measurement updates.
The update selects its contact mode from the completed plant step. Contact
coordinates are censored measurements of the free endpoint: their measured
clamped states reset those coordinates with measurement covariance, but their
innovation rows are excluded from torque inference. Free-coordinate
innovations continue updating the full correlated disturbance posterior.
On release it retains free-motion sensitivities even if the prior nominal
prediction crosses a limit; otherwise the filter can latch at a stop and
never learn the disturbance that released it. The covariance retains
uncertainty in unresolved torque combinations rather than freezing every
load or arbitrarily capping uncertainty. All-contact steps have no torque
measurement update. This matches this simulator's end-of-step clamp model;
it is not a physical contact-force sensor or a guarantee for hardware impacts.
``estimator_joint_at_limit.e/d/c/b`` reports completed-step limit
contact; ``estimator_disturbance_observable.e/d/c/b`` reports eligibility for
full-vector use by the controller (not whether the EKF ran, nor a general
mathematical observability guarantee). DR continues rejecting constrained
full-vector evidence; a partial posterior is not an independently identified
four-dimensional torque measurement. PlotJuggler shows finite valid posterior
means and bands even when constrained, with a separate 0/1 arm-status plot.
``estimator_valid`` means the update is finite and synchronized, not that a
locked load is observable. Unknown loads and model mismatch remain estimated
when observable; no injected torque enters the observer input.
Its simulator noise settings are
``diag(Q_c)=[1e-8]*4+[1e-6]*4+[0.001]*4`` and
``diag(R)=[1e-6]*4+[1e-5]*4``; ``Q_d=Q_c*dt``.

Each robot publishes a normalized controller-performance stream:

- ``/<prefix>/performance/controller``
  (``simlab/msg/ControllerPerformance``)

The topic is updated from the same command-publish loop that sends vehicle and
manipulator commands. It compares the measured robot state against the active
trajectory/reference commands. During vehicle planning, those references are
the active trajectory-generator samples. During replay in reference-tracking mode, they
come from the replay profile desired-state columns.
The combined replay tracking metrics are inactive when either subsystem uses
command-only or zero-command playback: no complete tracking reference exists.

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

Hardware vehicle identification
-------------------------------

Vehicle hardware launches request MAVLink ATTITUDE (30) at 200 Hz and
SCALED_PRESSURE2 (137) at 10 Hz through MAVROS automatically. The
``bringup.mavros_telemetry`` helper retries failed requests, reapplies after
connection changes, and refreshes every 60 seconds. Simulation-only launches
do not start it. These are requested transmission rates, not guarantees of fresh
sensor sampling: the tested ArduSub 4.5.3 delivered about 166.7 Hz attitude and
10 Hz pressure. This does not increase the separate raw-acceleration rate.
No autopilot persistent parameters or actuator commands are changed.

Hardware pose feedback uses fresh valid DVL velocity whenever available,
including during manual operation. Loss of bottom lock (for example, in air)
omits the DVL measurement update; fresh IMU and pressure measurements continue
updating pose. Horizontal position is then unobserved and may drift. Valid
fresh DVL measurements restore full sensor availability automatically.
Disturbance and identification validity remain false while DVL is unavailable;
missing velocity is never treated as a measurement of zero.

The hardware EKF predicts at the control-loop rate and consumes each new sensor
reading once, independently of the other sensors. GPIO estimator validity
requires all three streams to be fresh, not to arrive in the same control cycle.
The hardware Xacro exposes dropout limits ``imu_timeout_s`` (0.5 seconds),
``pressure_timeout_s`` (1.5 seconds, three periods at the measured 2 Hz telemetry
rate), and ``dvl_timeout_s`` (0.5 seconds). These are availability checks, not
update rates. Following initialization, prediction and pose publication continue
during a dropout, but validity is false and missing IMU/pressure blocks commands.
Predicted pose during missing feedback must not be treated as measured position.
Raw identification endpoints retain their separate 50 ms alignment and maximum
interval checks; asynchronous EKF updates do not relax identification acceptance.

For the measured BlueOS TCP link, MAVROS timesync round trips were 12.6--22.8 ms.
Its default 10 ms acceptance limit rejected all those samples. A 30 ms limit
accepts that measured range while rejecting larger outliers. This permits up to
approximately 15 ms of path-asymmetry error and is not a precision timestamp
guarantee. Persist this on the BlueOS MAVROS configuration under
``/mavros/time: ros__parameters: max_rtt_sample: 30``; it is not an autopilot
sensor-rate setting. Verify ``/mavros/timesync_status`` after restarting MAVROS.

Manual and direct-thruster launches allow direct commands with fresh IMU and
pressure feedback even without DVL. This allowance does not change DVL sensor
selection. Feedback/replay operation still requires the full estimator validity
signal; loss of essential IMU or pressure feedback holds thrusters neutral.

Interactive hardware launch selects ``vehicle_identification_hardware.yaml``.
Edit that file's weight bounds, sensor-window size and validation scales, then
set ``confirm_hardware_thrust_calibration_zero_current_uncoupled: true`` only
after checking its stated assumptions. An explicit ``vehicle_identifier_config``
path overrides the default; ``off`` omits the node. MOSEK remains optional.

The hardware GPIO exports an ``identification.*`` snapshot on
``/dynamic_joint_states``: generation, step, time, period, valid, depth,
attitude.0..2, velocity.0..5 and wrench.0..5. Motion is raw body-NED sensor data,
not dynamics-EKF predictions. New snapshots require fresh IMU, pressure and
DVL readings within the existing 50 ms receipt-time skew gate. Receipt-time
alignment does not compensate transport latency or replace source-clock
synchronization. Invalid data and gaps invalidate the sample sequence.

The wrench is the interval average of the calibrated thrust-model output for
transmitted PWM commands. It is not measured thrust, requested force, or a
disturbance estimate. Thrust calibration errors, currents, external forces,
sensor noise and timing errors can bias the fitted parameters. A retained model
passing prediction checks does not establish unbiased parameter recovery.

The constant-inertia identifier uses momentum differences, without measured
acceleration. Each row spans two completed intervals (three sensor endpoints),
with quadratic quadrature of the state-dependent terms. It falls back to
composite trapezoids when the recorded wrench changes between intervals or
adjacent periods differ by more than a factor of two. Applied wrench is always
integrated using the recorded interval averages, not interpolated. This is a
finite-sample approximation, not exact agreement with the plant's RK4 steps.
Overlapping rows share measurement noise; the information screen is not a
confidence bound. Validation excludes rows whose earliest measurement overlaps
the fitting interval or the configured separation gap. Reset or missing steps
clear both endpoint history and the fitting window.

The same ``/vehicle_identification/<robot GPIO>/candidate`` diagnostic topic
provides candidate parameters, retained parameters and retained RMS to
PlotJuggler. Automatic application to control remains disabled. The horizon
counts fresh sensor snapshots, not controller cycles (300 samples at 5 Hz is
approximately 60 seconds). Hardware performance must be validated with recorded
sensor data and controlled trials before using estimates for control.

Related Guides
--------------

- Use :doc:`controls_and_menus` for RViz menu, joystick, and task behavior.
- Use :doc:`replay_and_experiments` for command replay profiles and replay
  logging.
- Use :doc:`sensors_and_perception` for sensor topics, camera launch options,
  and perception-facing camera streams.


Vehicle simulation measurement model
------------------------------------

Depth, delivered attitude and DVL velocity are sampled independently on the
completed-dynamics clock. ``sim_sensor_rates_hz`` is [depth, attitude, DVL],
defaulting to [10, 200, 10] Hz. These are configurable example rates, not measured
hardware delivery rates. Zero selects every completed step for legacy/ideal
experiments. A sensor can produce at most one observation per physics step;
sampling is quantized to the nearest completed step (timestamps remain the
actual completed time) and missed deadlines do not fabricate samples. A half-step
scheduling horizon prevents same-rate physics jitter from dropping IMU samples.
The EKF predicts every step and corrects only newly sampled channels, using the
existing exported prediction and masked correction functions. Held observations
are never repeatedly fused. Pausing dynamics stops the sensor clocks.

``sim_sensor_noise_std`` contains seven white-noise standard deviations: NED
depth (m), roll/pitch/yaw (rad), body-FRD velocity (m/s). The example default is
[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001], NOT a calibration of Bar30,
Navigator or A50. ``sim_dvl_covariance`` optionally replaces the velocity part
with a full positive-definite symmetric row-major 3x3 covariance in (m/s)^2.
Its lower Cholesky factor generates correlated velocity errors. Empty uses the
three velocity standard deviations. Neither setting configures the EKF.
Zero standard deviations with empty DVL covariance give ideal measurements;
zero rates select every-step sampling. EKF uncertainty remains independently
configured even with ideal measurements.

Noise is sampled in canonical estimator coordinates; the same measurements feed
sensor interfaces and the EKF. Physical state, TF truth and plant dynamics are
unchanged. Attitude noise is additive in Euler coordinates, away from pitch
singularities. This is an output-level AHRS model, not raw IMU integration.
Gyro and accelerometer measurements are sampled on the IMU clock and fused by
the exported nonlinear sensor correction. ``sim_raw_imu_noise_std`` is six
standard deviations: body gyro (rad/s) followed by specific force (m/s^2), with
example defaults [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]. These are not Navigator
calibration data. Set both noise vectors to zero for ideal measurements.
``imu_offset_body`` is the fixed IMU position in body FRD metres, default zero.
The model includes gravity, rotating-coordinate acceleration and lever-arm
angular/centripetal acceleration; ``sensor_gravity`` defaults to 9.80665 m/s^2.
The observer receives noisy specific force, never simulator acceleration truth.
Temporal correlation, sensor bias/drift, altitude-dependent bottom lock and
transport latency are not yet simulated. They require separate characterization;
manufacturer accuracy is not a random-noise standard deviation. The current
white-noise profile must not be described as calibrated hardware realism.

The simulation EKF uses independent, positive measurement-uncertainty settings:

* ``depth_attitude_measurement_variances``: four variances for depth (m^2),
  roll/pitch/yaw (rad^2), default [1e-6, 1e-6, 1e-6, 1e-6].
* ``dvl_measurement_covariance``: positive-definite symmetric row-major 3x3
  body-FRD velocity covariance ((m/s)^2), default 1e-6 times identity.
* ``raw_imu_measurement_variances``: six variances, body gyro ((rad/s)^2)
  followed by specific force ((m/s^2)^2), default
  [1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4].

These are fixed prior assumptions, not online estimates or privileged access to
injected noise. Defaults preserve the previous default EKF tuning, but changing
any ``sim_*`` noise setting no longer changes R. Existing custom profiles that
deliberately require matched noise must now set both sides explicitly.
The sensor-noise menu also leaves these assumptions unchanged. Hardware already
uses configured measurement variances and received DVL covariance; this change
does not alter its behavior.

``sim_sensor_noise_seed`` is an unsigned 32-bit seed, defaulting to a stable hash
of the robot prefix. Vehicle reset restarts noise and sampling clocks; retained
model replacement does not. Exact random-sequence reproducibility assumes the
same standard library, physics steps, seed and sensor configuration.

Use the existing launch argument
``sim_vehicle_sensor_config:=/absolute/path/sensors.yaml``, for example::

  /robot_1_BlueSystemMultiInterface_topics_interface:
    ros__parameters:
      sim_sensor_noise_enabled: true  # Default; set false for ideal measurements.
      sim_sensor_noise_seed: 42
      sim_sensor_rates_hz: [10.0, 200.0, 10.0]
      sim_sensor_noise_std: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
      sim_dvl_covariance: []
      sim_raw_imu_noise_std: [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]
      # Independent EKF assumptions (variances, not standard deviations):
      depth_attitude_measurement_variances: [0.000001, 0.000001, 0.000001, 0.000001]
      dvl_measurement_covariance: [0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001]
      raw_imu_measurement_variances: [0.000001, 0.000001, 0.000001, 0.0001, 0.0001, 0.0001]

All settings are read-only startup parameters. ``ekf_navigation_process_intensities``
(12 values) and ``ekf_wrench_process_intensities`` (6 values) remain configurable
in squared state units per second. Wrench defaults remain
[0.0007, 0.0007, 0.0007, 0.0007, 0.0007, 0.0007], in N^2/s then (N m)^2/s.
An evaluated faster profile [1, 1, 1, 0.1, 0.1, 0.1] is deliberately opt-in.
Faster tracking trades against
noise at small/zero loads; these settings are engineering assumptions, not
sensor accuracy specifications or guaranteed disturbance recovery.

Continuous process covariance is propagated along the same nominal dynamics as
the mean prediction using the exported ``disturbance_ekf_predict_covariance``.
Three-point quadrature accounts for noise introduced during the interval;
intervals longer than 20 ms are subdivided without fabricating new measurements.
This preserves the configured intensity and changes its discretization, not the
sensor data or load assumptions. It does not guarantee equal force/torque
tracking speeds or eliminate the noise/bandwidth tradeoff. The original
``disturbance_ekf_predict`` and discrete-Q update exports remain available.

Standard MCAP ``dynamic_joint_states`` recordings include truth, held measurements,
``sensor_new.0`` through ``sensor_new.12`` and ``sensor_sample_time.0`` through
``sensor_sample_time.2``. Sample times are seconds of completed dynamics since
vehicle reset; pair them with ``sim_generation`` and ``sim_step``. The optional
EKF evaluation trace is labeled ``vehicle_ekf_trace_v4_integrated_process_noise``.
Its Q slot contains the integrated discrete process covariance, including
motion/wrench cross terms; it is no longer just continuous intensity times dt.
Its measurement,
innovation and measurement-covariance dimensions are now 13; the suffix contains
13 update flags, three sample times, the local dynamics clock, three IMU offset
coordinates and gravity. Flags 7..9 are gyro and 10..12 specific force.
For exact replay, compose mean and transition over the same equal prediction
substeps: ``ceil(dt / 0.02)`` steps of ``dt / steps``, then add the recorded
total Qd. A single full-interval predictor call is not the same numerical
integration when dt exceeds 20 ms.
Old trace replayers must reject this new schema until updated; they must not
interpret it as a seven-measurement trace or repeatedly fuse held observations.
No independent recorder is added.

The optional simulation parameter identifier still requires adjacent fresh
physics-step measurements. It now refuses held multirate observations instead
of fitting them as fresh data. Its existing experiments can explicitly use
``sim_sensor_rates_hz: [0.0, 0.0, 0.0]``; that is a synchronous simulation
experiment, not realistic A50 cadence. Supporting interval identification with
asynchronous endpoints requires a separate input-integration adapter. Retained
model application itself is unchanged.

Hardware vehicle sensor settings
--------------------------------

A50 packets provide a full velocity covariance in (m/s)^2. The hardware EKF now
uses that matrix, including cross terms. Nonfinite, nonsymmetric, zero or
non-positive-definite matrices invalidate that velocity observation; there is no
silent fixed-covariance fallback. The existing installation contract requires
A50 velocity axes aligned with body FRD. Different mounting requires transforming
both velocity and covariance and accounting for the measurement point.

Bar30 depth conversion uses ``surface_pressure_pa`` and the configured water
density. Set this reference to the average absolute pressure measured at the
surface. Default 101325 Pa retains compatibility and logs an uncalibrated-reference
warning. Startup does not auto-zero: the vehicle could already be submerged.
Changing the reference changes the depth datum and requires a controlled restart.

``depth_attitude_measurement_variances`` is [depth, roll, pitch, yaw], in m^2 and
rad^2. Its provisional defaults [0.01, 0.005, 0.005, 0.005] are unchanged and need
calibration of the delivered Bar30/AHRS output. MAVROS orientation covariance
may itself be a configured constant; raw gyro noise density cannot be substituted
for attitude variance. Use
``hardware_vehicle_sensor_config:=/absolute/path/hardware-sensors.yaml``::

  /robot_1_BlueSystemMultiInterface_topics_interface:
    ros__parameters:
      surface_pressure_pa: 101325.0  # REPLACE with measured surface pressure
      depth_attitude_measurement_variances: [0.01, 0.005, 0.005, 0.005]

This config is rejected for a simulated vehicle, and the simulation sensor config
is rejected for a hardware vehicle. No hardware actuation is needed to test these
measurement-conversion and covariance paths.

The hardware raw IMU input is ``/mavros/imu/data_raw``, separate from delivered
attitude on ``/mavros/imu/data``. MAVROS caches accelerometer readings in attitude
messages; using that cached field for every attitude packet would fuse duplicate
observations. Raw packets require a strictly increasing timestamp, a recognised
body FLU frame and finite values. FRD conversion precedes bias subtraction.
Missing/unavailable channels are masked, never replaced with zero observations.
``raw_imu_bias_body`` is [gyro, specific force] in FRD, default zero;
``raw_imu_measurement_variances`` defaults to [1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4].
Calibrate these, the IMU offset and the AHRS covariance before hardware use.
MAVROS covariance constants are not measured calibration. AHRS/raw-IMU
correlation and unknown bias drift are not estimated by this model.
Both raw and attitude IMU topics are included in the standard MCAP topic list.

Correlated wrench uncertainty
-----------------------------

``ekf_wrench_process_covariance`` optionally supplies a full symmetric positive-
semidefinite 6x6 continuous covariance, row-major in body FRD [force xyz, torque
xyz]. Empty preserves ``ekf_wrench_process_intensities``. The full matrix replaces
that diagonal block, and is multiplied by the completed prediction interval once.
It configures a stochastic load model; it is not a measured or injected wrench.
Cross terms have mixed N*(N m)/s units. For a known fixed load-reference mapping
``tau_body = B * wrench_reference``, use ``Q_body = B * Q_reference * B.T``.
A tool-load prior is appropriate only when that assumption is intended. A matrix
computed for a held arm is not an automatic moving-tool prior. Arbitrary vehicle
loads must not silently inherit a tool-specific covariance. Defaults are unchanged.

Both vehicle drivers expose ``estimated_disturbance.covariance.i.j`` for i,j=0..5,
including force/torque cross terms, through the standard dynamic-joint-state/MCAP
pipeline. Task-wrench inference whitens with this full covariance before its SVD.
With six vehicle observations this preserves the point estimate and corrects its
uncertainty. With redundant observable arm rows it also affects weighting. Arm
covariance is still diagonal and vehicle/arm cross-covariance is unavailable;
the published status explicitly identifies that approximation. Older recordings
without full vehicle covariance use an explicitly labeled legacy diagonal fallback;
a partial or malformed full covariance is rejected instead of silently downgraded.

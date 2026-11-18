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

Readback uses ``GetSimDynamics``. Check ``success`` and each component's
``*_valid`` flag before using its configuration. The combined service reads
both components separately; it is not an atomic snapshot.

``ResetSimUvms`` selects the subsystems to reset, optional initial states, and
optional dynamics parameters. Vehicle pose is NED ``[x,y,z,roll,pitch,yaw]``;
twist and wrench are body-frame ``[u,v,w,p,q,r]`` and ``[Fx,Fy,Fz,Tx,Ty,Tz]``.
Manipulator state follows joint order e, d, c, b, a.

Inspect the installed request definition::

   ros2 interface show ros2_control_blue_reach_5/srv/ResetSimUvms

``hold_commands`` keeps simulated manipulator and vehicle commands blocked
after reset until the matching release service is called. By itself this does
not freeze physics: gravity, buoyancy and other passive dynamics still act.

Set ``pause_dynamics=true`` with ``hold_commands=true`` to freeze integration
until release. State timestamps continue updating during the pause. Interactive
simulation startup and menu resets use this pause; an accepted execution releases
it. A pause without command hold is rejected.

Use ``hold_commands=false`` to reset and continue immediately.

``use_coupled_dynamics`` tells the backend how to interpret the supplied
vehicle and manipulator parameter sets. Use ``false`` for independent vehicle
and manipulator subsystem parameters. Use ``true`` when the parameters belong
to one coupled UVMS dynamics model.

``SetSimDynamics`` selects vehicle and/or manipulator parameter updates.
``GetSimDynamics`` reads the effective parameters back. Named profiles are
covered in :doc:`replay_and_experiments`.

For field names and array sizes, inspect the installed definitions::

   ros2 interface show ros2_control_blue_reach_5/srv/SetSimDynamics
   ros2 interface show ros2_control_blue_reach_5/msg/SimVehicleDynamics
   ros2 interface show ros2_control_blue_reach_5/msg/SimManipulatorDynamics

Vehicle parameters cover inertia, restoring forces, drag, current and thruster
allocation. The thrust matrix is row-major 6×8, with wrench rows and thruster
columns. Manipulator parameters cover link inertia, friction, gravity, payload,
transforms and artificial joint locks.

Simulated Camera Dynamics Service
---------------------------------

The simulator camera renderer exposes a typed camera profile service:

- ``/sim_camera_renderer_node/set_sim_camera_dynamics``
  (``ros2_control_blue_reach_5/srv/SetSimCameraDynamics``)

``SetSimCameraDynamics`` carries underwater-effect enable, haze, tint, blur,
noise and vignette settings. Each value has a corresponding ``set_*`` flag.
The same settings are runtime parameters on ``/sim_camera_renderer_node``;
see :doc:`sensors_and_perception` for examples.

Dynamic Obstacle Services
-------------------------

Dynamic obstacles are owned by the SimLab source runtime. All edits use:

- ``/dynamic_obstacle_sim_node/edit_dynamic_obstacles``
  (``simlab/srv/EditDynamicObstacles``)

The request selects ``get``, ``add``, ``update``, ``remove``, ``clear``,
``replace``, or ``reset``, carries ``DynamicObstacleArray`` or IDs, and optionally
checks the expected scene revision. Responses contain the completed result and
authoritative scene. Creating an obstacle does not require a vehicle path.

``DynamicObstacle`` carries an ID, pose, twist, visual and collision geometry,
mesh resources, scales and color. Inspect the message with::

   ros2 interface show ros2_control_blue_reach_5/msg/DynamicObstacle

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

RViz and scripts use the same backend operations. The main services are:

- ``/backend/robot_command`` — ``simlab/srv/BackendRobotCommand``:
  controllers, planning, replay, reset, grasper and recording.
- ``/backend/world_command`` — ``simlab/srv/BackendWorldCommand``:
  world profiles and dynamic replanning.
- ``/backend/pose_command`` — ``simlab/srv/BackendPoseCommand``:
  vehicle and end-effector targets.
- ``/backend/waypoint_command`` — ``simlab/srv/BackendWaypointCommand``:
  waypoint missions.

World commands
~~~~~~~~~~~~~~

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

Dynamic replanning requests replacement paths without interrupting the current
trajectory. Generated replacements are not collision-checked again before activation. It does
not perform automatic collision holds or recovery. See the example below.

Robot commands
~~~~~~~~~~~~~~

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
- ``stop_execution``: cancel execution and stop command output.
- ``reset_simulation``: reset the selected simulated robot.
- ``release_simulation``: release held commands after simulation reset and
  immediately hold the current vehicle pose with the selected feedback
  controller. This is the safe API default for headless/frontend clients.
- ``release_simulation_raw``: release the low-level simulator hold without
  installing a feedback hold target. This is intended for debugging only.
- ``hold_current_state`` or ``hold_current_vehicle_pose``: hold the current
  estimated vehicle pose and arm state until an explicit new target.
- ``release_and_hold``: install a feedback hold target and release the
  simulator if it is currently held.
- ``replay_select_profile``: select CmdReplay profile named by ``name``.
- ``replay_start``: reset and start CmdReplay. Requires CmdReplay and a
  selected replay profile.
- ``replay_stop``: stop CmdReplay.
- ``replay_status``: report execution ID, state, profile, pass count, timing
  and pending reset status.
- ``grasper``: use ``name: open`` or ``name: close``.
- ``set_ik_tool_axis``: set the task-space tool axis from ``vector3``.
- ``set_ik_base_align_weight``: set the IK base-alignment weight from
  ``scalar``.
- ``start_mcap_recording``: start the MCAP recorder.
- ``stop_mcap_recording``: stop the active MCAP recording.

Pose and waypoint commands
~~~~~~~~~~~~~~~~~~~~~~~~~~

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
- ``execute``: execute the waypoint mission.

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

An accepted service request may start asynchronous work. Use planner results,
replay status or scene status to determine completion.

Replay execution identity
~~~~~~~~~~~~~~~~~~~~~~~~~

``replay_start`` returns an execution ID; ``replay_status`` reports progress.
For a conditional stop, pass that ID in ``BackendRobotCommand.name`` with
``replay_stop``. An empty name stops the current execution. After stopping during
a reset, wait for ``reset_pending=false`` before starting again.

Controller model configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``/backend/controller_configuration`` uses
``simlab/srv/ControllerConfiguration``:

.. code-block:: text

   int32 robot_index
   string controller_name
   string operation          # get, set_model, or set_gains
   string model_profile      # named profile or absolute JSON path, set_model only
   string gains_json         # JSON object, set_gains only
   ---
   bool success
   string message
   string configuration_json

``set_model`` configures the controller's estimated model, not the simulator
plant. It preserves gains, controller equations and internal actuator bounds.
It is allowed only for simulated robots that are inactive or command-held,
outside replay preparation/execution and outstanding replay resets. It does not
reset controller memory. ``get`` is read-only and remains available during execution.

``set_gains`` accepts controller-specific arrays in ``gains_json``. PID/InvDyn
use ``kp``, ``ki``, ``kd`` and their ``arm_`` equivalents. SMC uses ``vehicle_``
or ``arm_`` followed by ``surface``, ``damping``, ``switching`` or ``boundary``.
Changes require an inactive or reset-held simulation with replay stopped and
reset controller memory. ``get`` returns the effective configuration.

Unsupported controller/model combinations are rejected. ``configuration_json``
reports the implementation, profile hash and effective parameters. Compare these
values independently of the plant configuration returned by ``GetSimDynamics``.

.. code-block:: shell

   ros2 service call /backend/controller_configuration simlab/srv/ControllerConfiguration \
     "{robot_index: 0, controller_name: PID, operation: set_model, model_profile: dory_alpha}"


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
The selected planner searches for a collision-free replacement path. The
generated trajectory is not collision-checked again before execution. Cooldown and hysteresis
limit repeated planning requests. A rejected replacement leaves the previous
reference active; no automatic collision hold or retreat is performed.


Planner Action
--------------

The ``/planner`` action uses ``simlab/action/PlanVehicle``::

   ros2 interface show simlab/action/PlanVehicle

Supply world-frame start/goal positions, quaternions in wxyz order, planner name,
time limit and collision radius. ``dynamic_obstacle_prediction_speed > 0`` enables
time-indexed obstacle predictions; otherwise collision checks use the snapshot.

Results contain success, path positions/orientations, waypoint count and length.
Feedback reports the planning stage. Built-in planners are ``Bitstar``,
``RRTstar`` and ``RRTConnect``. SimLab validates the result before trajectory
execution; planner success alone does not mean the robot has started moving.

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

Simulation and estimator state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The arm and vehicle GPIOs publish ``sim_generation``, ``sim_step`` and
``sim_period``. Reset starts a new generation; only completed integration
advances the step. Applied effort describes the transition into the reported
state, excluding external loads and constraint reactions.

``estimated_disturbance.*`` contains external torque (arm) or body-FRD wrench
(vehicle), with variance and full covariance fields. Check ``estimator_valid``
and freshness before use. For the arm, also check
``estimator_disturbance_observable.e/d/c/b``: locked or constrained joints do not
provide independently observable external torque. Numerical validity alone is
not observability. The gripper has no disturbance estimate.

Use estimated states for feedback and physical states for evaluation. Both are
recorded. Sensor settings are in :doc:`sensors_and_perception`.

Controller Performance Topic
----------------------------


``/<prefix>/performance/controller`` (``simlab/msg/ControllerPerformance``)
compares feedback with the active trajectory or replay reference. Combined
tracking metrics are inactive during command-only or zero-command playback.

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

Vehicle identification
----------------------

Build both the simulator and SimLab with a working MOSEK SDK/license::

   colcon build --packages-select ros2_control_blue_reach_5 simlab \
     --cmake-args -DUVMS_BUILD_VEHICLE_IDENTIFIER=ON -DUVMS_BUILD_ARM_IDENTIFIER=ON \
     -DUVMS_MOSEK_PLATFORM="$HOME/mosek/11.0/tools/platform/linux64x86"
   source install/setup.bash

Interactive simulation then starts identification with the normal sensor rates;
no separate sensor YAML is needed. Use submerged, uncoupled, zero-current motion
with effort control. Vary translation and rotation; hovering cannot identify a
full model. Initial collection takes about 34 seconds at 10 Hz, followed by
validation on new measurements.

Read the short status::

   ros2 topic echo /vehicle_identification/robot_1_IOs/summary --field data

Run the automated solver and noisy CPU-plant qualification from the workspace::

   ./build/ros2_control_blue_reach_5/test_identification_solver

``/vehicle_identification/<robot GPIO>/candidate`` contains detailed results.
Only validated retained models feed the simulation EKF; controller and plant
parameters are unchanged. Set ``vehicle_identifier_config:=off`` to disable it.

Hardware uses ``vehicle_identification_hardware.yaml``. Confirm thrust
calibration and model assumptions in that configuration before enabling it.
Hardware models are observation-only and require independent validation before
use in control.


Manipulator identification
--------------------------

The build above enables both identifiers. Interactive mode starts arm
identification automatically; use ``arm_identifier_config:=off`` to disable it.

Hardware arm identification is observation-only: it uses the same 48-parameter
model and validation, but does not update the hardware EKF or controller.
It interpolates serial feedback to a common receive-time watermark and integrates
current-derived torque. This torque is not a calibrated torque-sensor reading.
Use a fixed base, no payload or external contact, and verify the current-to-torque
calibration. Air measurements do not qualify a model for underwater operation.
The hardware gripper is excluded pending current/force calibration.

Launch the physical arm with a simulated vehicle::

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
     use_vehicle_hardware:=false use_manipulator_hardware:=true \
     sim_robot_count:=0 task:=interactive
   ros2 topic echo /arm_identification/robot_real__arm_IOs/summary --field data

The ``hardware_context`` identifier parameter is gravity (3), base pose (6),
and world pose (6). Its default matches the simulation model; set it in a custom
``arm_identifier_config`` YAML if the physical mounting differs.

Use uncoupled simulation with zero payload and no injected loads or Stribeck
friction. Move all four arm joints through different configurations, speeds and
both directions. Avoid joint stops; locked-joint measurements are excluded.
Arm windows containing speeds below ``min_sliding_speed`` (default 0.005 rad/s)
are excluded: near-stationary Coulomb friction cannot be inferred from velocity
sign. Set this speed above the measurement resolution. Increase actual joint motion if collection stalls; changing targets alone
is not sufficient.
Move the gripper separately to identify its mass, damping and stiffness.
Collection takes at least 32 seconds, then validation requires new varied motion.

Read the status::

   ros2 topic echo /arm_identification/robot_1__arm_IOs/summary --field data

The ``/arm`` and ``/gripper`` topics under that prefix contain candidates,
retained models, excitation rank and independent prediction errors.
``needs varied joint motion`` means the available data cannot resolve the model;
more stationary samples will not help. A rejected replacement preserves the
previous validated model unless later measurements repeatedly contradict it.

Validated arm and gripper models update their simulation EKFs. The summary
shows ``new model pending acknowledgement`` until the estimator confirms the
handover, then ``identified model applied``. Controller and plant parameters
are unchanged. Reset, invalidation or loss of identifier updates restores the
default estimator. Position/velocity estimates and their covariance are preserved
at handover; disturbance estimates are reinitialized for the new model.

Link parameters are physically constrained representatives, not unique recovered
masses or inertias. Acceptance checks later torque prediction, with default RMS
limits of 0.03 Nm per arm joint and 0.1 in the gripper model's effort units.

The ``start``, ``stop`` and ``reset`` Trigger services share that prefix.
Stop/reset clear identification without stopping robot motion; call start to
collect again. Run the plant and validation tests with::

   ./build/ros2_control_blue_reach_5/test_arm_identification


Related Guides
--------------

- Use :doc:`controls_and_menus` for RViz menu, joystick, and task behavior.
- Use :doc:`replay_and_experiments` for command replay profiles and replay
  logging.
- Use :doc:`sensors_and_perception` for sensor topics, camera launch options,
  and perception-facing camera streams.

Disturbance interfaces
----------------------

Each simulated robot exposes:

- ``/<prefix>/disturbance/run``: ``simlab/action/RunDisturbance``.
- ``/<prefix>/disturbance/stop``: ``std_srvs/srv/Trigger``.
- ``/<prefix>/disturbance/status``: accepted configuration and run status.
- ``/<prefix>/disturbance/command``: simulation load transport.
- ``/<prefix>/disturbance/{vehicle,arm}/state``: applied loads and plant-step times.
- ``/<prefix>/disturbance/task_wrench`` and ``task_estimate``: requested and
  estimated tool wrench.

Use the action client in :doc:`replay_and_experiments`. Receivers enforce one
session owner and expire unrenewed commands after one second. Overlapping runs are rejected.

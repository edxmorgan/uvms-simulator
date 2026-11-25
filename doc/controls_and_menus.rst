Controls, Menus, and Teleoperation
==================================

The operator-facing launch tasks are:

- ``interactive``: RViz menus, interactive markers, planning, replay,
  waypoint execution, grasper commands, and optional manual joystick command
  input.
- ``manual``: PS4 joystick direct command teleoperation.
- ``direct_thrusters``: keyboard PWM testing for individual thruster channels.

Interactive Task
----------------

Launch as described in :doc:`userdoc`.

Select the active robot once from ``Robots``. Tool menus such as ``Path
Planner``, ``Cmd Replay``, ``Robot Control → Grasper``, ``Waypoints``, and ``Plan & Execute``
then operate on that selected robot.

Main menu groups:

- ``Plan & Execute``: activate the currently selected feedback controller if it
  is inactive, then plan and execute the selected target or waypoint mission.
- ``Stop Execution``: cancel execution and stop command output; already-running
  simulation physics continues. Use a feedback hold when you want to maintain pose.
- ``Robots``: select active robot when multiple simulated robots exist.
- ``Waypoints``: add, delete, clear, or stop vehicle waypoint missions.
- ``Path Planner``: select the active robot's planner backend, currently
  ``Bitstar``, ``RRTstar``, or ``RRTConnect``.
- ``Cmd Replay``: select the active robot's replay profile, reset/play, and
  stop replay. See :doc:`replay_and_experiments` for profile format and
  experiment logging.
- ``Dynamics Profile``: apply a named robot dynamics parameter profile during
  simulation.
- ``Data Recording``: start or stop MCAP recording for the configured robot,
  reference, performance, camera, and dynamic obstacle topics.
- ``Robot Control → Grasper``: open/close the active robot's grasper through feedback
  controllers only.
- ``Reset Manager``: simulator state reset and release controls. Reset clears
  the selected robot's waypoint/path state and the global dynamic obstacle set.
  Requests for hardware namespaces are rejected with a log message.
- ``Robot Control``: select controller, control space, and IK settings for the
  active robot.
- ``Environment``: switch static scenes in simulation; see :doc:`environment_scenes`.

Controller tuning
-----------------

Open ``Robot Control → Controller → Tune controller…``, or run
``ros2 run simlab controller_tuning``. Select a robot, controller and component.
Edit the numeric arrays and click ``Apply`` during execution; no stop or rebuild
is needed. The panel verifies the effective values and shows which controller is
actually selected for execution. Editing another controller does not select it
or start motion.

PID, InvDyn and SMC declare their editable parameters in the controller API.
When the private Namor dependency is available, OGES (``Ours``) and DR-OGES
(``DR-Ours``) also appear in the same editor with weights, scales and gripper gains.
Weights use a table with one row per joint/vehicle axis and columns for a1, a2,
a12 (cross) and ac; each cell edits one native weight without reordering the vector.
The editor uses that schema, including array dimensions, rather than assuming
PID gains. Updates are validated as a whole between control callbacks. For PID
and InvDyn, changing Ki resets only the changed integral axes; other gain edits
preserve accumulated error. Scene/reset/release transitions must finish before
applying parameters. Model and actuator limits are not tuning fields.

``Undo`` restores the last applied values for that robot/controller/component.
``Refresh`` reads effective values. ``Save applied`` saves all declared parameters
of the chosen controller to YAML, excluding unsent edits. ``Load YAML`` fills the
selected component's editor; click ``Apply`` to send it.

Startup defaults have a single source:
``simlab/resource/controller_profiles/defaults.yaml``. This contains the common
nominal model, limits and home poses, plus simulation/hardware controller gains.
The loader reads it once and gives each controller independent arrays. These
nominal controller models are separate from the simulation plant profiles.

A saved tuning YAML can also be loaded at startup using the interactive
controller's ROS string parameter ``<robot_prefix><controller_lowercase>_profile``;
for example ``robot_real_pid_profile: /absolute/path/pid.yaml`` or
``robot_1_smc_profile: /absolute/path/smc.yaml``. The same validation is used at
startup and during live tuning. Without this parameter, YAML defaults apply.

Arm arrays follow axis_e, axis_d, axis_c, axis_b, axis_a (grasper); vehicle arrays
follow surge, sway, heave, roll, pitch, yaw. Use PlotJuggler to compare tracking
error and effort during each trial.

Use the obstacle marker's menu to edit obstacles. Scripts use
``/dynamic_obstacle_sim_node/edit_dynamic_obstacles``; no active path is required.

Feedback Control and Replay
---------------------------

Select a feedback controller under **Robot Control**, then use **Plan & Execute**
for targets and waypoints. Selecting a controller alone does not start motion.

Task-space moves require a clear direct vehicle translation before execution.
Use vehicle planning to move around obstacles.
If a start is rejected because its planning sphere overlaps an obstacle, use
**Reset Manager** to reset the simulation to a clear pose, then plan again.
Changing the goal or planner does not repair an obstructed start.

For CSV playback, select ``CmdReplay`` and a profile, then start from **Cmd Replay**.
The profile selects command replay, reference tracking, hold or zero output for
each subsystem. See :doc:`replay_and_experiments` for profile configuration.

Controller notes:

- ``PID`` combines feedback terms with hydrostatic restoring compensation on
  the vehicle side. The manipulator side is joint-space feedback with
  configured command limits.
- ``InvDyn`` implements inverse-dynamics control in the computed-torque sense:
  desired state and desired acceleration are mapped through an estimated model
  to actuation, with feedback terms correcting model and state-estimation
  errors. The manipulator model comes from ``Floating-KinDyn-Graph``; the
  vehicle model comes from ``diff_uv``.

``SMC`` is also available through the controller menu. Its current gains are
for simulation. Configure gains through ``/backend/controller_configuration``;
see :doc:`services_and_interfaces`. Switching feedback controllers preserves
an active plan but can change commanded effort.

Waypoint Menu
-------------

The waypoint menu operates on vehicle waypoint missions:

- ``Add``: append the current target pose as a vehicle waypoint.
- ``Delete``: remove a selected waypoint.
- ``Clear``: remove all waypoints and clear visualization.
- ``Stop``: stop execution while keeping the waypoint list.

Joystick in the Interactive Task
--------------------------------

In the ``interactive`` task, the robot starts a joystick reader when the
matching ``/dev/input/jsN`` device exists.

``Share`` switches between:

- direct joystick command publishing, and
- the selected feedback controller.

Mode changes clear raw and cached joystick commands and cancel outstanding
motion ownership. Returning to interactive control captures measured vehicle
and arm references but leaves output idle until an explicit execution request.
Pending waypoint or replay preparation cannot reclaim control after a switch.
Disconnecting a joystick only stops execution when that robot is in joystick
mode. Joysticks remain assigned by robot index; selecting a robot in RViz does
not reassign its joystick. Automatic joystick reconnection is not implemented.

This is manual joystick command input inside the ``interactive`` task. It is
separate from launching the standalone ``manual`` task.

Manual Task
-----------

Launch:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py task:=manual

The ``manual`` task starts the PS4 joystick teleoperation node. Joystick input is
published directly as commands.

Vehicle controls:

- Left stick vertical: surge.
- Left stick horizontal: sway.
- ``L2`` / ``R2``: heave down/up.
- ``L1`` / ``R1``: roll.
- Right stick vertical: pitch.
- Right stick horizontal: yaw.

Manipulator controls:

- D-pad left/right: ``axis_e`` negative/positive.
- D-pad up/down: ``axis_d`` positive/negative.
- Triangle / X: ``axis_c`` positive/negative.
- Square / Circle: ``axis_b`` positive/negative.
- Right-stick press / left-stick press: grasper ``axis_a`` positive/negative.

Options mode:

- ``Options`` toggles between manipulator joint mode and auxiliary mode.
- In auxiliary mode, D-pad left/right publishes light commands.
- In auxiliary mode, D-pad up/down publishes camera mount pitch commands.

.. _direct_thruster_keyboard_task:

Direct Thruster Keyboard Task
-----------------------------

Launch:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=true \
       use_manipulator_hardware:=false \
       sim_robot_count:=0 \
       task:=direct_thrusters

Keyboard channel mapping:

.. code-block:: text

   u -> channel 0
   i -> channel 1
   o -> channel 2
   p -> channel 3
   h -> channel 4
   j -> channel 5
   k -> channel 6
   l -> channel 7

Neutral PWM is ``1500``. Pressed keys publish active PWM ``1700`` for the
mapped channel. Releasing a key returns the channel to neutral.

Obstacle editing
----------------

Use the cyan **New obstacle preview** marker. Set its pose, choose geometry under
**Obstacle settings (preview)**, then **Add obstacle**. To change an existing
object, use **Select obstacle**, edit the preview, then **Update selected obstacle**.
The preview itself is not collision geometry until accepted.

**Shape → Mesh catalog** selects a bundled rigid mesh. **Scene / source** holds
world-profile, clear and reset actions; **Replanning** enables its supervisor.
Source selection is a launch option, independent of planner selection.

For custom assets, set ``obstacle_mesh_resource`` or load a catalog using the
``obstacle_mesh_catalog`` parameter and **Mesh catalog → Reload catalog**.
Catalog entries map names to local mesh resources and scales::

   {
     "My mesh": {
       "collision_mesh_resource": "file:///absolute/path/mesh.stl",
       "collision_mesh_scale": [1.0, 1.0, 1.0]
     }
   }

Collision and overlay displays
------------------------------

``launch_collision_contact:=true`` enables contact/clearance markers.
``launch_voxelviz:=true`` enables environment and dynamic-obstacle surface clouds.
Each display cloud is limited to 250,000 sampled points; collision checks use the
complete geometry. Set the voxel node's ``max_display_points`` parameter to 0
at startup for the complete diagnostic cloud.
Clouds may take time to build; they are visualization, not filled occupancy maps.
Mesh collision checks use triangle surfaces; use primitive collision shapes for
solid-volume behavior. Assets must exist on each machine performing queries.

Text overlays are off by default; enable their displays in RViz. Effort arrows
show vehicle commands and arm actuator effort. Disturbance indicators show valid
estimates; locked/limit-constrained arm joints require checking their status.

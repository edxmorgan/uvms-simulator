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

The ``interactive`` task starts ``interactive_controller`` and exposes an RViz
interactive-marker menu.

Launch:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py task:=interactive

Select the active robot once from ``Robots``. Tool menus such as ``Path
Planner``, ``Cmd Replay``, ``Grasper``, ``Waypoints``, and ``Plan & Execute``
then operate on that selected robot.

Main menu groups:

- ``Plan & Execute``: activate the currently selected feedback controller if it
  is inactive, then plan and execute the selected target or waypoint mission.
- ``Robots``: select active robot when multiple simulated robots exist.
- ``Waypoints``: add, delete, clear, or stop vehicle waypoint missions.
- ``Path Planner``: select the active robot's planner backend, currently
  ``Bitstar`` or ``RRTstar``.
- ``Cmd Replay``: select the active robot's replay profile, reset/play, and
  stop replay. See :doc:`replay_and_experiments` for profile format and
  experiment logging.
- ``Grasper``: open/close the active robot's grasper through feedback
  controllers only.
- ``Reset Manager``: simulator state reset and release controls. Hidden for the
  ``robot_real_`` hardware namespace.
- ``<robot> Control``: select controller and control space for a robot.

Feedback Control and Replay
---------------------------

Feedback control and replay are intentionally separated:

- ``PID`` and ``InvDyn`` are closed-loop feedback controllers.
- ``CmdReplay`` is an open-loop command playback controller.
- ``Plan & Execute`` uses the selected feedback controller and refuses to run
  while ``CmdReplay`` is selected.
- Replay reset/playback requires both ``CmdReplay`` and an explicitly selected
  replay profile.
- Grasper menu commands are rejected while ``CmdReplay`` is active, so they do
  not queue and apply later.

At startup the default PID controller is selected but inactive. It does not
publish closed-loop commands until the user explicitly activates a behavior.
Pressing ``Plan & Execute`` activates the selected feedback controller on
demand.

Controller semantics:

- ``PID`` is not a pure textbook PID-only controller. The vehicle command
  includes hydrostatic restoring compensation in addition to feedback terms.
  The manipulator side is joint-space feedback with configured command limits.
- ``InvDyn`` implements inverse-dynamics control in the computed-torque sense:
  desired state and desired acceleration are mapped through an estimated model
  to actuation, with feedback terms correcting model and state-estimation
  errors. The manipulator model comes from ``Floating-KinDyn-Graph``; the
  vehicle model comes from ``diff_uv``.

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

Grasper Menu Parameters
-----------------------

The grasper menu uses these parameters:

- ``grasper_menu_open_effort``
- ``grasper_menu_close_effort``
- ``grasper_menu_effort_duration``

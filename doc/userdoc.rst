First Simulation
================

Build and source the workspace as described in :doc:`installation`.

Launch
------

Start one simulated vehicle and arm::

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
     use_manipulator_hardware:=false use_vehicle_hardware:=false \
     sim_robot_count:=1 task:=interactive

This starts the simulator, controllers, planner, RViz and PlotJuggler.
Each simulated component starts paused, including in mixed hardware/simulation
launches. Arm-only joystick commands and replay profiles with
``vehicle: zero_command`` leave the simulated vehicle paused. A vehicle or
whole-robot execution request releases it; ``sim_startup_pause:=false`` permits
free drift from launch.
Readiness is automatic; selecting a controller or moving a target marker does
not start motion.

Move the robot
--------------

1. In RViz, select a robot from **Robots** if more than one is present.
2. Choose the controller and control space under **Robot Control**.
3. Move the target marker and select **Plan & Execute**.
4. To queue vehicle goals, add targets under **Waypoints**, then execute.

**Stop Execution** cancels execution and stops command output. Already-running
physics continues; components still paused remain paused.

When replacing an active vehicle plan, the robot holds and settles before the
planner captures its new start state. A failed plan does not execute.
After a simulator reset, use **Plan & Execute** again; no separate release is
needed. See :doc:`controls_and_menus` for teleoperation and replay controls.

Choose a scene from **Environment**. Switching cancels the old plan; execute a
new one after loading completes. Available scenes and failure recovery are in
:doc:`environment_scenes`.

View and record data
--------------------

PlotJuggler starts by default. Use ``launch_plotjuggler:=false`` to disable it.
PlotJuggler 4 uses its saved streaming-buffer setting. For workspace-specific
messages, run a native build from the sourced workspace.

Useful topics:

- ``/dynamic_joint_states``: joint and vehicle states, efforts and estimates.
- ``/<prefix>/reference/targets``: reference trajectories.
- ``/<prefix>/performance/controller``: tracking metrics.

Start and stop MCAP recording from **Data Recording**. To record from launch,
append ``record_data:=true``. Bags are saved under
``~/ros_ws/recordings/mcap/uvms_bag_YYYYmmdd_HHMMSS`` and can be opened in
PlotJuggler. See :doc:`replay_and_experiments` for replay profiles and logging.

Launch options
--------------

Append these to the launch command as needed:

- ``sim_robot_count:=N``: simulate multiple robots.
- ``environment_scene:=pipeline``: choose the initial scene.
- ``gui:=false``: run without GUI applications.
- ``launch_rviz:=false``: disable RViz only.
- ``launch_voxelviz:=true``: show environment voxels.
- ``launch_collision_contact:=true``: enable collision/clearance visualization.
- ``cleanup_stale_nodes:=false``: disable pre-launch process cleanup when
  deliberately running separate simulations.

Camera options are in :doc:`sensors_and_perception`. To list all launch arguments::

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py --show-args

Inspect the runtime
-------------------

If the robot is not ready, check controllers and feedback::

   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ros2 topic echo /dynamic_joint_states

To inspect the robot description separately::

   ros2 launch ros2_control_blue_reach_5 view_robot.launch.py

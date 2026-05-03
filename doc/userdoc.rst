Tutorial
========

This tutorial assumes the workspace has already been installed, built, and
sourced as described in :doc:`installation`.

Verify the Robot Description
----------------------------

Open the robot description in RViz:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 view_robot.launch.py

This checks the xacro/URDF description and the RViz model display before
starting the full runtime stack.

Launch Interactive Simulation
-----------------------------

Start one fully simulated UVMS in the interactive task:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_manipulator_hardware:=false \
       use_vehicle_hardware:=false \
       sim_robot_count:=1 \
       task:=interactive

The launch starts ros2_control, simulated vehicle/manipulator hardware,
controllers, SimLab interactive controls, planner nodes, and RViz.

Common Launch Arguments
-----------------------

- ``use_manipulator_hardware:=false``: use the simulated manipulator.
- ``use_vehicle_hardware:=false``: use the simulated vehicle.
- ``sim_robot_count:=1``: spawn one simulated UVMS.
- ``task:=interactive``: run RViz menus, planning, replay, and optional
  manual joystick command input.
- ``task:=manual``: run PS4 direct command teleoperation.
- ``task:=direct_thrusters``: run keyboard direct-thruster control.
- ``record_data:=true``: start the rosbag2 MCAP recorder.
- ``gui:=false``: run without RViz.

Use the ``interactive`` task for the normal simulator tutorial path. See
:doc:`controls_and_menus` for the menu and joystick behavior of each task.

Inspect the Runtime
-------------------

List active controllers:

.. code-block:: shell

   ros2 control list_controllers

List available hardware interfaces:

.. code-block:: shell

   ros2 control list_hardware_interfaces

Inspect the central state stream:

.. code-block:: shell

   ros2 topic echo /dynamic_joint_states

The exact controller names depend on robot count, task, and hardware/simulation
selection. See :doc:`services_and_interfaces` for the main runtime interfaces.

Plot Data
---------

The main launch file starts PlotJuggler by default:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_manipulator_hardware:=false \
       use_vehicle_hardware:=false \
       sim_robot_count:=1 \
       task:=interactive \
       launch_plotjuggler:=true

Set ``launch_plotjuggler:=false`` when running headless or when you want to open
PlotJuggler manually. The launch file starts ``/snap/bin/plotjuggler`` so the
newer Snap release is used instead of the older ROS package executable.

Useful live topics to plot:

- ``/dynamic_joint_states`` for measured joint position, velocity, effort, and
  acceleration interfaces.
- ``/<prefix>/reference/targets`` for vehicle and arm reference trajectories.
- ``/<prefix>/performance/controller`` for normalized tracking and controller
  performance metrics.

For recorded experiments, open
``~/ros_ws/recordings/mcap/uvms_bag_YYYYmmdd_HHMMSS`` in PlotJuggler and load
the same topics from the bag.

Record Data
-----------

The interactive RViz menu has ``Data Recording`` actions for starting and
stopping MCAP recording during a session. The same actions are exposed through
the SimLab backend API for non-RViz clients. For headless runs, start recording
at launch:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_manipulator_hardware:=false \
       use_vehicle_hardware:=false \
       sim_robot_count:=1 \
       task:=interactive \
       record_data:=true

Bags are saved under ``~/ros_ws/recordings/mcap/uvms_bag_YYYYmmdd_HHMMSS``.
Replay-session CSV logs are saved under
``~/ros_ws/recordings/replay_sessions`` when enabled by a replay profile.
MCAP recordings include measured robot state, the selected camera feed on
``/alpha/image_raw`` and ``/alpha/camera_info``, and per-robot desired target
topics for vehicle pose/velocity/acceleration and arm
position/velocity/acceleration. They also include per-robot controller
performance metrics on ``/<prefix>/performance/controller``; see
:doc:`services_and_interfaces` for the metric fields.

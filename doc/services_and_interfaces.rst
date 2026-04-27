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

Most robot-specific names include the robot prefix. Simulated robots use
prefixes such as ``robot_1_``. The real hardware prefix is typically
``robot_real_``.

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
- ``/robot_1_set_sim_dynamics``: update manipulator payload/gravity dynamics.

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

   float64 gravity
   float64 payload_mass
   float64 payload_ixx
   float64 payload_iyy
   float64 payload_izz

``hold_commands`` keeps simulated manipulator and vehicle commands blocked
after reset until the matching release service is called.

Use ``hold_commands=true`` when you want to inspect the reset state before
controllers drive again. Use ``hold_commands=false`` when a replay/profile
manager should reset and then start immediately.

``SetSimDynamics`` request fields:

.. code-block:: text

   float64 gravity
   float64 mass
   float64 ixx
   float64 iyy
   float64 izz

The response reports ``success`` and ``message``.

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

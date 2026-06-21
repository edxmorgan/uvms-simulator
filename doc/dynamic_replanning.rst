Dynamic Replanning
==================

Dynamic replanning lets the interactive SimLab runtime replace an active
vehicle trajectory when a dynamic obstacle begins blocking the remaining path.
It works for both a normal ``Plan & Execute`` vehicle target and an executing
vehicle waypoint mission. It is intended for online planner experiments: the
robot keeps following the current trajectory while the selected planner searches
for a replacement, then switches to the new trajectory if planning succeeds.

Runtime Model
-------------

The dynamic-replanning path is split into modular pieces:

- ``DynamicWorldModel`` stores the current dynamic-obstacle snapshot.
- A dynamic replanner strategy decides whether the active path is unsafe.
- The selected vehicle planner computes a replacement path.
- The selected trajectory generator converts the replacement path into a new
  executable trajectory.
- The active feedback controller tracks the current trajectory command.

The default strategy is ``clearance_hysteresis``. It samples the remaining path
against dynamic obstacles, triggers a replan when clearance falls below the
configured margin, and suppresses repeated replans for the same unchanged path
until there is a new path or obstacle update.

Enable Dynamic Replanning
-------------------------

Launch the interactive simulator:

.. code-block:: bash

   cd ~/ros_ws
   source install/setup.bash

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
     use_manipulator_hardware:=false \
     use_vehicle_hardware:=false \
     sim_robot_count:=1 \
     task:=interactive \
     launch_rviz:=true \
     launch_plotjuggler:=false \
     launch_camera:=false

Enable dynamic replanning from another terminal:

.. code-block:: bash

   cd ~/ros_ws
   source install/setup.bash

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: enable_dynamic_replanning, robot_index: 0, rate: 10.0, cooldown: 0.5, lookahead_time: 25.0, safety_margin: 0.25, replan_hysteresis: 0.05}"

Check status:

.. code-block:: bash

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: dynamic_replanning_status}"

Disable it:

.. code-block:: bash

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: disable_dynamic_replanning}"

RViz Menu
---------

The same controls are available from the interactive marker context menu in
RViz under ``Dynamic Obstacles``:

- ``Add Path Obstacle`` enables dynamic replanning if needed, then places a
  generated spherical obstacle ahead of the selected robot on its currently
  active path.
- ``Clear Obstacles`` removes all dynamic obstacles.
- ``Enable Replanning`` and ``Disable Replanning`` control the dynamic
  replanner.
- ``Status`` prints the active replanning strategy and counters to the node log.

``Add Path Obstacle`` requires an active planned path. If no path is active, the
menu action logs a warning and does not change the world.

The Reset Manager clears dynamic obstacles as part of simulator reset. This keeps
reset behavior consistent with the rest of the simulated world state and prevents
old obstacles from affecting the next experiment.

Path-Obstacle Demo
------------------

The ``spawn_path_obstacle`` command places a spherical obstacle ahead of the
selected robot along its current active path. This is useful for reproducible
dynamic-replanning demonstrations because the obstacle is guaranteed to be
near the path the robot is actually following.

Start a long vehicle waypoint:

.. code-block:: bash

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: clear_dynamic_obstacles}"

   ros2 service call /backend/waypoint_command simlab/srv/BackendWaypointCommand \
     "{robot_index: 0, command: clear}"

   ros2 service call /backend/pose_command simlab/srv/BackendPoseCommand \
     "{robot_index: 0, command: add_waypoint, pose: {position: {x: 15.0, y: 6.0, z: -1.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, use_current_target: false}"

   ros2 service call /backend/waypoint_command simlab/srv/BackendWaypointCommand \
     "{robot_index: 0, command: execute}"

After the path is active, add obstacles while the robot is moving:

.. code-block:: bash

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: spawn_path_obstacle, name: blocker_1, robot_index: 0, distance_ahead: 4.0, radius: 0.75}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: spawn_path_obstacle, name: blocker_2, robot_index: 0, distance_ahead: 4.0, radius: 0.75}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: spawn_path_obstacle, name: blocker_3, robot_index: 0, distance_ahead: 4.0, radius: 0.75}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: spawn_path_obstacle, name: blocker_4, robot_index: 0, distance_ahead: 4.0, radius: 0.75}"

   ros2 service call /backend/world_command simlab/srv/BackendWorldCommand \
     "{command: spawn_path_obstacle, name: blocker_5, robot_index: 0, distance_ahead: 4.0, radius: 0.75}"

Expected behavior:

- each accepted obstacle appears in RViz;
- the active path is checked against the updated obstacle set;
- the dynamic replanner logs a reason such as ``path clearance ... below
  margin``;
- the planner action returns a replacement path;
- The selected trajectory generator starts a new trajectory;
- the vehicle continues toward the waypoint without stopping unless no safe
  replacement can be found before the conflict becomes imminent.

Implementation Notes
--------------------

Dynamic replanning uses the physical robot collision radius when asking the
planner for a new path. The safety margin belongs to the replanning decision,
not to the robot radius. Inflating the planner radius can make the current
vehicle state invalid during a live replan, especially near obstacles.

Vehicle path planning uses a spherical vehicle approximation and plans the
translation path. Planner requests therefore project the vehicle start and goal
quaternions to yaw-only before calling OMPL. This prevents transient roll or
pitch during trajectory tracking from causing OMPL to reject an otherwise valid
live start state.

The controller state is not changed by this projection. The projection only
affects the planner request used to compute a geometric replacement path.

Validation Commands
-------------------

Focused tests:

.. code-block:: bash

   cd ~/ros_ws
   source install/setup.bash

   python3 -m pytest -q \
     src/uvms-simlab/test/test_dynamic_replanner.py \
     src/uvms-simlab/test/test_vehicle_yaw_continuity.py

Useful log checks after a GUI run:

.. code-block:: bash

   rg "DynamicReplanner\] replanning|Planner found solution" ~/.ros/log/<latest-run>/launch.log
   rg "Skipping invalid start state|Planner action failed|stopping robot" ~/.ros/log/<latest-run>/launch.log

The first command should show replanning triggers and planner successes. The
second command should normally produce no output for a successful demo.

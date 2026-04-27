Project Overview
================

This repository set provides a ROS 2 underwater vehicle-manipulator system
centered on a BlueROV-style floating base and a Reach Alpha manipulator. It is
designed to run the same high-level workflows across simulation, mixed
hardware/simulation, and hardware-in-the-loop experiments.

The stack is split across two main packages:

- ``uvms-simulator`` provides the exported ROS package
  ``ros2_control_blue_reach_5``. It contains the launch system, xacro/URDF
  robot descriptions, ros2_control hardware interfaces, reset/dynamics
  services, camera drivers, controller-manager configuration, and dependency
  workspace metadata.
- ``uvms-simlab`` provides the exported ROS package ``simlab``. It contains
  the interactive RViz runtime, controller implementations, planner action
  server/client, replay profiles, experiment logging, joystick tooling, mocap
  helpers, and perception utilities.

The intended mental model is one UVMS project with two cooperating ROS
packages: ``uvms-simulator`` owns the system description and hardware/simulator
interfaces, while ``uvms-simlab`` owns the experiment/runtime behavior layered
on top of those interfaces.

System Layers
-------------

- Robot description and launch: ``uvms-simulator`` defines the UVMS model,
  hardware/sim selection, controller configuration, and launch-time runtime
  options.
- Hardware interfaces: real and simulated manipulator/vehicle interfaces
  expose the same command/state surfaces through ros2_control.
- Runtime control: ``uvms-simlab`` selects behaviors, controllers, planners,
  replay profiles, grasper commands, and visualization through RViz and
  joystick inputs.
- Experiment infrastructure: command replay, reset/dynamics metadata, replay
  session logging, rosbag recording, mocap conversion, and plotting utilities
  support repeatable simulator and hardware experiments.
- Environment/perception: bathymetry/workspace visualization, collision
  context, camera drivers, and optional RGB-to-pointcloud tooling support
  planning and operator feedback.

Core Runtime Nodes
------------------

- ``interactive_controller``: RViz interactive markers, controller switching,
  path planning, waypoint execution, grasper commands, reset management, and
  command replay orchestration.
- ``planner_action_server_node``: OMPL-backed path-planning action server used
  by the interactive controller.
- ``bag_recorder``: rosbag2 MCAP recording for simulator and hardware sessions.
- ``mocap_publisher``: OptiTrack/mocap4r2 bridge output conversion and path
  publishing when mocap is enabled.
- ``rgb2cloudpoint_publisher``: optional RGB-to-pointcloud perception helper.
- ``collision_contact_node``, ``voxelviz_node``, and ``env_obstacles_node``:
  environment visualization and collision/context utilities.

Launch Modes
------------

The main launch file is provided by ``uvms-simulator``. The ``task:=...``
argument selects the runtime mode:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py task:=interactive

Supported task modes:

- ``interactive``: RViz menu/marker workflow with planning, replay, reset
  manager, grasper commands, overlays, and SimLab runtime nodes.
- ``manual``: PS4 joystick teleoperation through ``joystick_controller``.
- ``joint``: custom joint-space command node entry point.
- ``direct_thrusters``: keyboard PWM channel control through
  ``direct_thruster_controller``.

Useful launch switches:

- ``use_manipulator_hardware:=true``: use real Reach Alpha hardware.
- ``use_vehicle_hardware:=true``: use real vehicle hardware.
- ``sim_robot_count:=N``: spawn N simulated UVMS robots.
- ``use_mocap:=true``: start OptiTrack/mocap4r2 bridge and mocap visualization
  helpers. The default is ``false``.
- ``record_data:=true``: start rosbag2 MCAP recording.
- ``gui:=false``: run without RViz.
- ``launch_camera:=auto|true|false``: control the GStreamer camera node.
- ``simulate_camera:=true``: use a synthetic test-pattern camera.

Command Replay
--------------

Command replay profiles live in ``uvms-simlab/resource/csv_playback``. A replay
profile contains a command CSV plus ``replay.json`` metadata for reset behavior,
initial state, dynamics, repeat count, and optional recording.

Replay is explicit:

- Select ``CmdReplay`` as the controller.
- Select a replay profile.
- Run reset/playback from the ``Cmd Replay`` menu.

Regular planning and replay are intentionally separated so planner trajectories
and raw command replay do not run at the same time.

Controller Modes
----------------

The default controller is bound at startup but remains idle until the user
explicitly activates a behavior. ``Plan & Execute`` activates the selected
regular controller on demand. ``CmdReplay`` remains isolated from planner
execution.

Dynamics Provenance
-------------------

The floating-base kinematics/dynamics and controller model structure used by
the stack are derived from two companion projects:

- `floating-KinDyn <https://github.com/edxmorgan/floating-KinDyn>`_: floating
  kinematics and dynamics foundations.
- `diff_uv <https://github.com/edxmorgan/diff_uv>`_: differentiable UVMS
  dynamics and controller formulation.

Hardware Versus Simulation
--------------------------

Simulation-only reset controls are exposed through ``Reset Manager`` and are
hidden for real robot prefixes. Hardware replay uses controller-based settling
before playback rather than simulator state reset.

Capability Map
--------------

- Multi-robot simulated UVMS bringup.
- Real/sim manipulator and vehicle mixing through launch arguments.
- Shared ros2_control command/state interfaces for simulator and hardware.
- Simulator reset/release and payload/gravity/inertia dynamics services.
- Hydrostatic-compensated feedback, computed-torque/inverse-dynamics control,
  and extensible controller paths, with MPC planned.
- OMPL vehicle planning through an action server.
- RViz interactive marker target selection.
- Vehicle waypoint queues and waypoint execution.
- Whole-body and joint-space control modes.
- PS4 joystick teleoperation.
- Direct thruster PWM keyboard testing.
- Command replay from CSV profiles with reset/dynamics metadata.
- Optional replay-session CSV logging.
- Mocap pose/path conversion and visualization when enabled.
- Camera launch, simulated camera mode, and optional RGB-to-pointcloud support.
- Bathymetry/workspace visualization and collision context.

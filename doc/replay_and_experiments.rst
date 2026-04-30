Command Replay and Experiments
==============================

Command replay runs repeatable CSV profiles. A profile can replay recorded
vehicle/manipulator commands directly, hold one subsystem while exciting the
other, or track recorded desired-state references through a feedback
controller. Use it for payload identification probes, vehicle force tests, and
whole-body experiments where the same profile should run across trials.

Workflow
--------

In RViz:

- Select the active robot from ``Robots``.
- Select ``CmdReplay`` from the robot controller menu.
- Select a replay profile from ``Cmd Replay > Profiles``.
- Use ``Cmd Replay > Reset Robot + Playback`` to prepare the initial condition
  and start playback.
- Use ``Cmd Replay > Stop`` to stop playback and publish zero replay
  effort/wrench commands.

Selecting ``CmdReplay`` chooses the replay controller. Playback starts after a
replay profile is selected and ``Reset Robot + Playback`` is requested.

Replay Profile Layout
---------------------

Replay profiles live in:

.. code-block:: text

   uvms-simlab/resource/playback_profile/<profile_name>/

Each replay profile contains:

- ``commands.csv``: the time-indexed command sequence.
- ``replay.json``: metadata describing the CSV columns, reset state, dynamics
  profile, subsystem mode, repeat behavior, and optional replay CSV logging.

Use descriptive names because replay profiles appear directly in the RViz
``Cmd Replay`` menu. Examples:

- ``arm_payload_0p76kg``
- ``vehicle_fx_10n_20s``

Command CSV
-----------

``commands.csv`` uses ``time_sec`` as the replay timeline. Each row is held
until the next timestamp.

For a constant 20 second vehicle ``Fx`` command:

.. code-block:: text

   time_sec,vehicle_fx
   0.0,10.0
   20.0,10.0

Common command columns:

- Vehicle wrench: ``vehicle_fx``, ``vehicle_fy``, ``vehicle_fz``,
  ``vehicle_tx``, ``vehicle_ty``, ``vehicle_tz``.
- Manipulator torque: ``tau_axis_e``, ``tau_axis_d``, ``tau_axis_c``,
  ``tau_axis_b``, ``tau_axis_a``.

Columns omitted from the CSV are filled with zero. This supports
manipulator-only, vehicle-only, and whole-body replay profiles.

Dynamics Profiles
-----------------

Dynamics profiles live in:

.. code-block:: text

   uvms-simlab/resource/dynamics_profiles/<profile_name>.json

A dynamics profile defines the robot parameter set used by simulator dynamics
services and supported hardware-side dynamics paths. It can be selected during
live simulation from the RViz ``Dynamics Profile`` menu, or referenced by a
replay profile through ``reset.robot_dynamics_profile``.

Installed examples:

- ``dory_alpha``: nominal Dory Alpha profile.
- ``dory_alpha_payload_0p76kg``: Dory Alpha with a 0.76 kg manipulator payload.

Each dynamics profile contains:

- ``use_coupled_dynamics``: ``false`` for independent vehicle/manipulator
  parameter blocks; ``true`` for one coupled UVMS dynamics model.
- ``vehicle``: lumped vehicle parameters, damping/restoring terms, current
  velocity, and the 6x8 thruster configuration matrix.
- ``manipulator``: link inertial parameters, friction terms, gravity vector,
  payload mass/inertia, base/world/tip offsets, and simulated manipulator
  constraint parameters.

For hardware namespaces, the vehicle parameter block is skipped. The
manipulator parameter block is sent to the hardware-side dynamics path when the
interface supports parameter updates.

Replay Metadata
---------------

``replay.json`` connects the CSV, reset request, dynamics profile, subsystem
mode, repeat behavior, and recording option.

Example:

.. code-block:: json

   {
     "csv": "commands.csv",
     "time_column": "time_sec",
     "columns": {
       "vehicle": ["vehicle_fx", "vehicle_fy", "vehicle_fz", "vehicle_tx", "vehicle_ty", "vehicle_tz"],
       "manipulator": ["tau_axis_e", "tau_axis_d", "tau_axis_c", "tau_axis_b", "tau_axis_a"]
     },
     "playback": {
       "repeats": 3
     },
     "subsystem_mode": {
       "vehicle": "hold_initial",
       "manipulator": "replay_command",
       "feedback_controller": "PID"
     },
     "reset": {
       "hardware_settle": {
         "controller": "PID",
         "position_tolerance": 0.18,
         "velocity_tolerance": 0.03,
         "vehicle_position_tolerance": 0.08,
         "vehicle_velocity_tolerance": 0.05,
         "timeout_sec": 30.0
       },
       "reset_manipulator": true,
       "reset_vehicle": true,
       "require_release_after_reset": false,
       "manipulator": {
         "enabled": true,
         "position": [3.1, 0.7, 0.4, 2.1, 0.0],
         "velocity": [0.0, 0.0, 0.0, 0.0, 0.0]
       },
       "vehicle": {
         "enabled": true,
         "pose": [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
         "twist": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
         "wrench": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       },
       "robot_dynamics_profile": "dory_alpha_payload_0p0kg"
     },
     "recording": {
       "enabled": false
     }
   }

Subsystem Mode
--------------

``subsystem_mode`` defines how each subsystem is driven during a replay pass:

- ``"replay_command"``: publish the recorded command columns from
  ``commands.csv``.
- ``"track_reference"``: use the recorded desired-state columns and the
  configured feedback controller to compute commands during replay.
- ``"hold_initial"``: use the configured feedback controller to hold the reset
  state.
- ``"zero_command"``: publish zero effort/wrench commands for that subsystem.

Typical experiment modes:

- Manipulator probe with vehicle station keeping:
  ``vehicle = "hold_initial"``, ``manipulator = "replay_command"``.
- Vehicle force test with manipulator hold:
  ``vehicle = "replay_command"``, ``manipulator = "hold_initial"``.
- Reference replay:
  set the subsystem to ``"track_reference"`` and set
  ``feedback_controller`` to the controller that should track the recorded
  desired state.
- Whole-body replay:
  ``vehicle = "replay_command"``, ``manipulator = "replay_command"``.

Reset Behavior
--------------

Replay reset prepares the robot before each pass.

- In simulation, replay calls reset services and sets the requested
  manipulator and vehicle state.
- On hardware, ``reset.hardware_settle`` uses the configured controller to move
  the physical system toward the requested initial condition before playback.

For payload identification profiles, keep ``robot_dynamics_profile`` matched to
the payload being tested.

Repeated Passes
---------------

For finite repeated playback, the sequence is:

.. code-block:: text

   reset -> CSV pass 1 -> reset -> CSV pass 2 -> reset -> CSV pass 3

Use ``repeats`` for identification experiments where each pass should begin
from the configured initial state.

Replay CSV Logging
------------------

Replay session recording is controlled per replay profile:

.. code-block:: json

   "recording": {
     "enabled": true
   }

Recorded rows cover the replay interval. Typical columns include:

- Arm position: ``q_alpha_axis_e/d/c/b``.
- Arm velocity: ``dq_alpha_axis_e/d/c/b``.
- Arm acceleration: ``ddq_alpha_axis_e/d/c/b``.
- Arm measured/applied effort: ``effort_alpha_axis_e/d/c/b``.
- Arm command: ``cmd_tau_axis_e/d/c/b/a``.
- Vehicle pose, body velocity, body acceleration, wrench, and command wrench.

Keep ``recording.enabled`` set to ``false`` for profiles where per-pass replay
logs are unnecessary.

This CSV logging belongs to CmdReplay. It records only the replay pass, not the
full ROS graph.

MCAP to Replay Profiles
-----------------------

Use the ``Data Recording`` RViz menu to start and stop an MCAP recording around
the behavior you want to capture. The MCAP records ROS topics such as
``dynamic_joint_states``, ``mocap_pose``, and per-robot experiment topics:

- ``/<prefix>/reference/targets``
- ``/<prefix>/performance/controller``

The reference topic uses ``simlab_msgs/msg/ReferenceTargets`` and
contains the world target, vehicle NED/body target, and manipulator reference in
one timestamped message. The performance topic uses
``simlab_msgs/msg/ControllerPerformance`` and records normalized tracking,
control-effort, energy, and time-to-tolerance metrics for the active behavior.

Convert one robot from that bag into a CmdReplay profile with:

.. code-block:: bash

   ros2 run simlab mcap_to_replay_profile \
       ~/ros_ws/uvms_bag_YYYYmmdd_HHMMSS \
       my_replay_profile \
       --robot-prefix robot_1_ \
       --dynamics-profile dory_alpha

The converter writes one replay profile:

.. code-block:: text

   uvms-simlab/resource/playback_profile/my_replay_profile/commands.csv
   uvms-simlab/resource/playback_profile/my_replay_profile/replay.json

``commands.csv`` contains the replay command columns plus aligned desired-state
columns from the target topics:

- Vehicle desired pose: ``target_ned_x/y/z/roll/pitch/yaw``.
- Vehicle desired body velocity: ``target_body_u/v/w/p/q/r``.
- Vehicle desired body acceleration:
  ``target_body_du/dv/dw/dp/dq/dr``.
- Arm desired position, velocity, and acceleration:
  ``arm_ref_axis_*``, ``arm_dref_axis_*``, and ``arm_ddref_axis_*``.

CmdReplay uses the command columns when a subsystem is in ``replay_command``
mode. It uses the desired-state columns when a subsystem is in
``track_reference`` mode.

Run the command from ``~/ros_ws`` to use that default output location. Use
``--output-root`` to write profiles somewhere else.

For multi-robot recordings, run the converter once per robot prefix:

.. code-block:: bash

   ros2 run simlab mcap_to_replay_profile bag_dir robot_1_profile --robot-prefix robot_1_
   ros2 run simlab mcap_to_replay_profile bag_dir robot_2_profile --robot-prefix robot_2_

Converter options:

- ``--sample-period 0.04``: downsample the profile to about 25 Hz.
- ``--vehicle-mode hold_initial``: set ``subsystem_mode.vehicle`` in
  ``replay.json``.
- ``--manipulator-mode hold_initial``: set ``subsystem_mode.manipulator`` in
  ``replay.json``.
- ``--feedback-controller PID``: set the feedback controller used by
  ``track_reference`` and ``hold_initial`` modes.
- ``--arm-interface effort``: read the recorded joint effort interface into the
  manipulator command columns.

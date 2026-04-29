Command Replay and Experiments
==============================

Command replay sends repeatable open-loop vehicle wrench and manipulator torque
commands from CSV files. Use it for payload identification probes, vehicle
force tests, and whole-body experiments where the same command sequence should
be applied across runs.

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
  profile, subsystem policy, repeat behavior, and optional recording.

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
policy, repeat behavior, and recording option.

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
       "repeats": 3,
       "loop": false
     },
     "control_policy": {
       "vehicle": "hold",
       "manipulator": "replay",
       "stabilizing_controller": "PID"
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

Control Policy
--------------

``control_policy`` defines each subsystem's command source during replay:

- ``"replay"``: use commands from the CSV.
- ``"hold"``: use the configured stabilizing controller to hold the initial
  state.
- ``"zero"``: publish zero effort/wrench commands for that subsystem.

Typical experiment policies:

- Manipulator probe with vehicle station keeping:
  ``vehicle = "hold"``, ``manipulator = "replay"``.
- Vehicle force test with manipulator hold:
  ``vehicle = "replay"``, ``manipulator = "hold"``.
- Whole-body replay:
  ``vehicle = "replay"``, ``manipulator = "replay"``.

Reset Behavior
--------------

Replay reset prepares the robot before each pass.

- In simulation, replay calls reset services and sets the requested
  manipulator and vehicle state.
- On hardware, ``reset.hardware_settle`` uses the configured controller to move
  the physical system toward the requested initial condition before playback.

For payload identification profiles, keep ``robot_dynamics_profile`` matched to
the payload being tested.

Repeats and Looping
-------------------

For finite repeated playback, the sequence is:

.. code-block:: text

   reset -> CSV pass 1 -> reset -> CSV pass 2 -> reset -> CSV pass 3

Use ``repeats`` for identification experiments where each pass should begin
from the configured initial state. Use ``loop`` for continuous replay.

Recording
---------

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

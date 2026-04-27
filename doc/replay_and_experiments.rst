Command Replay and Experiments
==============================

Command replay is for repeatable open-loop command playback from CSV. It is
used for payload identification probes, vehicle force tests, and whole-body
experiments.

Profile Layout
--------------

Profiles live in:

.. code-block:: text

   uvms-simlab/resource/csv_playback/<profile_name>/

Each profile contains:

- ``commands.csv``: command samples.
- ``replay.json``: metadata for CSV columns, repeat behavior, reset state,
  dynamics, control policy, and optional recording.

Creating a Profile
------------------

Create:

.. code-block:: text

   uvms-simlab/resource/csv_playback/my_profile/commands.csv
   uvms-simlab/resource/csv_playback/my_profile/replay.json

Guidelines:

- Use explicit names such as ``arm_payload_0p76kg`` or
  ``vehicle_fx_10n_20s``.
- Include only command columns you need; missing columns become zero.
- Set ``control_policy`` so the non-excited subsystem is held or zeroed
  intentionally.
- Set ``recording.enabled`` only when you want per-pass replay CSV logs.
- For hardware, tune ``reset.hardware_settle`` tolerances and timeout for the
  real system rather than using simulator reset assumptions.

CSV Timing
----------

``time_sec`` is the playback timeline. A sample value is held until the next
sample timestamp. For a constant 20 second command, two rows are enough:

.. code-block:: text

   time_sec,vehicle_fx
   0.0,10.0
   20.0,10.0

Missing command columns are treated as zero. This lets a manipulator-only CSV
omit vehicle columns, and a vehicle-only CSV omit manipulator columns.

Replay Metadata
---------------

Typical profile structure:

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
       "dynamics": {
         "gravity": 9.81,
         "payload_mass": 0.0,
         "payload_inertia": [0.0, 0.0, 0.0]
       }
     },
     "recording": {
       "enabled": false
     }
   }

Control Policy
--------------

``control_policy`` defines what happens to subsystems that are not being
replayed:

- ``"replay"``: use commands from the CSV.
- ``"hold"``: use the stabilizing controller to hold the initial state.
- ``"zero"``: publish zero commands for that subsystem.

Examples:

- Manipulator probe while station-keeping vehicle:
  ``vehicle = "hold"``, ``manipulator = "replay"``.
- Vehicle force test while locking manipulator:
  ``vehicle = "replay"``, ``manipulator = "hold"``.
- Whole-body replay:
  ``vehicle = "replay"``, ``manipulator = "replay"``.

Repeats
-------

For finite repeated playback, the sequence is:

.. code-block:: text

   reset -> CSV pass 1 -> reset -> CSV pass 2 -> reset -> CSV pass 3

``loop`` is for continuous replay. For identification experiments, prefer
finite ``repeats`` so each pass begins from the configured initial state.

Simulation Versus Hardware Reset
--------------------------------

In simulation, replay can call reset services directly and set both manipulator
and vehicle state.

On hardware, replay cannot teleport state. Hardware profiles use
``reset.hardware_settle``: a regular controller drives the robot toward the
configured initial condition, then replay starts after tolerance checks or
timeout.

Recording
---------

Replay session recording is controlled by each profile:

.. code-block:: json

   "recording": {
     "enabled": true
   }

Recorded CSV rows include only the replay interval, not the hardware settle
phase. Columns include:

- Arm position: ``q_alpha_axis_e/d/c/b``.
- Arm velocity: ``dq_alpha_axis_e/d/c/b``.
- Arm acceleration: ``ddq_alpha_axis_e/d/c/b``.
- Arm measured/applied effort: ``effort_alpha_axis_e/d/c/b``.
- Arm command: ``cmd_tau_axis_e/d/c/b/a``.
- Vehicle pose, body velocity, body acceleration, wrench, and command wrench.

Plotting Replay Sessions
------------------------

Use the standalone plotting helper in the replay-session tooling to inspect:

- ``q_alpha_axis_{e,d,c,b}``
- ``dq_alpha_axis_{e,d,c,b}``
- ``ddq_alpha_axis_{e,d,c,b}``
- ``effort_alpha_axis_{e,d,c,b}``
- ``cmd_tau_axis_*``

This is the fastest way to verify whether command and measured effort are
aligned for a replay session.

Command Replay and Experiments
==============================

Command replay runs repeatable open-loop commands from a CSV profile. It is
used for payload identification probes, vehicle force tests, and whole-body
experiments where the exact command sequence matters.

Basic Workflow
--------------

In RViz:

- Select the active robot from ``Robots``.
- Select ``CmdReplay`` from the robot controller menu.
- Select a replay profile from ``Cmd Replay > Profiles``.
- Use ``Cmd Replay > Reset Robot + Playback`` to reset or settle the robot,
  then start playback.
- Use ``Cmd Replay > Stop`` to stop playback and publish zero replay commands.

Playback does not start from only selecting ``CmdReplay``. A replay profile
must be selected explicitly.

Profile Files
-------------

Profiles live in:

.. code-block:: text

   uvms-simlab/resource/csv_playback/<profile_name>/

Each profile contains:

- ``commands.csv``: command samples.
- ``replay.json``: metadata for CSV columns, repeat behavior, reset state,
  dynamics, control policy, and optional recording.

Profile naming:

- Use explicit names such as ``arm_payload_0p76kg`` or
  ``vehicle_fx_10n_20s``.
- Avoid generic names such as ``test`` or ``profile1`` because profiles appear
  directly in the RViz menu.

Command CSV
-----------

``time_sec`` is the replay timeline. Each row is held until the next timestamp.
For a constant 20 second vehicle ``Fx`` command, two rows are enough:

.. code-block:: text

   time_sec,vehicle_fx
   0.0,10.0
   20.0,10.0

Supported command columns are configured by ``replay.json``. The common column
names are:

- Vehicle wrench: ``vehicle_fx``, ``vehicle_fy``, ``vehicle_fz``,
  ``vehicle_tx``, ``vehicle_ty``, ``vehicle_tz``.
- Manipulator torque: ``tau_axis_e``, ``tau_axis_d``, ``tau_axis_c``,
  ``tau_axis_b``, ``tau_axis_a``.

Missing columns are treated as zero. This lets a manipulator-only CSV omit
vehicle columns, and a vehicle-only CSV omit manipulator columns.

Replay Metadata
---------------

``replay.json`` connects the CSV to the replay controller and defines reset,
repeat, hold, and recording behavior. A typical profile looks like:

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

``control_policy`` defines what each subsystem does during replay:

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

Reset Behavior
--------------

Simulation and hardware reset are intentionally different:

- In simulation, replay calls reset services and can set manipulator and
  vehicle state directly.
- On hardware, replay cannot teleport state. The ``reset.hardware_settle``
  controller moves the system toward the configured initial condition before
  playback starts.

The ``reset.dynamics`` section sets gravity and payload values used by the
profile. For payload identification profiles, keep this metadata consistent
with the physical payload being tested.

Repeats and Looping
-------------------

For finite repeated playback, the sequence is:

.. code-block:: text

   reset -> CSV pass 1 -> reset -> CSV pass 2 -> reset -> CSV pass 3

``loop`` is for continuous replay. For identification experiments, prefer
finite ``repeats`` so each pass begins from the configured initial state.

Recording
---------

Replay session recording is controlled per profile:

.. code-block:: json

   "recording": {
     "enabled": true
   }

Recorded CSV rows include only the replay interval, not the hardware settle
phase. Typical columns include:

- Arm position: ``q_alpha_axis_e/d/c/b``.
- Arm velocity: ``dq_alpha_axis_e/d/c/b``.
- Arm acceleration: ``ddq_alpha_axis_e/d/c/b``.
- Arm measured/applied effort: ``effort_alpha_axis_e/d/c/b``.
- Arm command: ``cmd_tau_axis_e/d/c/b/a``.
- Vehicle pose, body velocity, body acceleration, wrench, and command wrench.

Keep ``recording.enabled`` set to ``false`` unless the experiment needs a
per-pass replay log.

Plotting Replay Sessions
------------------------

Use the replay-session plotting helper to inspect:

- ``q_alpha_axis_{e,d,c,b}``
- ``dq_alpha_axis_{e,d,c,b}``
- ``ddq_alpha_axis_{e,d,c,b}``
- ``effort_alpha_axis_{e,d,c,b}``
- ``cmd_tau_axis_*``

This is the quickest way to check whether command, measured effort, position,
velocity, and acceleration are aligned during a replay pass.

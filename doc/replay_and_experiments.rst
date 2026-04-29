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
- Use ``Cmd Replay > Reset Robot + Playback`` to reset the simulated robot or
  drive hardware to the configured initial condition, then start playback.
- Use ``Cmd Replay > Stop`` to stop playback and publish zero effort/wrench
  replay commands.

Playback does not start from only selecting ``CmdReplay``. A replay profile
must be selected explicitly.

Profile Files
-------------

Profiles live in:

.. code-block:: text

   uvms-simlab/resource/playback_profile/<profile_name>/

Each profile contains:

- ``commands.csv``: command samples.
- ``replay.json``: metadata for CSV columns, repeat behavior, reset state,
  robot dynamics profile, control policy, and optional recording.

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
       "robot_dynamics_profile": "dory_alpha_payload_0p0kg"
     },
     "recording": {
       "enabled": false
     }
   }

``robot_dynamics_profile`` names a complete robot dynamics profile under
``uvms-simlab/resource/dynamics_profiles``. Each profile defines the robot
dynamics used during replay, including vehicle parameters, manipulator
parameters, and ``use_coupled_dynamics``. Set ``use_coupled_dynamics`` to
``false`` for independent vehicle/manipulator subsystem parameters. Set it to
``true`` when the parameters belong to one coupled UVMS dynamics model.

Control Policy
--------------

``control_policy`` defines what each subsystem does during replay:

- ``"replay"``: use commands from the CSV.
- ``"hold"``: use the stabilizing controller to hold the initial state.
- ``"zero"``: publish zero effort/wrench commands for that subsystem.

Examples:

- Manipulator probe while station-keeping vehicle:
  ``vehicle = "hold"``, ``manipulator = "replay"``.
- Vehicle force test while holding manipulator position:
  ``vehicle = "replay"``, ``manipulator = "hold"``.
- Whole-body replay:
  ``vehicle = "replay"``, ``manipulator = "replay"``.

Reset Behavior
--------------

Simulation and hardware reset are intentionally different:

- In simulation, replay calls reset services and can set manipulator and
  vehicle state directly.
- On hardware, replay cannot instantaneously reset physical state. The
  ``reset.hardware_settle`` controller moves the system toward the configured
  initial condition before playback starts.

For payload identification profiles, keep ``robot_dynamics_profile`` consistent
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

Recorded CSV rows include only the replay interval, not the hardware
initialization phase. Typical columns include:

- Arm position: ``q_alpha_axis_e/d/c/b``.
- Arm velocity: ``dq_alpha_axis_e/d/c/b``.
- Arm acceleration: ``ddq_alpha_axis_e/d/c/b``.
- Arm measured/applied effort: ``effort_alpha_axis_e/d/c/b``.
- Arm command: ``cmd_tau_axis_e/d/c/b/a``.
- Vehicle pose, body velocity, body acceleration, wrench, and command wrench.

Keep ``recording.enabled`` set to ``false`` unless the experiment needs a
per-pass replay log.

Inspecting Replay Sessions
--------------------------

Replay-session CSV files can be loaded into PlotJuggler, Python, MATLAB, or a
notebook. Compare the replayed command columns against the measured state and
effort columns from the same pass:

- ``cmd_tau_axis_e/d/c/b/a`` against ``effort_alpha_axis_e/d/c/b``.
- ``cmd_tau_axis_e/d/c/b`` against ``q_alpha_axis_e/d/c/b``,
  ``dq_alpha_axis_e/d/c/b``, and ``ddq_alpha_axis_e/d/c/b``.
- ``cmd_vehicle_fx/fy/fz/tx/ty/tz`` against vehicle pose, body velocity, and
  body acceleration columns.

This helps detect timestamp offsets, dropped samples, wrong CSV columns, or
unexpected actuator/sensor delay.

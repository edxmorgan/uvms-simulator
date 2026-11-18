Command Replay and Experiments
==============================

Run CSV command sequences or track recorded references through a feedback
controller. Each profile defines initial states, subsystem modes and repetitions.

Workflow
--------

In RViz:

- Select the active robot from ``Robots``.
- Select ``CmdReplay`` from ``Robot Control > Controller``.
- Select a replay profile from ``Cmd Replay > Profiles``.
- Use ``Cmd Replay > Reset Robot + Playback`` to prepare the initial condition
  and start playback.
- Use ``Cmd Replay > Stop`` to stop playback and publish zero replay
  effort/wrench commands.

Replay Profile Layout
---------------------

Replay profiles live in:

.. code-block:: text

   uvms-simlab/resource/playback_profile/<profile_name>/

Each replay profile contains:

- ``commands.csv``: the time-indexed command sequence.
- ``replay.json``: metadata describing the CSV columns, reset state, dynamics
  profile, subsystem mode, repeat behavior, and optional replay CSV logging.

The folder name appears in the profile menu. Copy an installed profile when
creating an experiment.

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
- ``dory_alpha_payload_0p38kg``: Dory Alpha with a 0.38 kg manipulator payload.
- ``camera_murky_underwater``: camera-only profile for a murkier simulated
  underwater view.

Each dynamics profile contains:

- ``use_coupled_dynamics``: ``false`` for independent vehicle/manipulator
  parameter blocks; ``true`` for one coupled UVMS dynamics model.
- ``vehicle``: lumped vehicle parameters, damping/restoring terms, current
  velocity, and the 6x8 thruster configuration matrix.
- ``manipulator``: link inertial parameters, friction terms, gravity vector,
  payload mass/inertia, base/world/tip offsets, and simulated manipulator
  constraint parameters.
- ``camera``: optional simulated camera profile parameters applied through
  ``/sim_camera_renderer_node/set_sim_camera_dynamics`` when the profile is
  selected. Supported fields are ``underwater_effect``,
  ``underwater_haze``, ``underwater_tint``, ``underwater_blur``,
  ``underwater_noise``, and ``underwater_vignette``.


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

For an arm probe with vehicle station keeping, select vehicle ``hold_initial``
and manipulator ``replay_command``. Reference tracking requires desired-state
columns as well as ``feedback_controller``.

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

Set ``playback.repeats`` to the number of passes. Each pass resets the robot
before playback. Reference tracking also clears controller memory and accumulated
performance metrics; gains and model parameters are retained. Missing feedback
controllers reject playback.

Replay CSV Logging
------------------

Set ``recording.enabled: true`` in ``replay.json`` to write per-pass CSV logs
under ``~/ros_ws/recordings/replay_sessions``. Columns include joint state and
effort, vehicle state and wrench, commands, and obstacle IDs. When obstacles are
present, a ``*_dynamic_obstacles.json`` sidecar stores start/end snapshots.

Leave this option false when only the standard MCAP recording is needed.
MCAP captures the time-varying ROS streams; CSV logging covers the replay pass.

Plot Replay Sessions
--------------------

Plot a recorded pass::

   ros2 run simlab plot_replay_session session.csv --time sim_time_sec --only pose
   ros2 run simlab plot_replay_session --only arm --arm e,d

Omitting the CSV path selects the newest file in ``recordings/replay_sessions``.
Use ``--help`` for all selections. Time can be ``replay_time_sec`` (default),
``sim_time_sec`` or ``wall_time_sec``.

The ROS node accepts equivalent parameters (underscores replace hyphens)::

   ros2 run simlab replay_plot_node --ros-args \
     -p csv:=/absolute/path/session.csv -p only:=arm -p arm:=e,d

MCAP to Replay Profiles
-----------------------

Record the experiment using **Data Recording** as described in :doc:`userdoc`.
Include ``/<prefix>/reference/targets`` for reference tracking and
``dynamic_joint_states`` for commands/state. Dynamic obstacle streams are
recorded for visualization; they are not commands to the simulator.

Convert one robot from that bag into a CmdReplay profile with:

.. code-block:: bash

   ros2 run simlab mcap_to_replay_profile \
       ~/ros_ws/recordings/mcap/uvms_bag_YYYYmmdd_HHMMSS \
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

For multi-robot bags, run the converter separately for each robot prefix.

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

Disturbance injection
---------------------

The simulation launch starts one idle disturbance node per robot. Apply a finite
load with::

   ros2 run simlab disturbance_run --robot-prefix robot_1_ \
     --dof surge yaw --amplitude 3 0.1 --waveform sine triangle \
     --frequency 0.1 --cycles 3 --phase 0 -90 --delay 0 2

Generalized DOFs are ``surge sway heave`` (N), ``roll pitch yaw`` and
``arm_e arm_d arm_c arm_b`` (Nm). Supply an amplitude per DOF. Waveform, phase
(degrees) and delay (seconds) accept one value or one per DOF.

Waveforms are ``sine``, ``step``, ``square``, ``triangle``, ``chirp``, ``gaussian``
and ``correlated_gaussian``. Chirp accepts ``--end-frequency``. Step and noise
support delay but not phase. Positive phase leads; delay postpones the start.

For time-correlated Gaussian loads::

   ros2 run simlab disturbance_run --robot-prefix robot_1_ \
     --dof surge roll --amplitude 1 0.1 --waveform correlated_gaussian \
     --correlation-time 2 --frequency 0.1 --cycles 7 --seed 42

For both Gaussian forms, amplitude is standard deviation, not a peak bound.
``cycles / frequency`` sets duration; frequency does not set noise bandwidth.
``correlation-time`` is the OU decay time in seconds.

Runs have a 2-second zero baseline and 3-second zero tail. Waveforms are sampled
at 20 Hz on completed simulation time; pause freezes waveform time. Reset aborts
the run. Injection is simulation-only.

Task-space loads
~~~~~~~~~~~~~~~~

Apply a wrench at the IK tool origin::

   ros2 run simlab disturbance_run --robot-prefix robot_1_ \
     --space task --wrench-frame tool --dof fx tz --amplitude 1 0.1 \
     --waveform sine --frequency 0.1 --cycles 3

Task DOFs are ``fx fy fz tx ty tz``. ``--wrench-frame`` selects tool axes,
vehicle-body FLU axes (``body``), or world NWU axes (``world``). All torques are
about the tool origin. Injection does not unlock joints.

Live plots and recording
~~~~~~~~~~~~~~~~~~~~~~~~

The viewer compares applied load and EKF estimate. Shading is the estimate's
model-based ±2σ uncertainty, not the injected noise amplitude. Constrained or
invalid estimates are marked unavailable. Task plots show equivalent wrench in
tool axes; a valid estimate alone does not establish tool contact.

Repeated runs for the same robot reuse the window. ``--no-plot`` skips it;
``--close-on-finish`` closes it on completion. Closing the window does not stop
injection. Ctrl+C in the run client cancels, or call::

   ros2 service call /robot_1_/disturbance/stop std_srvs/srv/Trigger '{}'

Start standard MCAP recording before injection. The recorder includes disturbance
command, applied vehicle/arm state, status/configuration and task-wrench topics,
alongside ``dynamic_joint_states``. Injection does not start recording or write
separate CSV/JSON files. CmdReplay and the MCAP converter do not reapply recorded
disturbance tracks.

For an independent live observer, run ``ros2 run simlab disturbance_plot --help``.
Custom waveform registration is covered in :doc:`developer_guide`.

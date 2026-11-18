Developer Guide
===============

This page is for extending the stack while preserving the existing state
machine and runtime behavior.

Where Things Live
-----------------

Simulator package:

- ``uvms-simulator/bringup/launch/robot_system_multi_interface.launch.py``:
  main launch file and task selection.
- ``uvms-simulator/msg`` and ``uvms-simulator/srv``: simulator-owned dynamics,
  reset, and camera profile interfaces.
- ``uvms-simulator/hardware``: ros2_control hardware interfaces for real and
  simulated vehicle/manipulator.
- ``uvms-simulator/bringup/config``: launch-time controller-manager
  configuration.
- ``uvms-simulator/description``: xacro/URDF descriptions.
- ``uvms-simulator/hardware/gstreamer_camera_*``: GStreamer camera node and
  driver.
- ``uvms-simulator/bringup/sim_camera_renderer.py``: simulated camera renderer
  with PyVista and Open3D backends.

SimLab package:

- ``uvms-simlab/simlab/interactive_control.py``: RViz menu layout and UI
  callback glue.
- ``uvms-simlab/simlab/uvms_backend.py``: shared backend API used by RViz
  menus and external service clients.
- ``uvms-simlab/msg``, ``uvms-simlab/srv``, and ``uvms-simlab/action``:
  SimLab-owned reference, performance, backend command, and planner action
  interfaces.
- ``uvms-simlab/simlab/robot.py``: robot state machine, controller registry,
  planner/replay dispatch, joystick bridge, grasper handling, logging.
- ``uvms-simlab/simlab/controllers``: controller implementations, including
  ``CmdReplay``.
- ``uvms-simlab/simlab/utils``: shared helper modules. ``geometry.py`` contains
  bounding-volume and marker geometry helpers, ``frames.py`` contains frame and
  pose conversion helpers, ``meshes.py`` contains URDF mesh/FCL/point-cloud
  helpers, and ``interactive_markers.py`` contains RViz interactive-marker helpers.
- ``uvms-simlab/resource/playback_profile``: replay profiles.
- ``uvms-simlab/resource/dynamics_profiles``: whole-robot dynamics parameter
  profiles selected by replay/reset workflows.
- ``uvms-simlab/resource/model_functions``: generated CasADi/shared-library
  model functions grouped by ``arm``, ``vehicle``, and ``whole_body``.
- ``uvms-simlab/simlab/planner_action_server.py`` and
  ``planner_action_client.py``: planner action server/client.

Runtime Modes
-------------

The main runtime modes are:

- ``TELEOP``: joystick/manual command publishing.
- ``PLANNER``: feedback control execution and planner trajectories.
- ``REPLAY``: CSV command replay.
- ``REPLAY_SETTLE``: hardware-only pre-replay controller settling.

Adding a Controller
-------------------

Controller choices are class-based. They live under
``uvms-simlab/simlab/controllers`` and are registered through
``DEFAULT_CONTROLLER_CLASSES``.

To add a controller:

- Create a controller file, for example ``simlab/controllers/my_controller.py``.
- Inherit from ``ControllerTemplate``.
- Implement ``vehicle_controller(...)`` and ``arm_controller(...)``.
- Override ``reset_controller_state()`` if the controller retains integrators,
  previous commands, filters or learned residuals. Independent replay passes
  call this hook before execution; it must preserve configuration and gains.
- For runtime estimated-model configuration, implement ``set_model_profile``
  and declare ``configuration_fields`` or override ``configuration_snapshot``.
  Report effective values, not only a profile label. Unknown plugins reject
  model configuration by default. Do not modify shared parameter classes or
  another robot's model when applying an instance-specific profile.
- Add the class to ``DEFAULT_CONTROLLER_CLASSES`` in
  ``simlab/controllers/__init__.py``.

The control loop calls the currently selected controller with the latest
vehicle/manipulator state and target commands.

Built-in controller behavior is described in :doc:`controls_and_menus`.

Use an existing controller as the signature reference. Vehicle output is a
six-element body-FRD wrench; arm output follows e, d, c, b, a (gripper).
Use estimated feedback and keep model/gain state local to each instance.
Append your class to the existing registry, then rebuild SimLab and restart.

Adding a Planner
----------------

Motion planning lives under ``simlab/motion_planning``. Return a typed
``MotionPlanResult``. The current ``PlanVehicle`` action accepts ``PATH`` only;
the selected trajectory generator adds timing.

Planner choices are class-based. They live under
``uvms-simlab/simlab/motion_planning/planners``. The RViz menu and planner
action server both read from ``DEFAULT_PLANNER_CLASSES``.

To add a planner:

- Create a planner file, for example
  ``simlab/motion_planning/planners/my_planner.py``.
- Inherit from ``PlannerTemplate``.
- Add the class to ``DEFAULT_PLANNER_CLASSES`` in
  ``simlab/motion_planning/planners/__init__.py``.
- Return a ``MotionPlanResult``. Planner plugins should not return raw result
  dictionaries.

Implement ``plan_vehicle`` with the signature in ``PlannerTemplate``, including
``dynamic_obstacle_prediction_speed``. Validate collision clearance before
returning success; use the existing OMPL adapters as examples. Append the class
to the registry rather than replacing built-in entries.

Adding a Trajectory Generator
-----------------------------

Vehicle trajectory generators are class-based plugins under
``uvms-simlab/simlab/motion_planning/trajectory_generators`` and are registered
through ``DEFAULT_VEHICLE_TRAJECTORY_GENERATOR_CLASSES``. The selected generator
turns a planner path into the pose, velocity, and acceleration references
tracked by the active controller.

To add a trajectory generator:

- Create a file, for example
  ``simlab/motion_planning/trajectory_generators/my_generator.py``.
- Inherit from ``VehicleTrajectoryGeneratorTemplate``.
- Implement ``current_reference()``, ``start_from_path(...)``,
  ``update(yaw_blend_factor)`` and ``close()``. Provide read-only timed
  ``preview_samples(...)`` for clearance monitoring.
- Add the class to ``DEFAULT_VEHICLE_TRAJECTORY_GENERATOR_CLASSES`` in
  ``simlab/motion_planning/trajectory_generators/__init__.py``.
- Select it with the ``vehicle_trajectory_generator`` ROS parameter.

Example selection:

.. code-block:: bash

   ros2 run simlab interactive_controller --ros-args \
     -p vehicle_trajectory_generator:=ruckig

The default generator is ``ruckig``.

Adding a Dynamics Backend or Robot Interface
--------------------------------------------

1. Implement the real/sim driver under ``uvms-simulator/hardware``.
2. Define joints, sensors, thrusters and frames under ``description``.
3. Wire hardware selection in ``bringup/launch`` and controllers in
   ``bringup/config``.
4. Expose the shared state/command interfaces, then add SimLab readers only
   where the contract requires them.

Specify frames, units, joint order and signs. Keep sensor conversion, actuator
limits and allocation in the driver. High-level consumers use estimated state;
physical simulation state is reserved for plant integration and evaluation.

Validate interface availability, frame conversions, fresh feedback, zero command
and single-axis allocation before closed-loop operation. Simulation and hardware
should use the same names when they represent the same quantity.

Kinematics and model integration
--------------------------------

``motion_planning/kinematics/forward`` loads FK exports; ``inverse`` registers
whole-body and manipulator solvers. Implement ``WholeBodyIKTemplate`` or
``ManipulatorIKTemplate`` and add the class to ``DEFAULT_IK_CLASSES``.
Use typed requests and return ``IKResult``; failures must not replace targets.

``Robot`` construction selects ``whole_body_ik_name`` and ``manipulator_ik_name``.
Defaults are ``whole_body_endpoint`` and ``manipulator_position``; these are not
launch arguments. Whole-body results are checked against FK and joint bounds.
Arm-only IK solves position with fixed vehicle pose; orientation is unconstrained
and finite output does not guarantee endpoint accuracy.
An IK endpoint is not a collision-checked path.

Generate models in the existing diff_uv and Floating-KinDyn-Graph notebooks.
Consumers load exported ``.casadi``/``.so`` artifacts; do not add source-runtime
imports or parallel export scripts. Use CasADi 3.7.2 for public serialized models
and validate loads/signatures before replacing them. Endpoint IK and its geometric
Jacobian come from ``usage/robot_inverse_kinematics_analysis.ipynb``. Arm EKF
exports come from ``usage/joint_ekf.ipynb``; vehicle exports use diff_uv
``usage/code_generation.ipynb``. Install matching libraries and adapters together.

Custom disturbance waveforms
----------------------------

Add a class in ``simlab/disturbances/my_waveform.py``::

   import math
   from simlab.disturbances.base import DisturbanceWaveform

   class Cosine(DisturbanceWaveform):
       registry_name = "cosine"

       @staticmethod
       def sample(c):
           return math.cos(2 * math.pi * c.frequency * c.elapsed
                           + math.radians(c.phase_deg))

Import it in ``simlab/disturbances/local.py`` and add it to ``LOCAL_WAVEFORMS``.
Rebuild and restart, then use ``--waveform cosine``. Installed packages can
instead register an entry point in the ``simlab.disturbances`` group.

Return a finite dimensionless value; the runtime applies amplitude, delay and
run boundaries. Sampling must be deterministic, fast and nonblocking. Names must
be unique. Optional hooks customize validation, plot style and metadata; see
``DisturbanceWaveform``. Plugins run trusted Python in the node process.

Obstacle sources
----------------

Implement the lifecycle in ``simlab/dynamic_obstacle_sources/behavior.py``:
``initialize``, ``step``, ``on_edit``, ``reset`` and ``close``. Select the source
with ``dynamic_obstacle_source``, ``obstacle_source_config`` and ``obstacle_seed``.
Sources receive scene/observation snapshots and control motion. Frontends own
geometry edits through ``EditDynamicObstacles``. Reset must restore seeded state.

Run tests
---------

Build both packages, then test with the generated ROS interfaces available::

   source install/setup.bash
   python3 -m pytest src/uvms-simlab/test -q
   colcon test --packages-select ros2_control_blue_reach_5
   colcon test-result --verbose

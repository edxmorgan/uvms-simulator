Developer Guide
===============

This page is for extending the stack while preserving the existing state
machine and runtime behavior.

Where Things Live
-----------------

Simulator package:

- ``uvms-simulator/bringup/launch/robot_system_multi_interface.launch.py``:
  main launch file and task selection.
- ``uvms-simulator/srv``: reset and dynamics service definitions.
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
- ``uvms-simlab/simlab/robot.py``: robot state machine, controller registry,
  planner/replay dispatch, joystick bridge, grasper handling, logging.
- ``uvms-simlab/simlab/controllers``: controller implementations, including
  ``CmdReplay``.
- ``uvms-simlab/resource/playback_profile``: replay profiles.
- ``uvms-simlab/resource/dynamics_profiles``: whole-robot dynamics parameter
  profiles selected by replay/reset workflows.
- ``uvms-simlab/resource/model_functions``: generated CasADi/shared-library
  model functions grouped by ``arm``, ``vehicle``, and ``whole_body``.
- ``uvms-simlab/simlab/planner_action_server.py`` and
  ``planner_action_client.py``: planner action server/client.

State Machine Rules
-------------------

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
- Add the class to ``DEFAULT_CONTROLLER_CLASSES`` in
  ``simlab/controllers/__init__.py``.

The control loop calls the currently selected controller with the latest
vehicle/manipulator state and target commands.

Semantics of built-in controllers:

- ``PID`` combines feedback terms with hydrostatic restoring compensation on
  the vehicle path. The manipulator path is joint-space feedback with
  configured command limits.
- ``InvDyn`` implements inverse-dynamics control in the computed-torque sense:
  desired state and desired acceleration are mapped through an estimated model
  to actuation, with feedback terms correcting model and state-estimation
  errors.

Example:

.. code-block:: python

   import numpy as np

   from simlab.controllers.base import ControllerTemplate


   class MyController(ControllerTemplate):
       registry_name = "MyController"

       def __init__(self, node, arm_dof=4, robot_prefix=""):
           super().__init__(node, arm_dof=arm_dof, robot_prefix=robot_prefix)
           self.vehicle_kp = np.array([8.0, 8.0, 10.0, 1.0, 1.0, 2.0])
           self.arm_kp = np.array([2.0, 2.0, 1.0, 1.0, 0.0])

       def vehicle_controller(
           self,
           state,
           target_pos,
           target_vel,
           target_acc,
           dt,
       ):
           state = self.vector(state, 12, "state")
           target_pos = self.vector(target_pos, 6, "target_pos")
           target_vel = self.vector(target_vel, 6, "target_vel")
           error = target_pos - state[:6]
           damping = target_vel - state[6:12]
           return self.vehicle_kp * error + 0.1 * damping

       def arm_controller(
           self,
           q,
           q_dot,
           q_ref,
           dq_ref,
           ddq_ref,
           dt,
       ):
           q = self.arm_vector(q, "q")
           q_dot = self.arm_vector(q_dot, "q_dot")
           q_ref = self.arm_vector(q_ref, "q_ref")
           dq_ref = self.arm_vector(dq_ref, "dq_ref")
           return self.arm_kp * (q_ref - q) + 0.05 * (dq_ref - q_dot)

Then register it:

.. code-block:: python

   from simlab.controllers.my_controller import MyController

   DEFAULT_CONTROLLER_CLASSES = [
       LowLevelPidController,
       LowLevelInvDynController,
       CmdReplayController,
       MyController,
   ]

After rebuilding, ``MyController`` appears in the RViz controller menu.

Adding a Planner
----------------

Planner choices are class-based, like controllers. They live under
``uvms-simlab/simlab/planners``. The RViz menu and planner action server both
read from ``DEFAULT_PLANNER_CLASSES``.

To add a planner:

- Create a planner file, for example ``simlab/planners/my_planner.py``.
- Inherit from ``PlannerTemplate``.
- Add the class to ``DEFAULT_PLANNER_CLASSES`` in
  ``simlab/planners/__init__.py``.
- Confirm the result dict contains ``xyz``, ``quat_wxyz``, ``count``,
  ``is_success``, ``path_length_cost``, ``geom_length``, and ``message``.

Example:

.. code-block:: python

   import numpy as np

   from simlab.planners.base import PlannerTemplate


   class MyPlanner(PlannerTemplate):
       registry_name = "MyPlanner"
       visible = True

       def plan_vehicle(
           self,
           *,
           start_xyz,
           start_quat_wxyz,
           goal_xyz,
           goal_quat_wxyz,
           time_limit,
           robot_collision_radius,
       ):
           xyz = np.asarray([start_xyz, goal_xyz], dtype=float)
           quat = np.asarray([start_quat_wxyz, goal_quat_wxyz], dtype=float)
           length = float(np.linalg.norm(xyz[-1] - xyz[0]))
           return {
               "is_success": True,
               "xyz": xyz,
               "quat_wxyz": quat,
               "count": int(xyz.shape[0]),
               "path_length_cost": length,
               "geom_length": length,
               "message": "MyPlanner returned a straight-line path.",
           }

Then register it:

.. code-block:: python

   from simlab.planners.my_planner import MyPlanner

   DEFAULT_PLANNER_CLASSES = [
       RrtStarPlanner,
       BitStarPlanner,
       RrtConnectPlanner,
       MyPlanner,
   ]

Adding a Dynamics Backend or Robot Interface
--------------------------------------------

Use this path when the robot itself changes: vehicle geometry, manipulator,
thruster layout, actuator protocol, sensor suite, estimator, or dynamics
model. A backend/interface change defines what state the rest of the stack can
read and how commands reach the real or simulated actuators.

ROS 2 control interface basics:

- A hardware interface is the boundary between ROS controllers and the robot or
  simulator.
- State interfaces are values the robot exposes to ROS: vehicle pose, body
  velocity, body acceleration, joint position, joint velocity, joint
  acceleration, measured effort, and similar feedback.
- Command interfaces are values ROS controllers write: vehicle wrench,
  thruster commands, joint effort commands, grasper commands, or other actuator
  inputs.
- Simulated and real implementations should use the same interface names when
  they represent the same physical quantity. That is what allows the same
  controller and logger to run in simulation, hardware, or mixed setups.

State and command contract:

- Decide the vehicle state convention: position/orientation frame, body
  velocity frame, acceleration frame, units, and sign convention.
- Decide the manipulator convention: joint names, joint ordering, units,
  encoder signs, zero offsets, and grasper treatment.
- Decide the command convention: vehicle wrench ordering, thruster command
  ordering, manipulator effort ordering, actuator limits, and saturation
  behavior.
- Expose enough state for the rest of the stack to avoid guessing. If
  acceleration or measured effort is available, publish it as a state interface
  rather than making controllers or loggers reconstruct it.

Vehicle command allocation:

- The high-level vehicle controller produces a 6-DOF command such as
  ``[Fx, Fy, Fz, Tx, Ty, Tz]``.
- The vehicle interface must map that command to the actual thrusters for the
  specific vehicle.
- The allocation must account for thruster position, thruster direction,
  command limits, deadband, PWM/force conversion, and actuator sign.
- Validate allocation with single-axis tests: pure surge, pure heave, pure yaw,
  and zero command.

Manipulator command allocation:

- Keep joint names and ordering explicit. For the current Alpha manipulator
  convention, the arm joints are ``axis_e``, ``axis_d``, ``axis_c``,
  ``axis_b``, and grasper ``axis_a``.
- Map controller effort commands to actuator commands using that explicit
  ordering.
- Handle actuator limits, encoder signs, gear ratios, and zero offsets inside
  the robot interface.
- If effort sensing is available, expose measured effort. Otherwise expose the
  best available estimate and make that clear in the interface documentation.

Sensor fusion and state estimation:

- Decide where sensor fusion lives. Typical inputs are IMU, depth/pressure,
  DVL, odometry, and actuator feedback.
- Publish one coherent estimated state to the rest of the stack instead of
  making each controller fuse sensors independently.
- Keep frame conversion in the interface or estimator layer. Controllers should
  receive state in the convention they expect.
- Use estimator/source timestamps when latency matters.

Dynamics model integration:

- A simulator backend implements state-update logic: current state plus command
  produces the next state.
- A hardware backend needs driver logic: read sensors, decode state, convert
  commands, and write actuator commands.
- If a dynamics model is used, document the input ordering, output ordering,
  frames, units, and signs.
- Connect vehicle model forces to thruster allocation and manipulator model
  efforts to joint/actuator mapping.

Where to change code:

- Add or modify robot interfaces under ``uvms-simulator/hardware``.
- Add or modify xacro/URDF joints, transmissions, thrusters, sensors, and
  frames under ``uvms-simulator/description``.
- Add controller-manager configuration under ``uvms-simulator/bringup/config``.
- Add launch arguments and hardware selection wiring in
  ``uvms-simulator/bringup/launch/robot_system_multi_interface.launch.py``.
- Add SimLab-side readers, wrappers, or parameters under ``uvms-simlab/simlab``
  only when the low-level state/command contract is clear.

Backend validation sequence:

- Bring up ros2_control and verify the expected state and command interfaces
  exist.
- Run open-loop single-axis actuator tests with conservative limits.
- Verify frame conventions using known vehicle poses and known joint
  positions.
- Verify state-estimation output with actuators disabled.
- Test zero command, then low-amplitude command profiles, before enabling
  planner execution.

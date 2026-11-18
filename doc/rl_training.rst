Batch RL Training
=================

``uvms_rl`` is the Python training API installed by ``simlab``. It runs many
UVMS environments through the batched simulator core without launching ROS
nodes. ``ros2_control`` owns the hardware integration path; RL tasks, rewards,
resets, and experiment configs live in ``uvms_rl``.

Quick Start
-----------

.. code-block:: bash

   cd ~/ros_ws
   colcon build --packages-select ros2_control_blue_reach_5 simlab
   source install/setup.bash

   ros2 run simlab uvms_rl_train_rsl \
     --config hover_vehicle_stage1_fixed \
     --backend gpu \
     --device cuda \
     --num-envs 2048

``hover_vehicle_stage1_fixed`` is the first curriculum experiment. It trains a
fixed-target hover policy through the GPU batch backend and writes logs and
checkpoints under ``recordings/rl_runs`` unless ``--log-dir`` is supplied.

Python Training Loop
--------------------

.. code-block:: python

   import numpy as np
   from uvms_rl import UvmsBatchEnv
   from uvms_rl.config import load_experiment


   experiment = load_experiment("hover_vehicle")
   env_cfg = experiment.config["env"]
   task_cfg = experiment.config["task"]

   env = UvmsBatchEnv(
       robot_count=env_cfg["robot_count"],
       control_dt=env_cfg["control_dt"],
       sim_dt=env_cfg["sim_dt"],
       max_episode_steps=env_cfg["max_episode_steps"],
       seed=env_cfg["seed"],
       backend=env_cfg.get("backend", "cpu"),
       dynamics_profile=env_cfg["dynamics_profile"],
       task=experiment.task_cls,
       task_config=task_cfg,
   )

   obs = env.reset()
   actions = np.zeros((env.robot_count, env.action_dim), dtype=np.float32)
   obs, rewards, dones, info = env.step(actions)

``obs`` is the task observation used by the policy. Use
``env.sim_observations()`` only when you need the raw simulator state.

RSL-RL Adapter
--------------

Use ``RslRlUvmsEnv`` when training with
`RSL-RL <https://github.com/leggedrobotics/rsl_rl>`_. The adapter wraps
``UvmsBatchEnv`` as RSL-RL's ``VecEnv`` interface and returns observations as a
``TensorDict`` with the ``policy`` observation group.

Install RSL-RL first if it is not already present:

.. code-block:: bash

   python3 -m pip install rsl-rl-lib

Create the adapter in Python with
``RslRlUvmsEnv.from_experiment("hover_vehicle")``. For a complete runner setup,
use ``uvms_rl/scripts/train_rsl.py`` and the experiment's ``trainer.rsl_rl`` config.

Use the Quick Start command for training. Add ``--iterations`` or ``--log-dir``
to override the experiment defaults; ``--help`` lists all options.

Completed environments reset automatically: ``dones`` marks the terminal
transition, while the returned observation belongs to the next episode.

Timing Contract
---------------

``control_dt`` is the policy-action interval; ``sim_dt`` is the internal
integration interval. One ``env.step`` holds the action for
``control_dt / sim_dt`` substeps. Set both explicitly when matching ROS timing.
For example, a 150 Hz action interval and 600 Hz integration use::

   env:
     control_dt: 0.006666666666666667
     sim_dt: 0.001666666666666667

Data Contract
-------------

The first dimension is the environment batch size.

.. code-block:: text

   task observations: [robot_count, task.policy_observation_dim]
   raw sim state:     [robot_count, 22]
   actions:           [robot_count, 11]
   rewards:           [robot_count]
   dones:             [robot_count]

Raw simulator state layout:

.. code-block:: text

   x, y, z, roll, pitch, yaw,
   u, v, w, p, q, r,
   arm_q1..arm_q5,
   arm_qd1..arm_qd5

The vehicle pose is the dynamics NED state. ``z`` is positive depth/down, so
underwater hover targets should use positive ``target_z`` values.

Action layout:

.. code-block:: text

   vehicle_wrench: force.x, force.y, force.z, torque.x, torque.y, torque.z
   arm_torque: arm_joint_1..arm_joint_5

Add an Experiment
-----------------

1. Copy an installed folder under ``uvms_rl/experiments`` and rename it.
   Keep its ``trainer.rsl_rl`` block when using the packaged PPO runner.
2. Put reusable task logic in ``uvms_rl/tasks/my_task.py``. Implement the
   ``TaskBase`` methods: ``reset``, ``reset_indices``, ``policy_observation``,
   ``reward_done`` and ``policy_observation_dim``.
3. In the experiment's ``task.py``, import that class as ``Task``.
4. Edit ``config.yaml``: backend, dynamics profile, batch size, timing, episode
   length, seed and task-specific settings.
5. Rebuild and run::

      colcon build --packages-select simlab
      source install/setup.bash
      ros2 run simlab uvms_rl_train_rsl --config my_experiment --backend cpu --iterations 1

Task reset returns raw states shaped ``[N, 22]``; indexed reset returns
``[len(indices), 22]``. Policy observations are ``[N, policy_observation_dim]``;
rewards and dones are ``[N]``. Use ``hover_vehicle`` as the implementation example.

Backend Boundary
----------------

Dynamics use float64; policy observations use float32.

Set ``env.backend`` and ``env.dynamics_profile`` explicitly. GPU requires the
CUDA extension and PyTorch CUDA support; unavailable backends or profiles fail
at construction. CUDA build options are in :doc:`installation`.

Before training against the GPU backend, validate the installed GPU dynamics
once after building:

.. code-block:: bash

   ros2 run ros2_control_blue_reach_5 uvms_gpu_dynamics_correctness

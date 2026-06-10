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

   ros2 run simlab uvms_rl_random_policy --config hover_vehicle --steps 100

``hover_vehicle`` is the packaged smoke-test experiment. It runs random actions
against ``1024`` environments by default and prints observation, reward, done,
time, and task metric summaries.

Python Training Loop
--------------------

.. code-block:: python

   import numpy as np
   from uvms_rl import UvmsBatchEnv
   from uvms_rl.config import load_experiment_config


   cfg = load_experiment_config("hover_vehicle")
   env_cfg = cfg["env"]
   task_cfg = cfg["task"]

   env = UvmsBatchEnv(
       robot_count=env_cfg["robot_count"],
       dt=env_cfg["dt"],
       max_episode_steps=env_cfg["max_episode_steps"],
       seed=env_cfg["seed"],
       task=task_cfg["name"],
       task_config=task_cfg,
   )

   obs = env.reset()
   actions = np.zeros((env.robot_count, env.action_dim), dtype=np.float32)
   obs, rewards, dones, info = env.step(actions)

``obs`` is the task observation used by the policy. Use
``env.sim_observations()`` only when you need the raw simulator state.

Timing Contract
---------------

The ROS runtime uses ``controller_manager.update_rate`` as the
``ros2_control`` loop frequency. The hardware receives that loop period as
``period.seconds()`` in ``read`` and ``write``. Set the RL experiment ``dt`` to
``1.0 / update_rate`` when training a policy intended for the ROS runtime.

The packaged ``hover_vehicle`` experiment uses ``dt: 0.006666666666666667``,
matching the current ``150 Hz`` controller config.

Data Contract
-------------

The first dimension is the environment batch size.

.. code-block:: text

   task observations: [robot_count, task.policy_observation_dim]
   raw sim state:     [robot_count, 22]
   actions:           [robot_count, 13]
   rewards:           [robot_count]
   dones:             [robot_count]

Raw simulator state layout:

.. code-block:: text

   x, y, z, roll, pitch, yaw,
   u, v, w, p, q, r,
   arm_q1..arm_q5,
   arm_qd1..arm_qd5

Action layout:

.. code-block:: text

   vehicle_action_0..vehicle_action_7,
   arm_action_0..arm_action_4

Add an Experiment
-----------------

Add experiments in ``uvms-simlab/uvms_rl``.

1. Create ``uvms_rl/tasks/<task_name>.py`` with a ``TaskBase`` subclass:

.. code-block:: python

   class MyTask(TaskBase):
       name = "my_task"

       @property
       def policy_observation_dim(self) -> int:
           return 33

       def reset(self, env):
           # Return initial raw simulator state with shape [N, 22].
           ...

       def policy_observation(self, env, sim_obs, actions):
           # Return policy input with shape [N, policy_observation_dim].
           ...

       def reward_done(self, env, sim_obs, actions):
           # Return rewards [N], dones [N], and metric dict.
           ...

2. Register it in ``uvms_rl/tasks/registry.py``:

.. code-block:: python

   TASKS = {
       "hover_vehicle": HoverVehicleTask,
       "my_task": MyTask,
   }

3. Add ``uvms_rl/experiments/<experiment>.yaml``:

.. code-block:: yaml

   env:
     robot_count: 1024
     dt: 0.01
     max_episode_steps: 500
     seed: 7

   task:
     name: my_task
     target_x: [-2.0, 2.0]

4. Rebuild and run:

.. code-block:: bash

   cd ~/ros_ws
   colcon build --packages-select simlab
   source install/setup.bash
   ros2 run simlab uvms_rl_random_policy --config <experiment> --steps 100

Backend Boundary
----------------

The current backend is the CPU mock ``BatchUvmsCore``. The Python task API is
kept separate so a future GPU dynamics core can replace the backend without
rewriting task definitions or trainer code.

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

.. code-block:: python

   from rsl_rl.runners import OnPolicyRunner
   from uvms_rl.rsl_adapter import RslRlUvmsEnv


   env = RslRlUvmsEnv.from_experiment("hover_vehicle")
   runner = OnPolicyRunner(env, train_cfg, log_dir="runs/hover_vehicle", device=str(env.device))
   runner.learn(num_learning_iterations=1000)

The packaged command trains from the experiment's ``trainer.rsl_rl`` config:

.. code-block:: bash

   ros2 run simlab uvms_rl_train_rsl --config hover_vehicle_stage1_fixed --backend gpu --device cuda

Training logs and checkpoints are written under ``recordings/rl_runs`` by
default. Override the backend, number of environments, iteration count, or log
directory from the command line:

.. code-block:: bash

   ros2 run simlab uvms_rl_train_rsl \
     --config hover_vehicle_stage1_fixed \
     --backend gpu \
     --device cuda \
     --num-envs 2048 \
     --iterations 3 \
     --log-dir /tmp/uvms_rsl_hover_stage1

RSL-RL does not call ``reset`` before training, so the adapter resets once at
construction. During training it performs same-step resets for completed
environment rows: returned ``dones`` still mark the terminal transition, while
the returned observations for those rows are already the next episode's initial
observations.

Timing Contract
---------------

The RL API separates the policy/control step from the internal simulation step.
One ``env.step(action)`` applies one policy action, holds it constant, advances
the simulator for one or more internal substeps, then returns one observation,
reward, and done flag per environment.

.. code-block:: text

   control_dt = policy action interval
   sim_dt     = internal simulator integration interval
   substeps   = control_dt / sim_dt

For transfer to the ROS runtime, match ``control_dt`` to the action interval
used by the controller path. The ROS runtime uses
``controller_manager.update_rate`` as the ``ros2_control`` loop frequency, so
start with:

.. code-block:: text

   control_dt = 1.0 / controller_manager.update_rate

Set ``sim_dt`` to the simulator integration period. If the simulator integrates
at the same rate as the controller, use ``sim_dt: control_dt``. If it integrates
faster, use the smaller simulator period and let ``uvms_rl`` substep.

Example for a ``150 Hz`` controller and ``600 Hz`` simulator:

.. code-block:: yaml

   env:
     control_dt: 0.006666666666666667
     sim_dt: 0.001666666666666667

The packaged hover experiments set ``sim_dt`` equal to ``control_dt`` for a
simple one-substep rollout. For transfer experiments, set
both values explicitly to the controller and simulator rates you want to match.

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

Add experiments as folders under ``uvms-simlab/uvms_rl/experiments``. An
experiment should own its config. Reusable task implementations should live in
``uvms-simlab/uvms_rl/tasks`` so multiple curriculum stages can share the same
reward/reset logic without importing from each other.

.. code-block:: text

   uvms_rl/experiments/my_experiment/
   |-- config.yaml
   `-- task.py

1. Create a reusable task implementation, for example
   ``uvms_rl/tasks/my_task.py``:

.. code-block:: python

   from uvms_rl.task_base import TaskBase


   class Task(TaskBase):

       @property
       def policy_observation_dim(self) -> int:
           return 33

       def reset(self, env):
           # Return initial raw simulator state with shape [N, 22].
           ...

       def reset_indices(self, env, indices):
           # Return reset raw simulator states with shape [len(indices), 22].
           # Required when using RSL-RL same-step resets.
           ...

       def policy_observation(self, env, sim_obs, actions):
           # Return policy input with shape [N, policy_observation_dim].
           ...

       def reward_done(self, env, sim_obs, actions):
           # Return rewards [N], dones [N], and metric dict.
           ...

2. Point the experiment folder at that task with a tiny
   ``uvms_rl/experiments/my_experiment/task.py``:

.. code-block:: python

   from uvms_rl.tasks.my_task import Task

3. Add ``uvms_rl/experiments/my_experiment/config.yaml``:

.. code-block:: yaml

   env:
     backend: cpu
     dynamics_profile: dory_alpha
     robot_count: 1024
     control_dt: 0.006666666666666667
     sim_dt: 0.001666666666666667
     max_episode_steps: 500
     seed: 7

   task:
     target_x: [-2.0, 2.0]

4. Rebuild and run:

.. code-block:: bash

   cd ~/ros_ws
   colcon build --packages-select simlab
   source install/setup.bash
   ros2 run simlab uvms_rl_train_rsl --config my_experiment --backend cpu --iterations 1

Backend Boundary
----------------

The task API is independent of the dynamics backend. CPU dynamics are the
portable baseline. GPU dynamics use the CUDA batch core for high-throughput
rollouts under the same task, reward, reset, and trainer code.

Experiment configs should name the backend explicitly:

.. code-block:: yaml

   env:
     backend: cpu
     dynamics_profile: dory_alpha

Use ``backend: gpu`` only on machines where ``ros2_control_blue_reach_5`` was
built with CUDA dynamics enabled. It does not silently fall back to CPU; if the
GPU extension or PyTorch CUDA support is missing, construction fails so the
training run cannot accidentally use the wrong backend.

``dynamics_profile`` is also required. It names the same whole-robot dynamics
profile used by the hardware simulator replay path. Missing profiles fail at
environment construction time instead of silently falling back to hidden
defaults.

Before training against the GPU backend, validate the installed GPU dynamics
once after building:

.. code-block:: bash

   ros2 run ros2_control_blue_reach_5 uvms_gpu_dynamics_correctness
   ros2 run ros2_control_blue_reach_5 uvms_gpu_dynamics_benchmark

After those pass, switch the experiment config to ``backend: gpu`` and use the
same task and trainer code.

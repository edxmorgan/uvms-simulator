# uvms-simulator

ROS 2 simulation and hardware interfaces for a BlueROV2 Heavy and Reach Alpha 5.
Exported ROS package: `ros2_control_blue_reach_5`.

![UVMS environment](doc/uvms_env.png)

## Start here

1. [Install and build](doc/installation.rst) the workspace and companion SimLab package.
2. Follow [First Simulation](doc/userdoc.rst) to launch, move and record.
3. For physical robots, follow [Hardware Setup](doc/hil_setup.rst).

## Guides

- [Project overview](doc/simlab_overview.rst)
- [Controls and teleoperation](doc/controls_and_menus.rst)
- [Environment scenes](doc/environment_scenes.rst)
- [Sensors and cameras](doc/sensors_and_perception.rst)
- [Replay and disturbance experiments](doc/replay_and_experiments.rst)
- [Services, actions and topics](doc/services_and_interfaces.rst)
- [Developer guide](doc/developer_guide.rst)
- [Batch RL training](doc/rl_training.rst)

Update this repository and [uvms-simlab](https://github.com/edxmorgan/uvms-simlab)
together. Rebuild and restart after interface or model updates; version and
Git LFS requirements are in the installation guide.

## License

[AGPL-3.0-or-later](LICENSE). Imported assets retain their bundled licenses.

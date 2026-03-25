# uvms-simulator

ROS 2 `ros2_control` package for simulating and interfacing an underwater vehicle-manipulator system built around the BlueROV2 Heavy and Reach Alpha 5.

`uvms-simulator` provides:

- Simulated and hardware-backed `ros2_control` system interfaces
- Multi-robot UVMS spawning with per-robot controller generation
- Vehicle and manipulator dynamics backed by CasADi-generated libraries
- State estimation surfaces for vehicle and manipulator subsystems
- RViz, TF, PlotJuggler, rosbag, and HIL-oriented bringup utilities

The exported ROS 2 package name is `ros2_control_blue_reach_5`.

![UVMS environment](doc/uvms_env.png)

## Installation

### System requirements

- Ubuntu with ROS 2 Jazzy
- `colcon`, `rosdep`, and `vcs`
- CasADi installed and discoverable at runtime

Install core ROS dependencies:

```bash
sudo apt update
sudo apt install git-lfs \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gpio-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-forward-command-controller \
    ros-$ROS_DISTRO-force-torque-sensor-broadcaster \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-rviz-2d-overlay-plugins \
    ros-$ROS_DISTRO-rviz-2d-overlay-msgs \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-plotjuggler-ros
```

Install native libraries used by the vehicle interface:

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

If CasADi is installed outside the default linker path:

```bash
export LD_LIBRARY_PATH=/path/to/casadi/build/lib:$LD_LIBRARY_PATH
```

### Workspace setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/edxmorgan/uvms-simulator.git
vcs import < uvms-simulator/dependency_repos.repos

cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Optional companion package

Most operator-facing modes depend on `uvms_simlab`, which is pulled in by `dependency_repos.repos`.

It adds:

- Interactive RViz control
- OMPL planning and trajectory execution
- PS4 teleoperation
- Direct thruster keyboard control
- Collision and voxel visualization
- Mocap, logging, and benchmarking tools

Extra packages commonly needed by `uvms_simlab`:

```bash
sudo apt install ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-cv-bridge
pip install pyPS4Controller pynput scipy casadi ruckig python-fcl trimesh pycollada
```

Optional perception extras:

```bash
pip install torch torchvision timm opencv-python
```

## Quick start

Launch one fully simulated UVMS with RViz and the interactive control stack:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    sim_robot_count:=1 \
    task:=interactive \
    use_manipulator_hardware:=false \
    use_vehicle_hardware:=false
```

Other common modes:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py task:=manual
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py task:=joint
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py task:=direct_thrusters
```

Headless recording:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    gui:=false \
    task:=manual \
    record_data:=true
```

Hardware-in-the-loop:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_manipulator_hardware:=true \
    use_vehicle_hardware:=true
```

## What is included

### Runtime plugins

- `SimReachSystemMultiInterfaceHardware`
- `ReachSystemMultiInterfaceHardware`
- `SimVehicleSystemMultiInterfaceHardware`
- `BlueRovSystemMultiInterfaceHardware`

### Supported task modes

| Task | Purpose |
| --- | --- |
| `interactive` | RViz marker workflow with planner/action support |
| `manual` | PS4 joystick teleoperation |
| `joint` | Joint-space control integration point |
| `direct_thrusters` | Direct PWM-style thruster control |

### Main repository layout

```text
uvms-simulator/
├── bringup/        # Launch files and runtime config generation
├── description/    # URDF, Xacro, meshes, RViz, PlotJuggler assets
├── hardware/       # ros2_control plugins and transport code
├── casadi_lib/     # Precompiled CasADi shared libraries
├── doc/            # User and HIL documentation
└── README.md
```

## Documentation

- User guide: `doc/userdoc.rst`
- Hardware-in-the-loop notes: `doc/hil_setup.rst`
- Companion tools: <https://github.com/edxmorgan/uvms-simlab>

## Related projects

- <https://github.com/edxmorgan/diff_uv>
- <https://github.com/edxmorgan/floating-KinDyn>
- <https://control.ros.org>

## Citation

```bibtex
@misc{uvms-simulator,
  author = {Edward Morgan},
  title = {uvms-simulator: A ros2_control framework for simulating and interfacing with the BlueROV2 Heavy and Reach Alpha 5 manipulator},
  year = {2023},
  publisher = {GitHub},
  url = {https://github.com/edxmorgan/uvms-simulator}
}
```

## License

Apache-2.0

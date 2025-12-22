# Underwater Vehicle & Manipulator Simulator 🌊🤖

`uvms-simulator` is a ROS 2 control framework for simulating and interfacing with the **BlueROV2 Heavy** and **Reach Alpha 5** manipulator. The companion package [uvms_simlab](https://github.com/edxmorgan/uvms-simlab) extends the simulator with interactive control, planning, visualization, and logging, and has been used in extensive HIL experiments.

<img src="doc/uvms_env.png" width="840"/>

## Highlights

- 🌊 **Validated hydrodynamics** – full 6‑DoF rigid-body dynamics, added masses, and thruster models derived from `diff_uv` and `diff_uvms`.
- 🤖 **Multi-agent ready** – spin up several BlueROVs with manipulators sharing the same world and controllers.
- 🔧 **Hardware-in-the-loop** – flip launch args to connect to a real BlueROV2 Heavy, Reach Alpha 5, and A50 DVL.
- 🧭 **Sensor fusion stack** – onboard EKF fuses IMU, DVL, and model-based predictions for state estimates.
- 🛠️ **ROS 2 control native** – forward command controllers, custom hardware interfaces, and ros2_control configs included.
- 📊 **Data + viz tooling** – RViz configs, TF publishers, PlotJuggler layouts, and bag-ready topics to accelerate benchmarking.

## Simlab toolkit

`uvms_simlab` is a separate package in the same workspace. It adds the interactive control, planning, and logging layers listed below.

- 🌀🖱️ **Direct RViz manipulation** – interactive markers for vehicle and arm-base targets.
- 🗺️ **SE(3) planning + execution** – OMPL planners with FCL validity checks, Ruckig time-parameterized motion, and RViz path markers.
- 🧱 **Collision + clearance viz** – FCL contact markers, environment AABB bounds, and workspace/vehicle point clouds.
- 🎮 **Control modes** – PS4 teleop, joint-space torque control, or direct thruster PWM via keyboard.
- 📡 **Mocap integrations** – OptiTrack/mocap4r2 publishing with live pose/path trails.
- 🌊 **Environment + perception tools** – voxelized bathymetry clouds and optional RGB-to-pointcloud.
- 📓 **Data logging** – rosbag2 MCAP recorder via `record_data:=true`.

## Kinematics and Dynamics

This framework uses dynamic and kinematic models from:

- [diff_uv](https://github.com/edxmorgan/diff_uv)
- [diff_uvms](https://github.com/edxmorgan/diff_uvms)

These provide Jacobians, dynamic matrices, and model terms for control, stability analysis, and identification.

## Requirements

### Base simulator

1. **Install ROS 2**
   [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. **Check ROS distro**

   ```bash
   echo $ROS_DISTRO
   ```

   If empty:

   ```bash
   export ROS_DISTRO=jazzy
   ```

3. **Install system dependencies**

   ```bash
   sudo apt update
   ```

   ```bash
   sudo apt-get install git-lfs \
       ros-$ROS_DISTRO-hardware-interface ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-gpio-controllers \
       ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-joint-state-broadcaster ros-$ROS_DISTRO-rviz-imu-plugin \
       ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-forward-command-controller \
       ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-msgs \
       ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-force-torque-sensor-broadcaster ros-$ROS_DISTRO-tf-transformations \
       ros-$ROS_DISTRO-rviz-2d-overlay-plugins ros-$ROS_DISTRO-rviz-2d-overlay-msgs ros-$ROS_DISTRO-rosbag2
   ```

   ```bash
   sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
   sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
   ```

4. **CasADi**

   Follow installation here:
   [https://github.com/casadi/casadi/wiki/InstallationLinux](https://github.com/casadi/casadi/wiki/InstallationLinux)

   If needed:

   ```bash
   export LD_LIBRARY_PATH=/path/to/casadi/build/lib:$LD_LIBRARY_PATH
   ```

### uvms_simlab add-ons

**ROS packages (apt)**

- `ros-$ROS_DISTRO-interactive-markers`
- `ros-$ROS_DISTRO-cv-bridge`

**Python packages (pip)**

- `pyPS4Controller`, `pynput`, `scipy`, `casadi`, `ruckig`, `python-fcl`, `trimesh`, `pycollada`
- Optional perception extras: `torch`, `torchvision`, `timm`, `opencv-python` (MiDaS RGB-to-pointcloud)

**Other**

- OMPL with Python bindings (`install-ompl-ubuntu.sh --python` from Kavraki Lab works well).
- Optional hardware: BlueROV2 Heavy + Reach Alpha 5 + Blue Robotics A50 DVL (or any robot stack you map through the provided interfaces).

## Quick start ⚡

1. **Clone and import repositories**

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/edxmorgan/uvms-simulator.git
   vcs import < uvms-simulator/dependency_repos.repos
   ```

2. **Install missing dependencies**

   ```bash
   cd ..
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**

   ```bash
   colcon build
   source install/setup.bash
   ```

4. **Install uvms_simlab extras**

   ```bash
   cd ~/ros2_ws
   # uvms_simlab is already pulled into this workspace via vcs import.
   sudo apt install ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-cv-bridge

   sudo pip install pyPS4Controller pynput scipy casadi ruckig python-fcl trimesh pycollada
   # Optional: RGB-to-pointcloud (MiDaS)
   pip install torch torchvision timm opencv-python
   wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
   chmod u+x install-ompl-ubuntu.sh
   ./install-ompl-ubuntu.sh --python

   colcon build
   source install/setup.bash
   ```

## Launch recipes 🚢

**Interactive planner & RViz**

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    sim_robot_count:=1 task:=interactive \
    use_manipulator_hardware:=false use_vehicle_hardware:=false
```

**PS4 joystick teleop**

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    task:=manual
```

**Joint-space control**

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    task:=joint
```

**Direct thruster PWM (keyboard)**

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    task:=direct_thrusters
```

**Headless data collection**

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    gui:=false task:=manual record_data:=true
```

> 💡 Recording: `record_data:=true` starts rosbag2 MCAP logging to `uvms_bag_YYYYmmdd_HHMMSS`.

> 💡 Hardware swap: set `use_vehicle_hardware:=true` and `use_manipulator_hardware:=true` to put your BlueROV2 Heavy, Reach Alpha 5, and A50 DVL directly into the loop.

## Task modes 🎛️

| task | Simlab node | What it does | Input |
| --- | --- | --- | --- |
| `interactive` | `interactive_controller` | RViz markers + planner execution | RViz mouse/menus |
| `manual` | `joystick_controller` | PS4 teleop with PID control | PS4 controller |
| `joint` | `joint_controller` | Skeleton node for custom joint-space torque commands | Your node/scripts |
| `direct_thrusters` | `direct_thruster_controller` | Direct PWM commands | Keyboard |

## Project layout 🧭

```
uvms-simulator/
├── bringup/                     # Launch files and system bringup
├── description/                 # URDF/Xacro and model assets
├── hardware/                    # ros2_control hardware interfaces
├── doc/                         # User guide + HIL docs
├── casadi_lib/                  # CasADi binaries/helpers
└── ros2_control_blue_reach_5.xml # System config
```

```
uvms_simlab/
├── simlab/uvms_backend.py            # Core backend, FCL world, planners, TFs
├── simlab/interactive_control.py     # RViz markers + menu control
├── simlab/se3_ompl_planner.py        # OMPL SE(3) planning
├── simlab/cartesian_ruckig.py        # Ruckig trajectory generation
├── simlab/joystick_control.py        # PS4 teleop node
├── simlab/joint_control.py           # Joint-space torque control
├── simlab/direct_thruster_control.py # Thruster PWM keyboard control
├── simlab/collision_contact.py       # FCL contact markers + clearance
├── simlab/voxel_viz.py               # Bathymetry voxel clouds
├── simlab/bag_recorder.py            # rosbag2 MCAP recorder
├── simlab/rgb2cloudpoint.py          # RGB to pointcloud (MiDaS)
└── resource/                         # CasADi controllers + models
```

## Documentation

- 📘 **User Guide**: `doc/userdoc.rst`
- 🔌 **Hardware in the Loop Setup**: `doc/hil_setup.rst`
- 🎮 **Simlab tools and examples**: [https://github.com/edxmorgan/uvms-simlab](https://github.com/edxmorgan/uvms-simlab)

## Cite

```bibtex
@misc{uvms-simulator,
  author = {Edward Morgan},
  title = {uvms-simulator: A ros2_control framework for simulating and interfacing with the BlueROV2 Heavy and Reach Alpha 5 manipulator},
  year = {2023},
  publisher = {GitHub},
  url = {https://github.com/edxmorgan/uvms-simulator}
}
```

## Resources

- [https://control.ros.org](https://control.ros.org)
- [https://github.com/Reach-Robotics](https://github.com/Reach-Robotics)
- [https://github.com/BlueRobotics](https://github.com/BlueRobotics)

## Contributing & community 🤝

Have a planner, sensor, or teleop workflow that should live here? Open an issue or PR. Contributions of any size are welcome—bug fixes, controller ideas, sensor plugins, or documentation improvements.

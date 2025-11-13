# Underwater Vehicle and Manipulator Simulator and Hardware Framework 🌊🤖

ROS2 control framework for simulating and interfacing with the **BlueROV2 Heavy** and **Reach Alpha 5** manipulator.

<img src="doc/uvms_env.png" width="840"/>

---

## Features

* 🌊 **Realistic Dynamics**
  Physics based UVMS modeling built on validated dynamic equations.

* 🤖 **Multi Agent Simulation**
  Run multiple vehicles and manipulators in a shared environment.

* 🔧 **Hardware in the Loop**
  Interface with the BlueROV2 Heavy, A50 DVL, and Reach Alpha 5 for real world testing.

* 🧭 **Sensor Fusion**
  Internal Kalman filter for combined IMU, DVL, and model based state estimation.

---

## Kinematics and Dynamics

This framework uses dynamic and kinematic models from

* [diff_uv](https://github.com/edxmorgan/diff_uv)
* [diff_uvms](https://github.com/edxmorgan/diff_uvms)

These provide Jacobians, dynamic matrices, and model terms for control, stability analysis, and identification.

---

## Setup

### Prerequisites

1. **Install ROS2**
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
       ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-force-torque-sensor-broadcaster ros-$ROS_DISTRO-tf-transformations
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

---

## Installation

### Clone and import repositories

```bash
cd ros2_ws/src
git clone https://github.com/edxmorgan/uvms-simulator.git
vcs import < uvms-simulator/dependency_repos.repos
```

### Install missing dependencies

```bash
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build the workspace

```bash
colcon build
source install/setup.bash
```

---

## Documentation

* 📘 **User Guide**
  `doc/userdoc.rst`

* 🔌 **Hardware in the Loop Setup**
  `doc/hil_setup.rst`

* 🎮 **Interactive control and coverage examples**
  [https://github.com/edxmorgan/uvms_simlab](https://github.com/edxmorgan/uvms_simlab)

---

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

---

## Resources

* [https://control.ros.org](https://control.ros.org)
* [https://github.com/Reach-Robotics](https://github.com/Reach-Robotics)
* [https://github.com/BlueRobotics](https://github.com/BlueRobotics)

Contributions are welcome. Open an issue or submit a pull request.
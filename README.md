# Underwater Vehicle & Manipulator Simulator and Hardware Framework

A ROS2-based framework for simulating and interfacing with the **BlueROV Heavy** and **Reach Alpha 5** manipulator. Underwater dynamics Model and hardware integration support for both simulation and real-world testing.

---

## Features

- **Realistic Dynamics:** Close to real simulation of underwater vehicle and manipulator behavior.
- **Multi-Agent Support:** Simulate multiple agents in a shared environment.
- **Hardware-in-the-Loop Support:** Integrate BlueROV Heavy hardware, including A50 DVL and Reach Alpha 5 manipulator.

---

## Dynamics Foundation

- Extends [diff_uv](https://github.com/edxmorgan/diff_uv)
- Extends [diff_uvms](https://github.com/edxmorgan/diff_uvms)

---

## Setup

### Prerequisites

1. **Install ROS2:**  
   Follow the [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

2. **Ensure `$ROS_DISTRO` is Set:**  
   Confirm that the environment variable `$ROS_DISTRO` is correctly set:
   ```bash
   echo $ROS_DISTRO
   ```
   If not, set it to your ROS2 distribution (e.g., `humble`):
   ```bash
   export ROS_DISTRO=humble
   ```

3. **Install Required Dependencies:**  
   Use your package manager to install dependencies:
   ```bash
   sudo apt-get install git-lfs \
       ros-$ROS_DISTRO-hardware-interface ros-$ROS_DISTRO-xacro \
       ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-joint-state-broadcaster \
       ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-forward-command-controller \
       ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-msgs \
       ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-force-torque-sensor-broadcaster
   ```

4. **Install CasADi:**  
   Follow the [CasADi Installation Instructions](https://github.com/casadi/casadi/wiki/InstallationLinux).

5. **Clone Additional Packages:**  
   - [uvms_dynamics_ros2_control](https://github.com/edxmorgan/uvms_dynamics_ros2_control)  
   - [uvms_interfaces](https://github.com/edxmorgan/uvms_interfaces/tree/main)

### Installation Steps

1. **Clone the Repository:**  
   In your ROS2 workspace's `src` directory, clone the project:
   ```bash
   cd /absolute/path/to/your_ros2_workspace/src
   git clone https://github.com/edxmorgan/uvms-simulator.git
   ```

2. **Install Dependencies:**  
   From the root of your workspace, install any missing dependencies:
   ```bash
   cd /absolute/path/to/your_ros2_workspace
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace:**  
   Compile the project:
   ```bash
   colcon build
   ```

4. **Source the Setup File:**  
   Update your environment:
   ```bash
   source install/setup.bash
   ```

Replace `/absolute/path/to/your_ros2_workspace` with the actual absolute path to your ROS2 workspace.

---

## Documentation & Use Cases

- **User Guide:** [User Documentation](doc/userdoc.rst)
- **HIL Setup:** [Hardware-in-the-Loop Instructions](doc/hil_setup.rst)
- **Manual Control:** For PS4 joystick control and coverage examples, clone [uvms_simlab](https://github.com/edxmorgan/uvms_simlab).

---

## Resources & Contributing

- [ROS Control](https://control.ros.org/rolling/index.html)
- [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
- [Blue Robotics](https://github.com/Bluerobotics)

Contributions are welcome! Please open an issue or submit a pull request.

---

## License

This project is licensed under the [MIT License](LICENSE).

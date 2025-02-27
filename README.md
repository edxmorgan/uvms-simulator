# Underwater Vehicle & Manipulator Simulator and Hardware Framework

A ROS2-based framework for simulating and interfacing with the **BlueROV Heavy** and **Reach Alpha 5** manipulator. It combines realistic underwater dynamics with hardware integration for both simulation and real-world testing.

---

## Features

- **Realistic Dynamics:** Authentic simulation of underwater vehicle and manipulator behavior.
- **Multi-Agent Support:** Simulate multiple agents in a shared environment.
- **Hardware-in-the-Loop Support:** Integrate BlueROV Heavy hardware, including A50 DVL and Reach Alpha 5 manipulator.

---

## Dynamics Foundation

- Extends [diff_uv](https://github.com/edxmorgan/diff_uv)
- Extends [diff_uvms](https://github.com/edxmorgan/diff_uvms)

---

## Setup

### Prerequisites

1. **ROS2 Installation:**  
   [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

2. **Dependencies:**  
   ```bash
   sudo apt-get install git-lfs \
       ros-<distro>-hardware-interface ros-<distro>-xacro \
       ros-<distro>-controller-manager ros-<distro>-joint-state-broadcaster \
       ros-<distro>-joint-state-publisher-gui ros-<distro>-forward-command-controller \
       ros-<distro>-ros2-control ros-<distro>-mavros ros-<distro>-mavros-msgs \
       ros-<distro>-nav2-msgs ros-<distro>-force-torque-sensor-broadcaster
   ```
   Replace `<distro>` with your ROS distribution (e.g., humble).

3. **CasADi:**  
   Follow the [CasADi Installation Instructions](https://github.com/casadi/casadi/wiki/InstallationLinux).

4. **Additional Packages:**  
   - [uvms_dynamics_ros2_control](https://github.com/edxmorgan/uvms_dynamics_ros2_control)  
   - [uvms_interfaces](https://github.com/edxmorgan/uvms_interfaces/tree/main)

### Installation

1. Navigate to your ROS2 workspace:
   ```bash
   cd <ros_ws>/src
   ```
2. Clone the repository:
   ```bash
   git clone https://github.com/edxmorgan/uvms-simulator.git
   ```
3. Install dependencies:
   ```bash
   cd <ros_ws>
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. Build the workspace:
   ```bash
   colcon build
   ```
5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Documentation & Use Cases

- **User Guide:** [User Documentation](doc/userdoc.rst)
- **HIL Setup:** [Hardware-in-the-Loop Instructions](doc/hil_setup.rst)
- **Manual Control:** For PS4 joystick and coverage examples, clone [uvms_simlab](https://github.com/edxmorgan/uvms_simlab).

---

## Resources & Contributing

- [ROS Control](https://control.ros.org/rolling/index.html)
- [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
- [Blue Robotics](https://github.com/Bluerobotics)

Contributions are welcome! Please open an issue or submit a pull request.

---

## License

This project is licensed under the [MIT License](LICENSE).

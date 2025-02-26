# Underwater Vehicle and Manipulator Simulator

A simulator for the **BlueROV Heavy** equipped with a **Reach Alpha 5** manipulator based on the `ros2_control` framework. Integrates Thor Fossen’s underwater dynamics with Featherstone's manipulator dynamics algorithm for realistic simulations.

---

## Features

- **Realistic Dynamics:** Accurate simulation of underwater vehicle and manipulator behaviors.
- **Multi-Agent Support:** Simulate multiple agents within a shared environment.
- **Hardware-in-the-Loop Support:** Integrates BlueROV Heavy hardware including A50 DVL, and Reach Alpha 5 manipulator for realistic interaction and testing.
- **Video Demonstration:** [![Watch the Video](https://img.youtube.com/vi/VRJUbpdvPIM/0.jpg)](https://www.youtube.com/watch?v=VRJUbpdvPIM)

---

## Getting Started
- [ROS2 installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Prerequisites

Ensure the following dependencies are installed:

```bash
sudo apt-get install git-lfs
sudo apt-get install ros-<distro>-hardware-interface
sudo apt-get install ros-<distro>-xacro
sudo apt-get install ros-<distro>-controller-manager
sudo apt-get install ros-<distro>-joint-state-broadcaster
sudo apt-get install ros-<distro>-joint-state-publisher-gui
sudo apt-get install ros-<distro>-forward-command-controller
sudo apt-get install ros-<distro>-ros2-control
sudo apt-get install ros-<distro>-mavros
sudo apt-get install ros-<distro>-mavros-msgs
sudo apt-get install ros-<distro>-nav2-msgs
sudo apt-get install ros-<distro>-force-torque-sensor-broadcaster
# Replace "<distro>" with your distrubution of ros. Tested with humble and jazzy

# Install CasADi (required for dynamics and kinematics calculations)
# Follow the installation instructions on the CasADi wiki:
# https://github.com/casadi/casadi/wiki/InstallationLinux
```

Clone additional packages to your ROS2 workspace:

- [uvms_dynamics_ros2_control](https://github.com/edxmorgan/uvms_dynamics_ros2_control)
- [uvms_interfaces](https://github.com/edxmorgan/uvms_interfaces/tree/main)
- [sensor/GPIO transform broadcaster](https://github.com/edxmorgan/tf2_broadcaster)

### Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/edxmorgan/uvms-simulator.git
    ```
2. Navigate to the project directory:
    ```bash
    cd blue-heavy-dynamics-simulator
    ```
3. Install dependencies:
    ```bash
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

## Dynamics Foundation

This simulator incorporates and extends:

- Vehicle dynamics from [diff_uv](https://github.com/edxmorgan/diff_uv).
- Unified UVMS dynamics from [diff_uvms](https://github.com/edxmorgan/diff_uvms).

---

## Documentation

For detailed setup and usage instructions, refer to the [User Documentation](doc/userdoc.rst).

---

## Online Resources

- [ROS Control](https://control.ros.org/rolling/index.html)
- [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
- [Blue Robotics](https://github.com/Bluerobotics)

---

For detailed instructions on setting up HIL, refer to the [Hardware-in-the-Loop Documentation](doc/hil_setup.rst).

---

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for enhancements or bug fixes.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

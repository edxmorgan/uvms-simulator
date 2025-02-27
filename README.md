# Underwater Vehicle and Manipulator Simulator

A simulator for the **BlueROV Heavy** equipped with a **Reach Alpha 5** manipulator based on the `ros2_control` framework. Integrates Thor Fossen’s underwater dynamics with Featherstone's manipulator dynamics algorithm for realistic simulations.

---

## Features

- **Realistic Dynamics:** Close to real simulation of underwater vehicle and manipulator behaviors.
- **Multi-Agent Support:** Simulate multiple agents within a shared environment.
- **Hardware-in-the-Loop Support:** Integrates BlueROV Heavy hardware including A50 DVL, and Reach Alpha 5 manipulator for hardware interactions and testing.
<!-- - **Video Demonstration:** [![Watch the Video](https://img.youtube.com/vi/VRJUbpdvPIM/0.jpg)](https://www.youtube.com/watch?v=VRJUbpdvPIM) -->

---

## Dynamics Foundation

This simulator incorporates and extends:

- Vehicle dynamics from [diff_uv](https://github.com/edxmorgan/diff_uv).
- Unified UVMS dynamics from [diff_uvms](https://github.com/edxmorgan/diff_uvms).

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

### Installation
1. Change directory to your ros workspace
    ```bash
    cd <ros_ws>/src //replace ros_ws with your ros2 workspace directory name
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

## Documentation

For detailed setup and usage instructions, refer to the [User Documentation](doc/userdoc.rst).

---

## Use Case

If you intend to run the coverage example or manual control via PS4 joystick, please clone [uvms_simlab](https://github.com/edxmorgan/uvms_simlab
) into your workspace. 

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

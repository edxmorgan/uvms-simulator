# uvms-simulator

ROS 2 `ros2_control` package for simulating and interfacing a BlueROV2 Heavy + Reach Alpha 5 UVMS.

Exported package name: `ros2_control_blue_reach_5`

![UVMS environment](doc/uvms_env.png)

## Features

- Simulated and hardware-backed `ros2_control` system interfaces
- Multi-robot UVMS spawning
- CasADi-backed vehicle and manipulator dynamics
- RViz, TF, rosbag, PlotJuggler, and HIL bringup utilities
- Per-robot reset and release services for simulated systems

## Requirements

- Ubuntu with ROS 2 Jazzy
- `colcon`, `rosdep`, `vcs`
- CasADi available at runtime

Core dependencies:

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
    ros-$ROS_DISTRO-plotjuggler-ros \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

If CasADi is outside the default linker path:

```bash
export LD_LIBRARY_PATH=/path/to/casadi/build/lib:$LD_LIBRARY_PATH
```

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/edxmorgan/uvms-simulator.git
vcs import < uvms-simulator/dependency_repos.repos

cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Quick Start

Simulated UVMS:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    sim_robot_count:=1 \
    task:=interactive \
    use_manipulator_hardware:=false \
    use_vehicle_hardware:=false
```

Other modes:

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

## Reset

Reset simulated `robot_1_`:

```bash
ros2 service call /robot_1_reset_sim_vehicle std_srvs/srv/Trigger
ros2 service call /robot_1_reset_sim_manipulator std_srvs/srv/Trigger
```

Release commands after reset:

```bash
ros2 service call /robot_1_release_sim_vehicle std_srvs/srv/Trigger
ros2 service call /robot_1_release_sim_manipulator std_srvs/srv/Trigger
```

List available endpoints:

```bash
ros2 service list | grep -E 'reset_sim|release_sim'
```

## Notes

- `interactive`, `manual`, and several operator-facing workflows depend on the companion package `uvms_simlab`.
- Reset services are per robot prefix, for example `robot_2_` and `robot_3_`.
- Reset holds commands until the matching release service is called.

## Plugins

- `SimReachSystemMultiInterfaceHardware`
- `ReachSystemMultiInterfaceHardware`
- `SimVehicleSystemMultiInterfaceHardware`
- `BlueRovSystemMultiInterfaceHardware`

## Docs

- User guide: `doc/userdoc.rst`
- HIL notes: `doc/hil_setup.rst`
- Companion tools: <https://github.com/edxmorgan/uvms-simlab>

## Related

- <https://github.com/edxmorgan/diff_uv>
- <https://github.com/edxmorgan/floating-KinDyn>
- <https://control.ros.org>

## License

AGPL-3.0-or-later

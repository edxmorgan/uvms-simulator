# uvms-simulator

ROS 2 `ros2_control` package for simulating and interfacing a BlueROV2 Heavy + Reach Alpha 5 UVMS.

Exported package name: `ros2_control_blue_reach_5`

![UVMS environment](doc/uvms_env.png)

## Features

- Simulated and hardware-backed `ros2_control` system interfaces
- Multi-robot UVMS spawning
- CasADi-backed vehicle and manipulator dynamics
- RViz, TF, rosbag, PlotJuggler, and HIL bringup utilities
- Per-robot combined reset and release services for simulated UVMS robots

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
    gstreamer1.0-plugins-base \
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

The main launch file uses `serial_port:=auto` by default for the Reach Alpha manipulator. Auto mode tries `/dev/serial/by-id/*`, `/dev/ttyUSB*`, then `/dev/ttyACM*`. If the manipulator is on a known device, pass it explicitly:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_manipulator_hardware:=true \
    use_vehicle_hardware:=false \
    serial_port:=/dev/ttyUSB1
```

## Camera

The camera is an independent GStreamer node. It is not owned by the vehicle hardware interface.

Camera launch arguments:

- `launch_camera:=auto`: default. Starts the camera when `use_vehicle_hardware:=true` or `simulate_camera:=true`.
- `launch_camera:=true`: always starts the camera node.
- `launch_camera:=false`: disables the camera node.
- `simulate_camera:=false`: default. Uses the hardware UDP camera pipeline.
- `simulate_camera:=true`: uses a synthetic GStreamer test pattern and publishes it as `/alpha/image_raw`.
- `camera_pipeline:=""`: optional custom GStreamer pipeline. If set, it overrides the default pipeline. The pipeline must end with `appsink name=camera_sink`.

Real vehicle hardware starts the camera automatically:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_vehicle_hardware:=true
```

Use the real camera without real vehicle hardware:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_manipulator_hardware:=true \
    use_vehicle_hardware:=false \
    sim_robot_count:=0 \
    task:=manual \
    launch_camera:=true
```

Simulate the camera when no camera hardware is connected:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_manipulator_hardware:=true \
    use_vehicle_hardware:=false \
    sim_robot_count:=0 \
    task:=manual \
    simulate_camera:=true
```

The camera publishes `sensor_msgs/msg/Image` on `/alpha/image_raw`. RViz adds an enabled `video feed` Image display whenever the camera node is launched. To verify images are arriving:

```bash
ros2 topic hz /alpha/image_raw
```

Run only the standalone camera node:

```bash
ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
    -p image_topic:=/alpha/image_raw \
    -p frame_id:=camera_link
```

Run only the standalone simulated camera:

```bash
ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
    -p image_topic:=/alpha/image_raw \
    -p frame_id:=camera_link \
    -p pipeline:='videotestsrc is-live=true pattern=ball ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=(string)BGR ! appsink name=camera_sink emit-signals=true sync=false async=false max-buffers=1 drop=true'
```

## Reset

Reset simulated `robot_1_`:

```bash
ros2 service call /robot_1_reset_sim_uvms std_srvs/srv/Trigger
```

Release commands after reset:

```bash
ros2 service call /robot_1_release_sim_uvms std_srvs/srv/Trigger
```

List available endpoints:

```bash
ros2 service list | grep -E 'reset_sim|release_sim'
```

Low-level services are still available when needed:

```bash
ros2 service call /robot_1_reset_sim_manipulator std_srvs/srv/Trigger
ros2 service call /robot_1_reset_sim_vehicle std_srvs/srv/Trigger
ros2 service call /robot_1_release_sim_manipulator std_srvs/srv/Trigger
ros2 service call /robot_1_release_sim_vehicle std_srvs/srv/Trigger
```

## Sim Payload

The simulated manipulator exposes runtime payload state through `${prefix}_arm_IOs`:

- `payload.mass`
- `payload.Ixx`
- `payload.Iyy`
- `payload.Izz`

Update the simulated payload online with the per-robot service:

```bash
ros2 service call /robot_1_set_sim_payload ros2_control_blue_reach_5/srv/SetPayload \
  "{mass: 0.15, ixx: 0.0, iyy: 0.0, izz: 0.0}"
```

Notes:

- This service is available for the simulated manipulator hardware interface.
- `payload.mass` is used by the current sim manipulator dynamics path.
- `payload.Ixx`, `payload.Iyy`, and `payload.Izz` are exported as live state and can be updated online, but they are not yet consumed by the current dynamics model.
- Resetting the simulated manipulator clears the payload back to zero.

## Notes

- `interactive`, `manual`, and several operator-facing workflows depend on the companion package `uvms_simlab`.
- Reset and release services are per robot prefix, for example `robot_2_` and `robot_3_`.
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

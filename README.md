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
- CasADi built from source using the upstream instructions:
  <https://github.com/casadi/casadi/wiki/SourceBuild>

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
    gstreamer1.0-plugins-base \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

Python dependencies used by SimLab and the simulated camera renderer:

```bash
python3 -m pip install pyPS4Controller pynput scipy casadi ruckig \
    python-fcl trimesh pycollada pyvista open3d
```

Install PlotJuggler from Snap for the newer MCAP and scripting support used by
the launch workflow:

```bash
sudo snap install plotjuggler
```

The main launch file starts `/snap/bin/plotjuggler` when
`launch_plotjuggler:=true`. If `ros-$ROS_DISTRO-plotjuggler-ros` is also
installed, it can remain installed; the launch file does not use that older
executable.

After installing CasADi, make sure its shared libraries are on the runtime linker path. If CasADi is outside the default linker path:

```bash
export LD_LIBRARY_PATH=/path/to/casadi/build/lib:$LD_LIBRARY_PATH
```

## Install

```bash
cd ~/ros_ws/src
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

PlotJuggler:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    sim_robot_count:=1 \
    task:=interactive \
    launch_plotjuggler:=true
```

The launch file starts `/snap/bin/plotjuggler`, so the Snap release is used
instead of the older ROS package executable. Plot live topics such as
`/dynamic_joint_states`, `/<prefix>/reference/targets`, and
`/<prefix>/performance/controller`. For recorded experiments, open the
`~/ros_ws/recordings/mcap/uvms_bag_YYYYmmdd_HHMMSS` MCAP bag in PlotJuggler.

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

Camera launch arguments:

- `launch_camera:=true`: default. Starts the selected camera path.
- `launch_camera:=false`: disables camera nodes.
- `launch_camera:=auto`: starts the selected camera path when `camera_source` resolves to `sim`, `real`, or mixed real/sim camera sources.
- `camera_source:=auto`: default. Uses the simulated renderer for simulated vehicles, the real GStreamer camera for real vehicle hardware, and mixed selection when real vehicle hardware and simulated robots are launched together.
- `camera_source:=sim`: forces the simulated renderer to publish `/alpha/image_raw`.
- `camera_source:=real`: forces the GStreamer camera node to publish `/alpha/image_raw`.
- `camera_pipeline:=""`: optional custom GStreamer pipeline. If set, it overrides the default pipeline. The pipeline must end with `appsink name=camera_sink`.
- `sim_camera_renderer_backend:=pyvista`: default simulated renderer backend. Use `open3d` for comparison or fallback.
- `sim_camera_render_all_cameras:=true`: renders every simulated robot camera. Set `false` to render only the selected feed for lower load.
- `sim_camera_underwater_effect:=true`: applies the underwater tint/haze only when the simulated camera is below the water surface. Set `false` for raw geometry/debug views.

The vehicle hardware interface starts the camera automatically:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_vehicle_hardware:=true
```

Use the camera without the vehicle hardware interface:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_manipulator_hardware:=true \
    use_vehicle_hardware:=false \
    task:=manual \
    launch_camera:=true \
    camera_source:=real
```

Use the simulated camera renderer:

```bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
    use_vehicle_hardware:=false \
    sim_robot_count:=1 \
    task:=interactive \
    camera_source:=sim
```

The selected camera publishes `sensor_msgs/msg/Image` on `/alpha/image_raw`. RViz adds an enabled `video feed` Image display whenever camera launch is enabled. To verify images are arriving:

```bash
ros2 topic hz /alpha/image_raw
```

In mixed real/sim launches, per-robot camera topics are also available, for example `/robot_real/camera/image_raw` and `/robot_1/camera/image_raw`. Selecting a robot in the interactive menu updates the selected `/alpha` feed.

Camera intrinsics are published on the matching `sensor_msgs/msg/CameraInfo` topic, for example `/alpha/camera_info` or `/robot_1/camera/camera_info`. Camera extrinsics are provided through TF, for example `world -> robot_1_camera_link` or `robot_1_base_link -> robot_1_camera_link`.

Run only the standalone camera node:

```bash
ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
    -p image_topic:=/alpha/image_raw \
    -p frame_id:=camera_link
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

## Manipulator Payload and Gravity

The simulated manipulator exposes runtime payload and gravity state through `${prefix}_arm_IOs`:

- `gravity`
- `payload.mass`
- `payload.Ixx`
- `payload.Iyy`
- `payload.Izz`

Update simulator dynamics online with the coordinated per-robot service:

```bash
ros2 service call /robot_1_set_sim_uvms_dynamics ros2_control_blue_reach_5/srv/SetSimDynamics \
  "{use_coupled_dynamics: false, set_vehicle_dynamics: false, set_manipulator_dynamics: true, manipulator: {gravity_vector: [0.0, 0.0, 9.81], payload_mass: 0.15, payload_inertia: [0.0, 0.0, 0.0]}}"
```

Notes:

- The coordinated service forwards selected manipulator parameters to
  `/${prefix}set_sim_manipulator_dynamics` and vehicle parameters to
  `/${prefix}set_sim_vehicle_dynamics`.
- The runtime RViz `Dynamics Profile` menu applies named whole-robot profiles
  through the same coordinated service.
- `gravity` is used directly by the current sim manipulator dynamics path.
- `payload.mass` is used directly by the current sim manipulator dynamics path.
- `gravity` is also exported through `${prefix}_arm_IOs` for downstream consumers such as `uvms-simlab`.
- `payload.Ixx`, `payload.Iyy`, and `payload.Izz` are exported as live state and can be updated online, but they are not yet consumed by the current dynamics model.
- Reset requests can carry typed vehicle and manipulator dynamics parameters. Replay profiles and the RViz `Dynamics Profile` menu use this path to apply named robot dynamics profiles.

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

# uvms-simulator

ROS 2 `ros2_control` package for simulating and interfacing a BlueROV2 Heavy + Reach Alpha 5 UVMS.

Exported package name: `ros2_control_blue_reach_5`

## 1. Install the BlueBoat simulator on a fresh computer

There are two downloads. This **simulator** fork supplies the BlueBoat hull, thrusters, and simulated physics. The companion **SimLab** fork supplies the planner, controllers, and RViz menus. Together they already include the original Dory simulator. You’ll download both into one folder and build them together once.

These steps are for Ubuntu **24.04 Desktop on x86-64**. Open Terminal with **Ctrl+Alt+T**. Use the same terminal for steps 1–8. Paste one block at a time and wait for it to finish.

### Step 1. Prepare Ubuntu

Enable Ubuntu’s Universe package collection and install the tools you’ll use to download and compile software. The locale setting gives this terminal UTF-8 text support.

```bash
export LC_ALL=C.UTF-8
sudo apt update
sudo apt install -y software-properties-common curl ca-certificates git build-essential cmake pkg-config python3-venv python3-dev nano file
sudo add-apt-repository -y universe
grep '^Suites:' /etc/apt/sources.list.d/ubuntu.sources
```

On a standard installation you should see `noble`, `noble-updates`, and `noble-backports` together, plus a separate `noble-security` entry. Only if the line beginning with `Suites: noble` is missing either updates or backports, open the file with the command below, set that line to `Suites: noble noble-updates noble-backports`, and keep the security entry. In nano, save with **Ctrl+O**, **Enter**, then exit with **Ctrl+X**.

```bash
sudo nano /etc/apt/sources.list.d/ubuntu.sources
```

### Step 2. Install ROS 2 Jazzy Desktop

ROS connects the simulator, planner, and RViz. Download the ROS repository configuration for Ubuntu 24.04 (“Noble”), then let Ubuntu install the desktop and development tools.

```bash
curl -fL https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb -o /tmp/ros2-apt-source.deb
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-tf-transformations
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
```

You should see `jazzy`. Sourcing a setup file makes its tools available in the current terminal.

### Step 3. Install the simulator’s system packages

In **System Requirements**, run the **“Install core apt dependencies”** block and the **PlotJuggler Snap** block in the terminal from step 2. These install Git LFS, ROS controls, RViz plugins, camera libraries, and the plotting window that the mixed launch opens. Stop after the Snap command and resume here at **step 4**. Skip the custom linker-path example and the upstream **Workspace Setup**, stock clones, and `vcs import`. [Open System Requirements](https://edxmorgan.github.io/uvms-simulator/installation.html#system-requirements)

### Step 4. Build the C++ CasADi library for the physics simulator

The simulator links to a **C++ shared library**. Build CasADi 3.8.1 from source, install it under `/usr/local`, and refresh Ubuntu’s library index. Keep this download under `tools` so it stays separate from the two ROS packages in `src`.

```bash
mkdir -p ~/blueboat_ws/tools ~/blueboat_ws/src
cd ~/blueboat_ws/tools
git clone --depth 1 --branch 3.8.1 https://github.com/casadi/casadi.git casadi
cmake -S casadi -B casadi-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_EXAMPLES=OFF
cmake --build casadi-build --parallel 2
sudo cmake --install casadi-build
sudo ldconfig
ldconfig -p | grep libcasadi.so
```

You should see `libcasadi.so` with a path under `/usr/local/lib`.

### Step 5. Give SimLab a Python environment that can also see ROS

Create the environment with Ubuntu’s Python. The `--system-site-packages` option lets it use the ROS and Ubuntu Python packages already installed.

```bash
mkdir -p ~/.venvs
/usr/bin/python3 -m venv --system-site-packages ~/.venvs/blueboat
source ~/.venvs/blueboat/bin/activate
which python
```

You should see a path ending in `/.venvs/blueboat/bin/python`. In that same terminal, install the planning and camera packages. The **Python** CasADi package gives SimLab its Python interface. OMPL 1.7.0 provides the waypoint planner’s Python API. NumPy 1.x matches the ROS image libraries, and SciPy 1.14.1 supports the quaternion convention SimLab uses for heading.

```bash
python -m pip install "numpy>=1.26,<2" "casadi==3.8.1" "ompl==1.7.0" "ruckig==0.19.4" \
  pyPS4Controller pynput "scipy==1.14.1" python-fcl "trimesh==5.1.0" pycollada pyvista open3d urdf-parser-py
```

Point the OMPL Python loader at the same native library as its planner. The 1.7.0 Linux wheel bundles two copies. These links inside your new environment avoid loading both, which can crash Python on exit.

```bash
cd "$VIRTUAL_ENV/lib/python3.12/site-packages/ompl/util"
ln -s ../../ompl.libs/libboost*.so* .
ln -s ../../ompl.libs/libompl-*.so* libompl.so
python -c 'import rclpy, casadi, fcl, ruckig; from ompl.base import StateValidityCheckerFn; from urdf_parser_py.urdf import URDF; print("Planner imports OK")'
```

You should see `Planner imports OK`. This covers the upstream **Python Dependencies** and **OMPL Python Bindings** for this run. Continue here with step 6.

### Step 6. Download both BlueBoat forks

Git LFS retrieves large files, including the compiled BlueBoat dynamics library. Enable it **before** cloning. Each clone command selects the public `lance/blueboat` branch automatically.

```bash
git lfs install
cd ~/blueboat_ws/src
git clone --branch lance/blueboat https://github.com/Llong112/uvms-simulator_coral.git uvms-simulator
git clone --branch lance/blueboat https://github.com/Llong112/uvms-simlab_coral.git uvms-simlab
file uvms-simulator/casadi_lib/libBlueBoat_xnext.so
```

You should see `ELF 64-bit` and `x86-64` for the library. These are all the source downloads you need in `src`. The only active entry in this branch’s dependency manifest would download stock SimLab a second time.

### Step 7. Install the remaining ROS dependencies

`rosdep` reads the two packages’ dependency lists and asks Ubuntu to install what is missing. Initialize its list on first use, then update and install.

```bash
cd ~/blueboat_ws
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "casadi uvms_controller tf2_broadcaster"
```

The three skipped names are intentional. Step 4 installed `casadi` from source. `uvms_controller` and `tf2_broadcaster` are listed in the simulator manifest but unused by this simulated launch.

### Step 8. Build both packages and check where ROS finds them

`colcon` compiles the two packages under `src`. The CMake option skips building the project website.

```bash
cd ~/blueboat_ws
colcon build --base-paths src --cmake-args -DUVMS_BUILD_DOCS=OFF
source ~/blueboat_ws/install/setup.bash
ros2 pkg prefix ros2_control_blue_reach_5
ros2 pkg prefix simlab
```

You should see two finished packages and two paths in your home directory, ending in `blueboat_ws/install/ros2_control_blue_reach_5` and `blueboat_ws/install/simlab`. You’re ready to launch.

## 2. Run BlueBoat, switch vessels, and choose a controller

**Every time you run it**, open a terminal and paste this block. The first three lines load ROS, your Python packages, and the built workspace. The launch runs both vessels in simulation. You only need to clone and build during installation.

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/blueboat/bin/activate
source ~/blueboat_ws/install/setup.bash
ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
  task:=interactive sim_robot_count:=2 sim_robot_types:=dory,blueboat \
  use_vehicle_hardware:=false use_manipulator_hardware:=false
```

1. **Select BlueBoat.** RViz opens with Dory selected. Click **Interact** in the RViz toolbar. Right-click the **vehicle target marker** and choose **Robots → `robot_2_`**. You should see BlueBoat in the same environment. Both vessels stay loaded. Selecting one leaves it at its current pose.
2. **Choose a controller.** Right-click the target again and open **Robot Control → Controller → Surface** or **SurfaceInvDyn**. For the first run, choose **Surface**, the default. Reopen the menu to see the checkmark beside the active controller, or read its name on the `robot_2_` row of the **Robot Control Status** overlay.
3. **Drive about one metre.** With BlueBoat and the target pointing the same way, drag a straight translation handle to move the target about 1 m ahead of the bow (one square on the default grid). Leave the rotation rings alone so the heading stays the same. The boat should wait while you move the marker. Right-click the target and choose **Plan & Execute** once. BlueBoat should drive toward it. A goal that changes only heading currently gives a warning because turns in place have no timing policy.
4. **Switch back or stop.** On the target marker, choose **Robots → `robot_1_`** to see Dory and its arm again. Dory has its own default controller, **PID**. When you finish, press **Ctrl+C** in the launch terminal.

BlueBoat can move forward and back (surge), drift sideways (sway), and turn (yaw). Both controllers command forward force and yaw torque. The same allocator divides those commands between the two thrusters, which cannot push sideways independently. The same CasADi model simulates the resulting boat motion. [Dynamics generator](scripts/generate_blueboat_dynamics.py) · [Boat profile](https://github.com/Llong112/uvms-simlab_coral/blob/lance/blueboat/resource/dynamics_profiles/blueboat.json) · [Controller source](https://github.com/Llong112/uvms-simlab_coral/blob/lance/blueboat/simlab/controllers/blueboat.py)

| **BlueBoat controller** | **Controls classification** | **How it decides what to command** |
| --- | --- | --- |
| `Surface` (default) | **Feedback:** position guidance, P speed control, PD-like steering | It combines the planned travel direction with the boat’s measured position error to choose a heading. **Forward force** comes from the difference between desired and measured forward speed. **Yaw torque** comes from heading error and turn-rate error. If the boat points far from the desired course, it reduces forward speed while turning. |
| SurfaceInvDyn | **Pure model-based feedforward:** planar inverse dynamics | It takes the planned velocities and accelerations and uses the boat’s modeled mass, yaw inertia, drag, and coupling to calculate **forward force** and **yaw torque**. For a fixed plan and time, drifting off course does not change its command. |

![UVMS environment](doc/uvms_env.png)

## Features

- Simulated and hardware-backed `ros2_control` system interfaces
- Multi-robot UVMS spawning
- CasADi-backed vehicle and manipulator dynamics
- RViz, TF, rosbag, PlotJuggler, and HIL launch utilities
- Per-robot combined reset and release services for simulated UVMS robots
- Batched CPU/GPU UVMS core for large-N simulation and RL training through `uvms_rl`

## Requirements

The list below is the original project's dependency reference. For BlueBoat, the [numbered installation steps](#1-install-the-blueboat-simulator-on-a-fresh-computer) cover these dependencies in order.

- Ubuntu with ROS 2 Jazzy
- `colcon`, `rosdep`, `vcs`
- C++ CasADi built from source (see step 4 above)

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

CasADi's shared libraries must be on the runtime linker path. Step 4 uses the standard local install prefix and refreshes the linker index.

## Install

For this BlueBoat branch, follow [steps 1–8 above](#1-install-the-blueboat-simulator-on-a-fresh-computer). They replace the original project's stock checkout and dependency import. If both package checks in step 8 passed, go straight to [running the simulator](#2-run-blueboat-switch-vessels-and-choose-a-controller).

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

`gui:=false` disables GUI applications, including RViz, RViz overlays, and
PlotJuggler. Use `launch_rviz:=false` when you only want to suppress RViz while
leaving other GUI tools controlled by their own launch flags.

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

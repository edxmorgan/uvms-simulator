Sensors, Camera, and Perception
===============================

The stack exposes navigation sensor state, camera data, mocap, and
visualization streams through ROS topics. Some streams are physical sensor feeds
from hardware interfaces. Others are state-derived simulator outputs or
visualization aids.

Sensor Topic Overview
---------------------

Core sensor topics:

- ``/dynamic_joint_states``: central robot state stream. In simulation this is
  produced by the simulated hardware interfaces. In hardware or mixed launches,
  entries come from the active hardware and simulator interfaces.
- ``/mavros/imu/data``: hardware IMU input when the vehicle hardware interface
  is active.
- ``/dvl/twist``: DVL velocity output from the vehicle hardware interface.
- ``/alpha/image_raw`` and ``/alpha/camera_info``: selected camera feed used
  by RViz and perception utilities. The selected feed can come from the
  simulated renderer or the real GStreamer camera node.
- ``/mocap_pose``, ``/mocap_path``, ``/map_mocap_pose``,
  ``/map_mocap_path``: mocap-derived pose/path topics when ``use_mocap:=true``.

Vehicle navigation sensor state is also exported through
``/dynamic_joint_states``:

- IMU attitude: ``imu_roll``, ``imu_pitch``, ``imu_yaw`` and unwrap variants.
- IMU quaternion: ``imu_orientation_w/x/y/z``.
- IMU angular velocity: ``imu_angular_vel_x/y/z``.
- IMU linear acceleration: ``imu_linear_acceleration_x/y/z``.
- Pressure-derived depth: ``depth_from_pressure2``.
- DVL attitude: ``dvl_gyro_roll``, ``dvl_gyro_pitch``, ``dvl_gyro_yaw``.
- DVL velocity: ``dvl_speed_x``, ``dvl_speed_y``, ``dvl_speed_z``.

The vehicle state convention used by SimLab is NED position/orientation with
body-frame velocity and acceleration.

Simulated Sensors
-----------------

Simulation exposes sensor-like streams through ``ros2_control`` state
interfaces and broadcaster topics:

- Vehicle pose, body velocity, body acceleration, IMU state, DVL speed, and
  pressure-derived depth are available through ``/dynamic_joint_states``.
- Manipulator joint position, velocity, acceleration, and effort are available
  through ``/dynamic_joint_states``.
- Camera simulation is available through the SimLab camera renderer. It renders
  the simulated scene from each simulated robot camera frame and publishes ROS
  image topics.
- Environment, workspace, and vehicle-base pointcloud topics are visualization
  streams, not physical sensor models.

The simulated camera is useful for operator view, image transport, RViz display,
and downstream perception-node checks. It is not a calibrated underwater optical
model; turbidity, lighting, lens distortion, and camera dynamics are not modeled.

Launch Behavior
---------------

The main launch file controls camera ownership with ``camera_source``:

- ``camera_source:=auto``: default. Uses the simulated renderer when the vehicle
  is simulated, the real GStreamer camera when only real vehicle hardware owns
  the camera, and mixed selection when real vehicle hardware and simulated
  robots are launched together. A custom ``camera_pipeline`` selects the real
  camera path for the real robot.
- ``camera_source:=sim``: force the simulated renderer to own ``/alpha``.
- ``camera_source:=real``: force the GStreamer camera node to own ``/alpha``.
  If ``camera_pipeline`` is empty, the node uses its built-in UDP H264 camera
  pipeline.

Camera launch is controlled separately:

- ``launch_camera:=true``: default. Start the selected camera path.
- ``launch_camera:=false``: disable camera nodes.
- ``launch_camera:=auto``: start the selected camera path when the resolved
  source is ``sim``, ``real``, or mixed.

Additional arguments:

- ``camera_pipeline:=""``: optional custom GStreamer pipeline. If provided, it
  must end with ``appsink name=camera_sink``.
- ``sim_camera_width:=480``: rendered simulated camera image width.
- ``sim_camera_height:=360``: rendered simulated camera image height.
- ``sim_camera_rate:=5.0``: rendered simulated camera frame rate in Hz.

The camera is independent of the planner, replay, and controller menus. It can
be launched with hardware, with simulation, or with mixed hardware/simulation
interfaces.

Camera Output
-------------

The selected camera feed publishes:

.. code-block:: text

   /alpha/image_raw    sensor_msgs/msg/Image
   /alpha/camera_info  sensor_msgs/msg/CameraInfo

Simulated per-robot feeds publish:

.. code-block:: text

   /robot_1/camera/image_raw
   /robot_1/camera/camera_info
   /robot_2/camera/image_raw
   /robot_2/camera/camera_info

The simulated renderer creates these topics for each simulated robot. A feed is
rendered only while a client is subscribed to it, or while it is the selected
``/alpha`` feed.

The selected feed frame id follows the active source:

- ``robot_real_camera_link`` when the selected feed is the real camera.
- ``robot_N_camera_link`` when the selected feed is a simulated robot camera.

RViz automatically enables a ``video feed`` image display when camera launch is
enabled. In interactive mode, selecting a robot updates the selected ``/alpha``
feed. In mixed real/sim launches, selecting ``robot_real_`` shows the real
camera, while selecting a simulated robot shows that robot's rendered simulated
camera.

Hardware Camera
---------------

When ``use_vehicle_hardware:=true`` without simulated robots,
``camera_source:=auto`` selects the real GStreamer camera:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=true

To force the camera on without using the vehicle hardware interface:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=false \
       launch_camera:=true \
       camera_source:=real

Use this when you want to validate the camera node by itself or with a different
hardware/simulation combination. The default GStreamer pipeline is used unless
``camera_pipeline`` is provided.

Mixed Real/Sim Camera
---------------------

When real vehicle hardware and simulated robots are launched together,
``camera_source:=auto`` resolves to mixed mode:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_manipulator_hardware:=true \
       use_vehicle_hardware:=true \
       sim_robot_count:=1 \
       task:=interactive

The real camera publishes on ``/robot_real/camera/image_raw`` and simulated
robots publish on their own ``/robot_N/camera/image_raw`` topics. RViz consumes
the selected ``/alpha/image_raw`` feed. The interactive robot selection menu
updates which per-robot camera is mirrored to ``/alpha``.

For a simulated vehicle with a real manipulator and a real camera:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=false \
       use_manipulator_hardware:=true \
       sim_robot_count:=1 \
       task:=interactive \
       camera_source:=real

Simulated Camera
----------------

Pure simulation uses the simulated camera renderer by default:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=false \
       sim_robot_count:=1 \
       task:=interactive

Force the simulated renderer explicitly:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       camera_source:=sim

Verify image transport:

.. code-block:: shell

   ros2 topic hz /alpha/image_raw

Standalone Real Camera Node
---------------------------

Run the default camera pipeline directly:

.. code-block:: shell

   ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
       -p image_topic:=/alpha/image_raw \
       -p frame_id:=camera_link

Camera Mount and Lights
-----------------------

The vehicle hardware interface listens for camera mount pitch commands:

.. code-block:: text

   /alpha/cameraMountPitch

In PS4 ``Options`` mode:

- D-pad up/down publishes camera mount pitch commands.
- D-pad left/right publishes light commands.

The hardware interface clamps the camera mount PWM to the configured safe
range.

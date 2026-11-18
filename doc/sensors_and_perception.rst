Sensors and Perception
======================

Sensor topics
-------------

- ``/dynamic_joint_states``: vehicle and joint measurements, estimated states,
  applied efforts and estimator status. See :doc:`services_and_interfaces`.
- ``/dvl/twist``: hardware DVL velocity in ``<prefix>dvl_link``.
- ``/alpha/image_raw`` and ``/alpha/camera_info``: selected camera image and
  calibration. Per-robot feeds use ``/robot_N/camera/`` or ``/robot_real/camera/``.

Vehicle feedback uses NED pose and body-FRD velocity/acceleration. Raw sensor
fields retain their source conventions. ``depth_from_pressure2`` is positive
with increasing pressure relative to ``surface_pressure_pa``.

Camera selection
----------------

Append these options to the launch command in :doc:`userdoc`:

- ``launch_camera:=false``: disable cameras; the default is ``true``.
- ``camera_source:=auto``: select simulated, real or mixed feeds from the
  launched robots. Selecting a robot in RViz selects its ``/alpha`` feed.
- ``camera_source:=sim`` or ``real``: force the source.
- ``camera_pipeline:="... appsink name=camera_sink"``: custom real-camera pipeline.

With a real camera, ``camera_auto_destination:=true`` registers this computer
with BlueOS. Set it to ``false`` when managing destinations manually.
``camera_blueos_url`` defaults to ``http://192.168.2.2:6020``;
``camera_stream_name`` selects a named stream when multiple cameras exist.
Registration preserves other viewers but briefly restarts the selected stream.
Previous settings are saved under ``~/.ros/camera_destination_backups/``.

To run only the real-camera receiver::

   ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
     -p image_topic:=/alpha/image_raw -p frame_id:=camera_link

Check incoming images with ``ros2 topic hz /alpha/image_raw``.

Simulated camera settings
-------------------------

- ``sim_camera_renderer_backend:=pyvista``: default; ``open3d`` is also supported.
- ``sim_camera_width:=480``, ``sim_camera_height:=360``: image dimensions.
- ``sim_camera_rate:=5.0``: frame rate in Hz.
- ``sim_camera_max_mesh_triangles:=12000``: per-visual mesh cap; lower is faster.
- ``sim_camera_render_all_cameras:=false``: render only the selected feed;
  the default renders all simulated cameras.
- ``sim_camera_underwater_effect:=false``: disable underwater post-processing.

Underwater settings can also be changed at runtime::

   ros2 param set /sim_camera_renderer_node sim_camera_underwater_haze 0.35

Runtime parameter names have the ``sim_camera_`` prefix. Settings are ``underwater_effect``, ``underwater_haze``, ``underwater_tint``,
``underwater_blur``, ``underwater_noise`` and ``underwater_vignette``.
For typed profile updates, see ``SetSimCameraDynamics`` in
:doc:`services_and_interfaces`. The effect is an appearance model, not calibrated
underwater optics.

Camera frames and calibration
-----------------------------

``CameraInfo`` accompanies images. Simulation uses pinhole intrinsics and zero
distortion. Image frame IDs identify ``robot_N_camera_link`` or
``robot_real_camera_link``.

Normal TF reports the estimated pose::

   ros2 run tf2_ros tf2_echo world robot_1_camera_link

Use ``/simulation_truth/tf`` for physical camera-pose evaluation.

Camera-mount pitch commands use ``/alpha/cameraMountPitch``. Joystick mount/light
controls are listed in :doc:`controls_and_menus`.

Vehicle measurements and EKF
----------------------------

Use ``sim_vehicle_sensor_config:=/absolute/path/sensors.yaml`` for startup settings::

   /robot_1_BlueSystemMultiInterface_topics_interface:
     ros__parameters:
       sim_sensor_noise_enabled: true
       sim_sensor_noise_seed: 42
       sim_sensor_rates_hz: [10.0, 200.0, 10.0]
       sim_sensor_noise_std: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
       sim_raw_imu_noise_std: [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]

``sim_sensor_noise_std`` is depth (m), roll/pitch/yaw (rad), then body-FRD
velocity (m/s). ``sim_raw_imu_noise_std`` is gyro (rad/s), then specific force
(m/s²). Optional ``sim_dvl_covariance`` supplies a row-major 3×3 velocity
covariance. These are example white-noise settings, not calibrated Navigator,
Bar30 or A50 noise models; drift, latency and bottom-lock loss are not simulated.

In the menu, use **Robot Control → Sensor noise (sim) → On / Off**.
The ``/<prefix>set_sensor_noise`` SetBool service changes the same setting.
``sensor_noise_enabled`` telemetry confirms application at the next completed
step. This changes vehicle measurements only; it does not reset or retune the
EKF. Arm encoders are unaffected.

EKF measurement uncertainty is configured independently:

- ``depth_attitude_measurement_variances``: depth and roll/pitch/yaw variances.
- ``dvl_measurement_covariance``: row-major 3×3 body-FRD velocity covariance.
- ``raw_imu_measurement_variances``: gyro and specific-force variances.

Sensor mounts are read once from the robot URDF at driver initialization.
The IMU, DVL and pressure offsets feed the compiled measurement model and its
analytic Jacobian. DVL measurements include ``omega x r``; pressure measures
the depth of its sensor origin. The IMU model includes tangential and
centripetal acceleration at its origin. Simulation generates the same sensor
quantities from plant state before adding noise.

URDF frames use ROS FLU; the driver converts mounting geometry to the dynamics'
FRD convention once. MAVROS IMU measurements are already body-aligned and are
accepted only with a body-frame header. Their mounting rotation is not applied
again. A50 velocity and covariance are rotated together from the installation's
sensor-FRD convention to body FRD. The ROS ``/dvl/twist`` output is expressed in
its declared sensor-FLU frame.

Hardware and simulation DVL state interfaces both contain sensor-FRD velocity;
the estimator rotates these measurements into body FRD. Invalid hardware DVL
packets are excluded from both estimator updates and ROS twist publication.
After a simulation reset, pressure and DVL telemetry includes the mounting
offsets immediately. Accelerometer telemetry is unavailable (NaN) until the
first completed physical IMU sample; its EKF measurement mask remains cleared.

The IMU is located at base_link. The pressure sensor is at the rear of the top
enclosure (roughly 10–15 cm, exact XYZ unmeasured); its zero-offset value remains
an explicit uncalibrated assumption. Set their fixed-joint origins in
the selected vehicle Xacro when calibration is available. There is no separate
runtime IMU-offset parameter.

The EKF does not read the simulator's noise settings or estimate sensor noise
online. ``ekf_navigation_process_intensities`` and
``ekf_wrench_process_intensities`` configure process uncertainty. Optional
``ekf_wrench_process_covariance`` replaces the diagonal wrench block with a
row-major 6×6 continuous covariance in [force xyz, torque xyz] order.
``ekf_adapt_wrench_process_noise`` enables experimental process-noise adaptation;
it is off by default and cannot be combined with the full covariance option.

Hardware sensor configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use ``hardware_vehicle_sensor_config:=/absolute/path/sensors.yaml`` for hardware.
Set ``surface_pressure_pa`` from a surface measurement for Bar30 depth conversion;
startup does not auto-zero a potentially submerged robot. Configure attitude,
gyro and accelerometer variances from calibration data.

A50 velocity uses its reported full covariance. The installation assumes
body-FRD-aligned DVL axes; a different mounting needs both velocity and covariance
transformed. Invalid covariance or missing bottom lock excludes that observation.
IMU attitude comes from ``/mavros/imu/data``; fresh raw gyro and accelerometer
samples come from ``/mavros/imu/data_raw``. Missing channels are not fused as zero.
Full estimator validity requires fresh IMU, pressure and DVL streams.

Arm disturbance estimation
~~~~~~~~~~~~~~~~~~~~~~~~~~

Arm disturbance estimates cover joints e, d, c and b; the gripper is excluded.
Locked or limit-constrained joints cannot reliably reveal an external load;
check the published observability flags as well as estimator validity.
``arm_joint_locks:=false`` disables artificial locks in simulation, but retains
joint limits, friction and actuator limits. Hardware estimation is observation-only;
see :doc:`hil_setup` before interpreting physical torque estimates.

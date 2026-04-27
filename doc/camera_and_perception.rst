Camera and Perception
=====================

The stack provides a GStreamer camera node, RViz image display wiring, camera
mount/light commands for the real vehicle, and optional RGB-to-pointcloud
utilities.

Launch Behavior
---------------

The main launch file controls the camera with these arguments:

- ``launch_camera:=auto``: default. Starts the camera when
  ``use_vehicle_hardware:=true`` or ``simulate_camera:=true``.
- ``launch_camera:=true``: always start the camera node.
- ``launch_camera:=false``: disable the camera node.
- ``simulate_camera:=false``: default. Use the hardware camera pipeline.
- ``simulate_camera:=true``: use a synthetic GStreamer test-pattern pipeline.
- ``camera_pipeline:=""``: optional custom GStreamer pipeline. If provided, it
  replaces the default pipeline and must end with
  ``appsink name=camera_sink``.

The camera is independent of the planner, replay, and controller menus. It can
be launched with hardware, with simulation, or as a standalone node.

Camera Output
-------------

The camera node publishes:

.. code-block:: text

   /alpha/image_raw  sensor_msgs/msg/Image

The frame id is selected from the active robot prefix:

- ``robot_real_camera_link`` when ``use_vehicle_hardware:=true``.
- The first simulated robot camera link otherwise.

RViz automatically enables a ``video feed`` image display when the camera node
is launched.

Hardware Camera
---------------

With real vehicle hardware, the default ``launch_camera:=auto`` starts the
camera:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=true

To force the camera on without using the real vehicle hardware interface:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=false \
       launch_camera:=true

Use this when you want to validate the camera node by itself or with a different
hardware/simulation combination.

Simulated Camera
----------------

Use ``simulate_camera:=true`` when no camera is connected:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_vehicle_hardware:=false \
       simulate_camera:=true

Verify image transport:

.. code-block:: shell

   ros2 topic hz /alpha/image_raw

Standalone Camera Node
----------------------

Run the default camera pipeline directly:

.. code-block:: shell

   ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
       -p image_topic:=/alpha/image_raw \
       -p frame_id:=camera_link

Run a standalone synthetic pipeline:

.. code-block:: shell

   ros2 run ros2_control_blue_reach_5 gstreamer_camera_node --ros-args \
       -p image_topic:=/alpha/image_raw \
       -p frame_id:=camera_link \
       -p pipeline:='videotestsrc is-live=true pattern=ball ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=(string)BGR ! appsink name=camera_sink emit-signals=true sync=false async=false max-buffers=1 drop=true'

Camera Mount and Lights
-----------------------

The real vehicle hardware interface listens for camera mount pitch commands:

.. code-block:: text

   /alpha/cameraMountPitch

In PS4 ``Options`` mode:

- D-pad up/down publishes camera mount pitch commands.
- D-pad left/right publishes light commands.

The hardware interface clamps the camera mount PWM to the configured safe
range.

RGB-to-Pointcloud
-----------------

SimLab includes ``rgb2cloudpoint_publisher`` for RGB-derived pointcloud
visualization. It subscribes to:

.. code-block:: text

   /alpha/image_raw

This feature is optional and requires additional Python packages:

.. code-block:: shell

   python3 -m pip install torch torchvision timm opencv-python

Use it as a perception/debugging aid, not as a calibrated depth sensor.

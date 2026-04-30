Installation and Build
======================

Build the project as a ROS 2 workspace. A complete workspace normally contains:

- ``uvms-simulator``: exported ROS package
  ``ros2_control_blue_reach_5``.
- ``uvms-simlab``: exported ROS package ``simlab``.
- ``simlab_msgs``: shared SimLab action, message, and service interfaces.
- Dependency repositories from ``uvms-simulator/dependency_repos.repos``.

System Requirements
-------------------

- Ubuntu with ROS 2 Jazzy.
- ``colcon``, ``rosdep``, and ``vcs``.
- CasADi built from source and available at runtime.
- Git LFS for large model/resource files.

Install core apt dependencies:

.. code-block:: shell

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
       ros-$ROS_DISTRO-interactive-markers \
       ros-$ROS_DISTRO-cv-bridge \
       gstreamer1.0-plugins-base \
       libgstreamer1.0-dev \
       libgstreamer-plugins-base1.0-dev

If CasADi is outside the system linker path:

.. code-block:: shell

   export LD_LIBRARY_PATH=/path/to/casadi/build/lib:$LD_LIBRARY_PATH

Workspace Setup
---------------

.. code-block:: shell

   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src

   git clone https://github.com/edxmorgan/uvms-simulator.git
   git clone https://github.com/edxmorgan/uvms-simlab.git

   vcs import < uvms-simulator/dependency_repos.repos

   cd ~/ros_ws
   rosdep install --from-paths src --ignore-src -r -y

Python Dependencies
-------------------

Install Python packages used by SimLab for joystick control, planning, FCL,
Ruckig, and optional perception:

.. code-block:: shell

   python3 -m pip install pyPS4Controller pynput scipy casadi ruckig \
       python-fcl trimesh pycollada

Optional RGB-to-pointcloud support:

.. code-block:: shell

   python3 -m pip install torch torchvision timm opencv-python

OMPL Python Bindings
--------------------

The planner server uses OMPL through Python bindings. One working installation
route is the Kavraki Lab installer:

.. code-block:: shell

   wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
   chmod u+x install-ompl-ubuntu.sh
   ./install-ompl-ubuntu.sh --python

Build
-----

Build and source the whole workspace:

.. code-block:: shell

   cd ~/ros_ws
   colcon build
   source install/setup.bash

For focused development:

.. code-block:: shell

   colcon build --packages-select ros2_control_blue_reach_5 simlab simlab_msgs
   source install/setup.bash

Documentation Build
-------------------

Install Python docs dependencies:

.. code-block:: shell

   python3 -m pip install -r src/uvms-simulator/doc/requirements-docs.txt

Build the Sphinx site:

.. code-block:: shell

   cd ~/ros_ws/src/uvms-simulator/doc
   make html

Open:

.. code-block:: text

   doc/_build/html/index.html

GitHub Pages Deployment
-----------------------

The repository includes a GitHub Actions workflow for publishing the Sphinx
site to GitHub Pages:

.. code-block:: text

   .github/workflows/docs-pages.yml

On pushes to ``main``, the workflow builds ``doc/_build/html``
and deploys it as the Pages artifact. Pull requests build the docs but do not
deploy them.

First Launch Check
------------------

After the build succeeds, launch one simulated interactive UVMS:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_manipulator_hardware:=false \
       use_vehicle_hardware:=false \
       sim_robot_count:=1 \
       task:=interactive

For hardware launch flows, see :doc:`hil_setup`.
For first-use runtime steps, see :doc:`userdoc`.

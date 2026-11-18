Installation and Build
======================

System Requirements
-------------------

- Ubuntu with ROS 2 Jazzy.
- ``colcon``, ``rosdep``, and ``vcs``.
- CasADi C++ libraries available to CMake and the runtime linker.
- Python CasADi 3.7.2 for the bundled serialized models.
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
       ros-$ROS_DISTRO-interactive-markers \
       ros-$ROS_DISTRO-cv-bridge \
       gstreamer1.0-plugins-base \
       libgstreamer1.0-dev \
       libgstreamer-plugins-base1.0-dev

PlotJuggler is optional. The launcher searches for ``plotjuggler4``, then
``plotjuggler`` on PATH, then the Snap launcher. A native installation sourced
with the workspace supports its custom ROS messages. Disable launch with
``launch_plotjuggler:=false`` if it is not installed.

The C++ CasADi dependency must be built with ``-DWITH_THREAD=ON`` and
``-DWITH_THREADSAFE_SYMBOLICS=ON``. Observer workers and the control loop create
matrix/sparsity objects concurrently; numeric evaluation support alone does not
protect CasADi's shared caches. The package checks the exported compile options
at configuration time. Rebuild and reinstall CasADi and then rebuild this
workspace when changing these options. A Python wheel does not configure the
separate C++ dependency.

If CasADi is outside the system linker path:

.. code-block:: shell

   export LD_LIBRARY_PATH=/path/to/casadi/build/lib:$LD_LIBRARY_PATH

Workspace Setup
---------------

.. code-block:: shell

   source /opt/ros/jazzy/setup.bash
   git lfs install
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src

   git clone https://github.com/edxmorgan/uvms-simulator.git

   vcs import < uvms-simulator/dependency_repos.repos
   git -C uvms-simulator lfs pull
   git -C uvms-simlab lfs pull

   cd ~/ros_ws
   rosdep install --from-paths src --ignore-src -r -y

Python Dependencies
-------------------

Install Python packages used by SimLab for joystick control, planning, FCL,
Ruckig, and simulated camera rendering:

.. code-block:: shell

   python3 -m pip install pyPS4Controller pynput scipy casadi==3.7.2 ruckig \
       python-fcl trimesh pycollada pyvista open3d

For optional training dependencies, see :doc:`rl_training`.

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

   colcon build --packages-select ros2_control_blue_reach_5 simlab
   source install/setup.bash

Optional GPU Dynamics Build
---------------------------

``ros2_control_blue_reach_5`` can build the batched UVMS GPU dynamics backend
when a CUDA toolkit is available. The default mode is ``AUTO``: CPU dynamics are
always built, and CUDA dynamics are added when ``nvcc`` and a reliable CUDA
architecture setting are available.

On modern CMake versions the package uses ``CMAKE_CUDA_ARCHITECTURES=native``.
If you need to set the architecture manually, use a real/native target rather
than PTX-only output. For example, RTX 5090 is compute capability ``12.0``, so
use ``120``:

.. code-block:: shell

   CUDACXX=/usr/local/cuda/bin/nvcc colcon build \
       --packages-select ros2_control_blue_reach_5 simlab \
       --cmake-args \
       -DUVMS_ENABLE_CUDA_DYNAMICS=ON \
       -DUVMS_CUDA_ARCHITECTURES=120

Avoid PTX-only settings such as ``90-virtual`` for the UVMS RL backend; they can
build successfully and still fail at runtime on newer GPUs if the driver/toolkit
pair cannot load that PTX. If you change CUDA toolkit or architecture settings,
delete the package build cache before rebuilding:

.. code-block:: shell

   rm -rf build/ros2_control_blue_reach_5 install/ros2_control_blue_reach_5

To force a CPU-only build:

.. code-block:: shell

   colcon build --packages-select ros2_control_blue_reach_5 simlab \
       --cmake-args -DUVMS_ENABLE_CUDA_DYNAMICS=OFF

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

The HTML docs are built by default during local ``colcon`` builds. To disable
that for a faster code-only build:

.. code-block:: shell

   colcon build --packages-select ros2_control_blue_reach_5 \
       --cmake-args -DUVMS_BUILD_DOCS=OFF

When docs are enabled, the generated HTML is written to
``src/uvms-simulator/doc/_build/html``.

GitHub Pages Deployment
-----------------------

The repository includes a GitHub Actions workflow for publishing the Sphinx
site to GitHub Pages:

.. code-block:: text

   .github/workflows/docs-pages.yml

On pushes to ``main``, the workflow builds ``doc/_build/html``
and deploys it as the Pages artifact. Pull requests build the docs but do not
deploy them.

Updating
--------

Update the simulator and SimLab together, rebuild both packages, and restart ROS
processes after interface or model changes. Use the companion revisions in ``dependency_repos.repos``.

Continue with :doc:`userdoc` or :doc:`hil_setup`.

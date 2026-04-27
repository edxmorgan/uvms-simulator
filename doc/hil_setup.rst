.. _ros2_control_RA5BHS_hil_setupdoc:

Hardware-in-the-Loop Setup
==========================

Overview
--------

This page is for hardware-specific setup notes. General workspace
installation, Python dependencies, and simulator launch checks live in
:doc:`installation`.

Prerequisites
-------------

- BlueOS must be running on ``aarch64``.
- Install the BlueOS ROS 2 extension on the vehicle companion computer.
- Apply the configuration settings from the
  `BlueOS ROS 2 extension thread <https://discuss.bluerobotics.com/t/blueos-ros2-extension-v0-0-2-is-here/19324/5?u=mrrobot_1>`_.

Launch Direction
----------------

Use the hardware launch arguments only when the corresponding hardware is
connected and configured:

.. code-block:: shell

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
       use_manipulator_hardware:=true \
       use_vehicle_hardware:=true \
       sim_robot_count:=0 \
       task:=interactive

Mixed hardware/simulation launches use the same launch file with one hardware
flag enabled and the other disabled.

Architecture & HIL Presentation
-------------------------------

For a detailed walkthrough of the hardware architecture and HIL test setup,
consult the slide deck:

`Architecture & HIL Presentation <https://lsumail2-my.sharepoint.com/:p:/g/personal/emorg31_lsu_edu/EZNXdx-t7KlGj5Qo0V1qlxQBU7RX0Y2PIy5yE-KyVJcoLg?e=94xglK>`_

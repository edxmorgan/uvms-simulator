:github_url: https://github.com/edxmorgan/uvms-simulator/blob/main/doc/userdoc.rst

.. _ros2_control_RA5BHS_userdoc:

************************************************
Reach Alpha Blue Heavy Dynamics Simulator
************************************************

The *Reach Alpha Blue Heavy Dynamics Simulator* includes an interface plugin that supports multiple state and command interfaces.

Tutorial Steps
--------------------------

1. **Verify the Simulator Descriptions**

   To check that the *Reach Alpha Blue Heavy Sim* descriptions are working correctly, use the following launch command:

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 view_robot.launch.py

   ..  .. note:: //
   ..  It is normal to see the message ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``. This warning appears because the ``joint_state_publisher_gui`` node needs a moment to start. The ``joint_state_publisher_gui`` provides a GUI to generate a random configuration for the robot, which will be displayed in *RViz*.

2. **Start the Reach Alpha 5 Example**

   Open a terminal, source your ROS2 workspace, and execute the launch file with:

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py

   Useful launch-file options:

   - ``use_manipulator_hardware:=false``: Starts the simulator and connects to a real reach alpha hardware manipulator in the loop. Default value is ``false``

   - ``use_vehicle_hardware:=false``: Starts the simulator and connects to a real bluerov2 heavy underwater vehicle in the loop. Default value is ``false``

   - ``sim_robot_count:=n``: Starts the simulator by spawning n number of underwater vehicle manipulator systems.

   example launch command with options 

   .. code-block:: shell

      ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py use_manipulator_hardware:=false use_vehicle_hardware:=false sim_robot_count:=1

   The launch file will load and start the robot hardware, controllers, and open *RViz*. You will see extensive output from the hardware implementation in the terminal, showing its internal states.

3. **Verify Running Controllers**

   To check which controllers are currently active, run:

   .. code-block:: shell

    ros2 control list_controllers

   The output should look like:

   .. code-block:: shell

      gravity_broadcaster_robot_1__axis_c[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      gravity_broadcaster_robot_1__axis_e[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      fts_broadcaster_1   [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      gravity_broadcaster_robot_1__axis_a[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
      uvms_controller     [uvms_controller/UvmsController] active    
      gravity_broadcaster_robot_1__axis_d[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      gravity_broadcaster_robot_1__axis_b[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active

   Observe how this output changes based on the launch file arguments used.

.. 5. **Send Commands to the Controller**

..    If the controllers are active, you can send commands to the *Forward Current Controller* as follows:

..    - For the ``forward_current_controller``:

..      .. code-block:: shell

..       ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

..    - For the ``forward_effort_controller``:

..      .. code-block:: shell

..       ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

..    .. note::
..       The first five floating-point values correspond to the manipulator, from the base at index[0] to the end-effector at index[4]. The following eight values are for the vehicle's thrusters.

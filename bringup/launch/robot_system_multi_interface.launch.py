# Copyright (C) 2024 Edward Morgan
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
# 
# You should have received a copy of the GNU Affero General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch.logging
from bringup.utils.rviz_utils import rviz_file_configure
from bringup.utils.controller_config import modify_controller_config, parse_controller_list, controller_spawner_nodes

# Create a logger for the launch file
logger = launch.logging.get_logger('robot_system_multi_interface_launch')


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='alpha',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Start robot with device port to hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "state_update_frequency",
            default_value="200",
            description="The frequency (Hz) at which the driver updates the state of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_manipulator_hardware",
            default_value="false",
            description="Start simulation with a real manipulator hardware in the loop",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_vehicle_hardware",
            default_value="false",
            description="Start simulation with a real vehicle hardware in the loop",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "task",
            default_value="interactive",
            description="Start simulation with a task",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_robot_count",
            default_value="1",
            description="Spawn with n numbers of robot agents",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "record_data",
            default_value="false",
            description="record robot data",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers",
            default_value="pid",
            description="Comma-separated list of controllers to be used",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):

    # Resolve LaunchConfigurations
    prefix = LaunchConfiguration("prefix").perform(context)
    use_manipulator_hardware = LaunchConfiguration("use_manipulator_hardware").perform(context)
    use_vehicle_hardware = LaunchConfiguration("use_vehicle_hardware").perform(context)
    task = LaunchConfiguration("task").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)
    state_update_frequency = LaunchConfiguration("state_update_frequency").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    sim_robot_count = LaunchConfiguration("sim_robot_count").perform(context)
    record_data = LaunchConfiguration("record_data").perform(context)
    record_data_bool = IfCondition(record_data).evaluate(context)  
    controllers_list_str = LaunchConfiguration('controllers').perform(context)
    task = task.lower()
    use_pwm = str(task in {'direct_thrusters'})

    # Define the robot description command
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_blue_reach_5"),
                    "xacro",
                    "robot_system_multi_interface.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "state_update_frequency:=",
            state_update_frequency,
            " ",
            "use_manipulator_hardware:=",
            use_manipulator_hardware,
            " ",
            "use_vehicle_hardware:=",
            use_vehicle_hardware,
            " ",
            "record_data:=",
            record_data,
            " ",
            "task:=",
            task,
            " ",
            "sim_robot_count:=",
            sim_robot_count,
            " ",
            "use_pwm:=",
            use_pwm
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    robot_controllers_read = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "config",
            "robot_multi_interface_forward_controllers.yaml",
        ]
    )
    robot_controllers_modified = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "config",
            "robot_multi_interface_forward_controllers_modified.yaml",
        ]
    )

    # resolve PathJoinSubstitution to a string
    robot_controllers_read_file = str(robot_controllers_read.perform(context))
    robot_controllers_modified_file = str(robot_controllers_modified.perform(context))

    use_manipulator_hardware_bool = IfCondition(use_manipulator_hardware).evaluate(context)
    use_vehicle_hardware_bool = IfCondition(use_vehicle_hardware).evaluate(context)


    robot_prefixes, robot_base_links, robot_ix, no_robots = modify_controller_config(use_vehicle_hardware_bool,
                                                                                 use_manipulator_hardware_bool,
                                                                                  robot_controllers_read_file,
                                                                                    robot_controllers_modified_file,
                                                                                      int(sim_robot_count))
    
    rviz_config_read = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "rviz",
            "rviz.rviz",
        ]
    )
    rviz_config_modified = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "rviz",
            "rviz_modified.rviz",
        ]
    )
    # resolve PathJoinSubstitution to a string
    rviz_config_read_file = str(rviz_config_read.perform(context))
    rviz_config_modified_file = str(rviz_config_modified.perform(context))
    
    rviz_file_configure(use_vehicle_hardware_bool, 
                        use_manipulator_hardware_bool,
                        robot_prefixes, 
                        robot_base_links, 
                        robot_ix, 
                        rviz_config_read_file, 
                        rviz_config_modified_file, task)

    # Nodes Definitions
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_modified],
        condition=IfCondition(gui),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_modified, robot_description],
        arguments=['--ros-args', '--log-level', 'controller_manager:=ERROR'],
        output="both",
    )


    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner Nodes
    fts_spawner_nodes = []
    thruster_pwm_spawner_nodes = []
    vehicle_effort_spawner_nodes = []
    manip_effort_spawner_nodes = []

    # Spawn fts and controllers
    for k, robot_i in enumerate(robot_ix):
        robot_fts_spawner_nodes, thruster_pwm_ctrl_spawner, vehicle_effort_ctrl_spawner, manip_effort_ctrl_spawner = controller_spawner_nodes(robot_i, robot_prefixes[k])
        fts_spawner_nodes.extend(robot_fts_spawner_nodes)
        thruster_pwm_spawner_nodes.append(thruster_pwm_ctrl_spawner)
        vehicle_effort_spawner_nodes.append(vehicle_effort_ctrl_spawner)
        manip_effort_spawner_nodes.append(manip_effort_ctrl_spawner)

    # Define other nodes if needed
    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler > /dev/null 2>&1'],
        output='screen',
        shell=True
    )

    controllers = parse_controller_list(controllers_list_str, no_robots)
    logger.info(f"Controller list: {controllers}" )

    # start task selected
    mode = OpaqueFunction(function=lambda context: [])

    if task in {'interactive', 'manual', 'motion_plan', 'joint', 'direct_thrusters'}:
        try:
            FindPackageShare("simlab").find("simlab")
        except Exception as e:
            raise RuntimeError(
                "uvms simlab package not found. If you intend to run the coverage example, interactive marker mode, or manual control via PS4 joystick, please install uvms_simlab from https://github.com/edxmorgan/uvms_simlab"
            ) from e

        mode_params = {
            'robots_prefix': robot_prefixes,
            'no_robot': len(robot_prefixes),
            'no_efforts': 11,
            'record_data': record_data_bool,
            'controllers': controllers,
        }
        
        vehicle_thruster_pwm_ctrl = [n.controller_name for n in thruster_pwm_spawner_nodes]
        vehicle_effort_ctrl = [n.controller_name for n in vehicle_effort_spawner_nodes]
        manipulator_effort_ctrl = [n.controller_name for n in manip_effort_spawner_nodes]

        task_map = {
            'interactive': ('interactive_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
            'manual': ('joystick_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
            'motion_plan': ('motion_plan_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
            'joint': ('joint_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
            'direct_thrusters': ('direct_thruster_controller', [*manipulator_effort_ctrl, *vehicle_thruster_pwm_ctrl]),
        }

        exec_name, start_ctrls = task_map[task]
        mode = Node(
            package='simlab',
            executable=exec_name,
            name=exec_name,
            parameters=[mode_params],
        )

    switch_cmd = [
        "ros2", "control", "switch_controllers",
        "-c", "/controller_manager",
        "--strict",
        "--activate", *start_ctrls,
    ]

    switch_proc = ExecuteProcess(
        cmd=switch_cmd,
        output="screen",
        shell=False,
    )
    clp_node = Node(
        package='simlab',
        executable="cloudpoint_publisher",
        name="cloudpoint_publisher"
    )
    estimator_node = Node(
        package='simlab',
        executable="estimator_publisher",
        name="estimator_publisher",
        parameters=[mode_params],
    )
    
    # 1) Collect all spawners in the exact order you want them to complete
    all_spawners = [
        *fts_spawner_nodes,
        *thruster_pwm_spawner_nodes,
        *vehicle_effort_spawner_nodes,
        *manip_effort_spawner_nodes,
    ]

    if not all_spawners:
        raise RuntimeError("No spawners were created")

    # 2) Chain them, so spawner[i+1] starts only after spawner[i] exits
    chain_handlers = []
    for prev, nxt in zip(all_spawners[:-1], all_spawners[1:]):
        chain_handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=prev, on_exit=[nxt])
            )
        )

    # 3) When the final spawner exits, run the switch
    switch_after_all = RegisterEventHandler(
        OnProcessExit(target_action=all_spawners[-1], on_exit=[switch_proc])
    )

    # 4) After the switch
    # 4a) start RViz if requested
    rviz_after_switch = RegisterEventHandler(
        OnProcessExit(target_action=switch_proc, on_exit=[rviz_node])
    )
    # 4b) start clp after the switch
    clp_after_switch = RegisterEventHandler(
        OnProcessExit(target_action=switch_proc, on_exit=[clp_node, estimator_node])
    )
    # # 4c) start estimator only after clp has started and exited its launch process
    # estimator_after_clp = RegisterEventHandler(
    #     OnProcessExit(target_action=clp_node, on_exit=[])
    # )
    # Define the simulator actions
    simulator_actions = [
        joint_state_broadcaster_spawner,
        control_node,
        mode,
        run_plotjuggler,
        robot_state_pub_node,
        all_spawners[0],       # kick off the chain by launching only the first spawner explicitly
        *chain_handlers, # event handlers that wire the sequence
        switch_after_all,
        rviz_after_switch,
        clp_after_switch,
        # estimator_after_clp,
    ]
    
    # Define simulator_agent
    simulator_agents = GroupAction(
        actions=simulator_actions,)

    # Launch nodes
    nodes = [
        simulator_agents
    ]
    
    return nodes
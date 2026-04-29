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

import glob
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch.logging
from bringup.utils.rviz_utils import rviz_file_configure
from bringup.utils.controller_config import modify_controller_config, parse_controller_list, controller_spawner_nodes

# Create a logger for the launch file
logger = launch.logging.get_logger('robot_system_multi_interface_launch')


def _runtime_file_path(file_name: str) -> str:
    runtime_dir = os.path.join(tempfile.gettempdir(), "ros2_control_blue_reach_5")
    os.makedirs(runtime_dir, exist_ok=True)
    return os.path.join(runtime_dir, file_name)


def _resolve_serial_port(serial_port: str, use_manipulator_hardware: bool) -> str:
    if serial_port.lower() != "auto":
        return serial_port

    candidate_patterns = (
        "/dev/serial/by-id/*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    )
    candidates = []
    for pattern in candidate_patterns:
        candidates.extend(sorted(glob.glob(pattern)))

    if candidates:
        selected_port = candidates[0]
        logger.info(f"serial_port:=auto selected {selected_port}")
        return selected_port

    fallback_port = "/dev/ttyUSB0"
    if use_manipulator_hardware:
        logger.warning(
            f"serial_port:=auto found no serial device; falling back to {fallback_port}"
        )
    return fallback_port


def _parse_bool_arg(name: str, value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"{name} must be one of: true, false.")


def _simulated_camera_pipeline() -> str:
    return (
        "videotestsrc is-live=true pattern=ball "
        "! video/x-raw,width=640,height=480,framerate=30/1 "
        "! videoconvert ! video/x-raw,format=(string)BGR "
        "! appsink name=camera_sink emit-signals=true sync=false async=false max-buffers=1 drop=true"
    )


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
            default_value="auto",
            description="Manipulator hardware serial port, or auto to use the first available serial device.",
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
            "launch_camera",
            default_value="auto",
            description="Start the independent GStreamer camera node. Use true, false, or auto. auto follows use_vehicle_hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulate_camera",
            default_value="false",
            description="Use a synthetic GStreamer test camera instead of the UDP hardware camera.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_pipeline",
            default_value="",
            description="Optional custom GStreamer pipeline. Must end with appsink name=camera_sink.",
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
            "same_initial_conditions",
            default_value="false",
            description="Start all simulated UVMS instances with the same initial pose/orientation.",
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
            "mode_enabled",
            default_value="true",
            description="Start the task-specific simlab control node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mocap",
            default_value="false",
            description="Start the OptiTrack bridge and mocap publisher.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_plotjuggler",
            default_value="true",
            description="Start PlotJuggler.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_overlay_text",
            default_value="true",
            description="Start the RViz overlay text bridge.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_collision_contact",
            default_value="true",
            description="Start collision and clearance visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_voxelviz",
            default_value="true",
            description="Start voxelized environment visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_env_obstacles",
            default_value="true",
            description="Start environment obstacle publishing.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_planner_action_server",
            default_value="false",
            description="Start the planner action server even without the interactive controller.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):

    # Resolve LaunchConfigurations
    prefix = LaunchConfiguration("prefix").perform(context)
    use_manipulator_hardware = LaunchConfiguration("use_manipulator_hardware").perform(context)
    use_vehicle_hardware = LaunchConfiguration("use_vehicle_hardware").perform(context)
    launch_camera = LaunchConfiguration("launch_camera").perform(context)
    simulate_camera = LaunchConfiguration("simulate_camera").perform(context)
    camera_pipeline = LaunchConfiguration("camera_pipeline").perform(context)
    task = LaunchConfiguration("task").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)
    state_update_frequency = LaunchConfiguration("state_update_frequency").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    sim_robot_count = LaunchConfiguration("sim_robot_count").perform(context)
    same_initial_conditions = LaunchConfiguration("same_initial_conditions").perform(context)
    record_data = LaunchConfiguration("record_data").perform(context)
    record_data_bool = IfCondition(record_data).evaluate(context)
    mode_enabled = LaunchConfiguration("mode_enabled").perform(context)
    use_mocap = LaunchConfiguration("use_mocap").perform(context)
    launch_plotjuggler = LaunchConfiguration("launch_plotjuggler").perform(context)
    launch_overlay_text = LaunchConfiguration("launch_overlay_text").perform(context)
    launch_collision_contact = LaunchConfiguration("launch_collision_contact").perform(context)
    launch_voxelviz = LaunchConfiguration("launch_voxelviz").perform(context)
    launch_env_obstacles = LaunchConfiguration("launch_env_obstacles").perform(context)
    launch_planner_action_server = LaunchConfiguration("launch_planner_action_server").perform(context)
    task = task.lower()
    use_pwm = str(task in {'direct_thrusters'})
    valid_tasks = {'interactive', 'manual', 'joint', 'direct_thrusters'}
    use_manipulator_hardware_bool = IfCondition(use_manipulator_hardware).evaluate(context)
    use_vehicle_hardware_bool = IfCondition(use_vehicle_hardware).evaluate(context)
    serial_port = _resolve_serial_port(serial_port, use_manipulator_hardware_bool)

    if task not in valid_tasks:
        raise RuntimeError(
            f"Unsupported task '{task}'. Expected one of: {', '.join(sorted(valid_tasks))}."
        )

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
            "same_initial_conditions:=",
            same_initial_conditions,
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
    # resolve PathJoinSubstitution to a string
    robot_controllers_read_file = str(robot_controllers_read.perform(context))
    runtime_tag = f"{task}_{sim_robot_count}_{os.getpid()}"
    robot_controllers_modified_file = _runtime_file_path(
        f"robot_multi_interface_forward_controllers_{runtime_tag}.yaml"
    )

    use_mocap_bool = IfCondition(use_mocap).evaluate(context)
    launch_planner_action_server_bool = IfCondition(launch_planner_action_server).evaluate(context)
    simulate_camera_bool = _parse_bool_arg("simulate_camera", simulate_camera)
    launch_camera_normalized = launch_camera.strip().lower()
    if launch_camera_normalized == "auto":
        launch_camera_bool = use_vehicle_hardware_bool or simulate_camera_bool
    elif launch_camera_normalized in {"true", "1", "yes", "on"}:
        launch_camera_bool = True
    elif launch_camera_normalized in {"false", "0", "no", "off"}:
        launch_camera_bool = False
    else:
        raise RuntimeError("launch_camera must be one of: auto, true, false.")
    camera_pipeline = camera_pipeline.strip()
    if not camera_pipeline and simulate_camera_bool:
        camera_pipeline = _simulated_camera_pipeline()

    is_hardware_uvms = use_manipulator_hardware_bool and use_vehicle_hardware_bool

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
    # resolve PathJoinSubstitution to a string
    rviz_config_read_file = str(rviz_config_read.perform(context))
    rviz_config_modified_file = _runtime_file_path(f"rviz_{runtime_tag}.rviz")
    
    rviz_file_configure(use_vehicle_hardware_bool, 
                        use_manipulator_hardware_bool,
                        robot_prefixes, 
                        robot_base_links, 
                        robot_ix, 
                        rviz_config_read_file, 
                        rviz_config_modified_file, task,
                        launch_camera_bool)

    reset_coordinator_proc = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "bringup.sim_reset_coordinator",
            "--robots-prefix",
            ",".join(robot_prefixes),
        ],
        output="screen",
    )

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
        arguments=["-d", rviz_config_modified_file],
        condition=IfCondition(gui),
        additional_env={
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/liboctomap.so"
        },
    )

    overlay_text_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='string_to_overlay_text_1',
        output='screen',
        parameters=[
            {"string_topic": "chatter"},
            {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
        ],
        condition=IfCondition(launch_overlay_text),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_modified_file, robot_description],
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
        shell=True,
        condition=IfCondition(launch_plotjuggler),
    )

    # start task selected
    mode = OpaqueFunction(function=lambda context: [])
    mode_params = {
        'robots_prefix': robot_prefixes,
        'no_robot': len(robot_prefixes),
        'no_efforts': 11,
        "robot_description": robot_description_content
    }

    try:
        FindPackageShare("simlab").find("simlab")
    except Exception as e:
        raise RuntimeError(
            "uvms simlab package not found. If you intend to run the coverage example, interactive marker mode, or manual control via PS4 joystick, please install uvms_simlab from https://github.com/edxmorgan/uvms_simlab"
        ) from e

    vehicle_thruster_pwm_ctrl = [n.controller_name for n in thruster_pwm_spawner_nodes]
    vehicle_effort_ctrl = [n.controller_name for n in vehicle_effort_spawner_nodes]
    manipulator_effort_ctrl = [n.controller_name for n in manip_effort_spawner_nodes]

    task_map = {
        'interactive': ('interactive_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
        'manual': ('joystick_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
        'joint': ('joint_controller', [*manipulator_effort_ctrl, *vehicle_effort_ctrl]),
        'direct_thrusters': ('direct_thruster_controller', [*manipulator_effort_ctrl, *vehicle_thruster_pwm_ctrl]),
    }

    exec_name, start_ctrls = task_map[task]
    mode = Node(
        package='simlab',
        executable=exec_name,
        name=exec_name,
        parameters=[mode_params],
        condition=IfCondition(mode_enabled),
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
    rgb2clpts_node = Node(
        package='simlab',
        executable="rgb2cloudpoint_publisher",
        name="rgb2cloudpoint_publisher"
    )
    bag_recorder_node = Node(
        package='simlab',
        executable="bag_recorder_node",
        name="bag_recorder_node",
        parameters=[
            mode_params,
            {"autostart_recording": record_data_bool},
        ],
    )

    camera_prefix = "robot_real_" if use_vehicle_hardware_bool else next(
        (prefix for prefix in robot_prefixes if prefix != "robot_real_"),
        robot_prefixes[0],
    )
    camera_node = Node(
        package="ros2_control_blue_reach_5",
        executable="gstreamer_camera_node",
        name="gstreamer_camera_node",
        output="screen",
        parameters=[{
            "image_topic": "/alpha/image_raw",
            "frame_id": f"{camera_prefix}camera_link",
            **({"pipeline": camera_pipeline} if camera_pipeline else {}),
        }],
    )

    optitrack_proc = ExecuteProcess(
        cmd=['ros2', 'launch', 'mocap4r2_optitrack_driver', 'optitrack2.launch.py'],
        output='screen',
        shell=False,
    )

    # optitrack_proc = Node(
    #     package='simlab',
    #     executable="motive_publisher",
    #     name="motive_publisher",
    #     parameters=[mode_params],
    # )


    mocap_node = Node(
        package='simlab',
        executable="mocap_publisher",
        name="mocap_publisher",
    )

    mocap_after_optitrack = RegisterEventHandler(
        OnProcessStart(target_action=optitrack_proc, on_start=[mocap_node])
    )

    mesh_collision_node = Node(
        package="simlab",
        executable="collision_contact_node",
        name="collision_contact_node",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
        }],
        condition=IfCondition(launch_collision_contact),
    )

    voxelviz_node = Node(
        package="simlab",
        executable="voxelviz_node",
        name="voxelviz_node",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
        }],
        condition=IfCondition(launch_voxelviz),
    )

    env_obstacles_node = Node(
        package="simlab",
        executable="env_obstacles_node",
        name="env_obstacles_node",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
        }],
        condition=IfCondition(launch_env_obstacles),
    )

    planner_action_server_node = None
    if task == "interactive" or launch_planner_action_server_bool:
        planner_action_server_node = Node(
            package="simlab",
            executable="planner_action_server_node",
            name="planner_action_server_node",
            output="screen",
            parameters=[{
                "robot_description": robot_description_content,
            }],
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
        OnProcessExit(target_action=switch_proc, on_exit=[rviz_node, overlay_text_node])
    )
    # 4b) start clp after the switch
    clp_after_switch = RegisterEventHandler(
        OnProcessExit(target_action=switch_proc, on_exit=[rgb2clpts_node])
    )

    simulator_actions = [
        reset_coordinator_proc,
        joint_state_broadcaster_spawner,
        control_node,
        mode,
        run_plotjuggler,
        robot_state_pub_node,
        all_spawners[0],
        *chain_handlers,
        switch_after_all,
        rviz_after_switch,
        mesh_collision_node,
        voxelviz_node,
        env_obstacles_node,
        bag_recorder_node,
    ]

    if launch_camera_bool:
        simulator_actions.append(camera_node)

    if use_mocap_bool:
        simulator_actions.extend([
            optitrack_proc,
            mocap_after_optitrack,
        ])

    if planner_action_server_node is not None:
        simulator_actions.append(planner_action_server_node)


    # if is_hardware_uvms:
    #     simulator_actions.append(clp_after_switch)

    # Define simulator_agent
    simulator_agents = GroupAction(
        actions=simulator_actions,)

    # Launch nodes
    nodes = [
        simulator_agents
    ]
    
    return nodes

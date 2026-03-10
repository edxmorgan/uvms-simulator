from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value="alpha"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("state_update_frequency", default_value="200"),
        DeclareLaunchArgument("use_manipulator_hardware", default_value="false"),
        DeclareLaunchArgument("use_vehicle_hardware", default_value="false"),
        DeclareLaunchArgument("sim_robot_count", default_value="1"),
        DeclareLaunchArgument("controller_name", default_value="pid"),
        DeclareLaunchArgument("planner_name", default_value="Bitstar"),
        DeclareLaunchArgument("goal_count", default_value="3"),
        DeclareLaunchArgument("goal_seed", default_value="11"),
        DeclareLaunchArgument("goal_file", default_value=""),
        DeclareLaunchArgument("time_limit", default_value="1.0"),
        DeclareLaunchArgument("robot_collision_radius", default_value="0.574"),
        DeclareLaunchArgument("planning_timeout_sec", default_value="15.0"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="75.0"),
        DeclareLaunchArgument("goal_tolerance_m", default_value="0.40"),
        DeclareLaunchArgument("settle_time_sec", default_value="1.0"),
        DeclareLaunchArgument("stop_speed_threshold", default_value="0.05"),
        DeclareLaunchArgument("sample_min_clearance", default_value="0.10"),
        DeclareLaunchArgument("sample_z_min", default_value="-4.0"),
        DeclareLaunchArgument("sample_z_max", default_value="-1.0"),
        DeclareLaunchArgument("min_goal_distance", default_value="4.0"),
        DeclareLaunchArgument("max_goal_distance", default_value="8.0"),
        DeclareLaunchArgument("startup_delay_sec", default_value="12.0"),
        DeclareLaunchArgument("inter_goal_pause_sec", default_value="2.0"),
        DeclareLaunchArgument("abort_on_failure", default_value="false"),
        DeclareLaunchArgument("output_path", default_value="/tmp/simlab_execution_benchmark_results.json"),
    ]

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
            LaunchConfiguration("prefix"),
            " ",
            "serial_port:=",
            LaunchConfiguration("serial_port"),
            " ",
            "state_update_frequency:=",
            LaunchConfiguration("state_update_frequency"),
            " ",
            "use_manipulator_hardware:=",
            LaunchConfiguration("use_manipulator_hardware"),
            " ",
            "use_vehicle_hardware:=",
            LaunchConfiguration("use_vehicle_hardware"),
            " ",
            "record_data:=false",
            " ",
            "task:=interactive",
            " ",
            "sim_robot_count:=",
            LaunchConfiguration("sim_robot_count"),
            " ",
            "use_pwm:=false",
        ]
    )

    stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_blue_reach_5"),
                    "launch",
                    "robot_system_multi_interface.launch.py",
                ]
            )
        ),
        launch_arguments={
            "prefix": LaunchConfiguration("prefix"),
            "serial_port": LaunchConfiguration("serial_port"),
            "state_update_frequency": LaunchConfiguration("state_update_frequency"),
            "use_manipulator_hardware": LaunchConfiguration("use_manipulator_hardware"),
            "use_vehicle_hardware": LaunchConfiguration("use_vehicle_hardware"),
            "task": "interactive",
            "sim_robot_count": LaunchConfiguration("sim_robot_count"),
            "gui": "false",
            "mode_enabled": "false",
            "record_data": "false",
            "use_mocap": "false",
            "launch_plotjuggler": "false",
            "launch_overlay_text": "false",
            "launch_collision_contact": "false",
            "launch_voxelviz": "false",
            "launch_env_obstacles": "false",
            "launch_planner_action_server": "true",
        }.items(),
    )

    execution_runner = Node(
        package="simlab",
        executable="execution_benchmark_runner",
        name="execution_benchmark_runner",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "robots_prefix": ["robot_1_"],
                "controller_name": LaunchConfiguration("controller_name"),
                "planner_name": LaunchConfiguration("planner_name"),
                "goal_count": LaunchConfiguration("goal_count"),
                "goal_seed": LaunchConfiguration("goal_seed"),
                "goal_file": LaunchConfiguration("goal_file"),
                "time_limit": LaunchConfiguration("time_limit"),
                "robot_collision_radius": LaunchConfiguration("robot_collision_radius"),
                "planning_timeout_sec": LaunchConfiguration("planning_timeout_sec"),
                "goal_timeout_sec": LaunchConfiguration("goal_timeout_sec"),
                "goal_tolerance_m": LaunchConfiguration("goal_tolerance_m"),
                "settle_time_sec": LaunchConfiguration("settle_time_sec"),
                "stop_speed_threshold": LaunchConfiguration("stop_speed_threshold"),
                "sample_min_clearance": LaunchConfiguration("sample_min_clearance"),
                "sample_z_min": LaunchConfiguration("sample_z_min"),
                "sample_z_max": LaunchConfiguration("sample_z_max"),
                "min_goal_distance": LaunchConfiguration("min_goal_distance"),
                "max_goal_distance": LaunchConfiguration("max_goal_distance"),
                "startup_delay_sec": LaunchConfiguration("startup_delay_sec"),
                "inter_goal_pause_sec": LaunchConfiguration("inter_goal_pause_sec"),
                "abort_on_failure": LaunchConfiguration("abort_on_failure"),
                "output_path": LaunchConfiguration("output_path"),
            }
        ],
    )

    shutdown_after_runner = RegisterEventHandler(
        OnProcessExit(
            target_action=execution_runner,
            on_exit=[EmitEvent(event=Shutdown(reason="execution benchmark runner finished"))],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            stack_launch,
            execution_runner,
            shutdown_after_runner,
        ]
    )

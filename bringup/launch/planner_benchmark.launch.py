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
        DeclareLaunchArgument("same_initial_conditions", default_value="false"),
        DeclareLaunchArgument("scenario_count", default_value="6"),
        DeclareLaunchArgument("scenario_seed", default_value="7"),
        DeclareLaunchArgument("planner_names", default_value="Bitstar,RRTstar"),
        DeclareLaunchArgument("time_limit", default_value="1.0"),
        DeclareLaunchArgument("robot_collision_radius", default_value="0.574"),
        DeclareLaunchArgument("sample_min_clearance", default_value="0.10"),
        DeclareLaunchArgument("sample_z_min", default_value="-4.0"),
        DeclareLaunchArgument("sample_z_max", default_value="-1.0"),
        DeclareLaunchArgument("min_goal_distance", default_value="4.0"),
        DeclareLaunchArgument("startup_delay_sec", default_value="12.0"),
        DeclareLaunchArgument("output_path", default_value="/tmp/simlab_planner_benchmark_results.json"),
        DeclareLaunchArgument("scenario_file", default_value=""),
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
            "same_initial_conditions:=",
            LaunchConfiguration("same_initial_conditions"),
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
            "same_initial_conditions": LaunchConfiguration("same_initial_conditions"),
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

    benchmark_runner = Node(
        package="simlab",
        executable="planner_benchmark_runner",
        name="planner_benchmark_runner",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "planner_names": LaunchConfiguration("planner_names"),
                "scenario_count": LaunchConfiguration("scenario_count"),
                "scenario_seed": LaunchConfiguration("scenario_seed"),
                "time_limit": LaunchConfiguration("time_limit"),
                "robot_collision_radius": LaunchConfiguration("robot_collision_radius"),
                "sample_min_clearance": LaunchConfiguration("sample_min_clearance"),
                "sample_z_min": LaunchConfiguration("sample_z_min"),
                "sample_z_max": LaunchConfiguration("sample_z_max"),
                "min_goal_distance": LaunchConfiguration("min_goal_distance"),
                "startup_delay_sec": LaunchConfiguration("startup_delay_sec"),
                "output_path": LaunchConfiguration("output_path"),
                "scenario_file": LaunchConfiguration("scenario_file"),
            }
        ],
    )

    shutdown_after_runner = RegisterEventHandler(
        OnProcessExit(
            target_action=benchmark_runner,
            on_exit=[EmitEvent(event=Shutdown(reason="planner benchmark runner finished"))],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            stack_launch,
            benchmark_runner,
            shutdown_after_runner,
        ]
    )

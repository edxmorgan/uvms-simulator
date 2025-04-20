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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml, copy
import random
from launch.actions import OpaqueFunction
from launch.substitutions import PythonExpression
import launch.logging

from launch.launch_description_sources import PythonLaunchDescriptionSource

# Create a logger for the launch file
logger = launch.logging.get_logger('robot_system_multi_interface_launch')

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

def rviz_file_configure(use_vehicle_hardware, use_manipulator_hardware, robot_prefixes,
                         robot_base_links, ix, rviz_config_path,
                         new_rviz_config_path, task)->None:
    # Load the RViz configuration file
    with open(rviz_config_path,'r') as file:
        rviz_config = yaml.load(file,yaml.SafeLoader)
    new_rviz_config = copy.deepcopy(rviz_config)

    rviz_view_configure(robot_prefixes, robot_base_links, new_rviz_config, task)
    rviz_states_axes_configure(robot_prefixes, new_rviz_config)
    rviz_robots_path_configure(robot_prefixes, new_rviz_config)

    if use_vehicle_hardware:
        image_stream_display("video feed", "/alpha/image_raw",new_rviz_config, True)

    if task == 'interactive':
        rviz_interactive_marker('controller_interactiveMarker', '/uvms_interactive_controls', new_rviz_config, True)
        rviz_point_cloud2('uvms_taskspace',"/workspace_pointcloud",new_rviz_config, True, "255; 255; 255",
                          custom_properties={"Decay Time": 0.2, "Size (Pixels)": 1.5})
        rviz_point_cloud2('uv_vehicle_base',"/base_pointcloud",new_rviz_config, True, "255; 0; 0",
                          custom_properties={"Decay Time": 0.2, "Size (Pixels)": 1.5})

    add_wrench_entries(ix, new_rviz_config, False)
    with open(new_rviz_config_path,'w') as file:
        yaml.dump(new_rviz_config,file,Dumper=NoAliasDumper)


def rviz_path_display(name, topic, rviz_config, color, enabled):
    path_config = {
        "Alpha": 1,
        "Buffer Length": 1,
        "Class": "rviz_default_plugins/Path",
        "Color": color,
        "Enabled": enabled,
        "Head Diameter": 0.3,
        "Head Length": 0.2,
        "Length": 0.3,
        "Line Style": "Lines",
        "Line Width": 0.03,
        "Name": name,
        "Offset": {
            "X": 0,
            "Y": 0,
            "Z": 0
        },
        "Pose Color": "255; 85; 255",
        "Pose Style": "None",
        "Radius": 0.03,
        "Shaft Diameter": 0.1,
        "Shaft Length": 0.1,
        "Topic": {
            "Depth": 5,
            "Durability Policy": "Volatile",
            "Filter size": 40,
            "History Policy": "Keep Last",
            "Reliability Policy": "Reliable",
            "Value": topic
        },
        "Value": True
    }
    rviz_config['Visualization Manager']['Displays'].append(path_config)


def rviz_axes_display(name, reference_frame, rviz_config, length, radius, enabled):
    added_axes = {'Class': 'rviz_default_plugins/Axes',
        'Enabled': enabled,
        'Length': str(length),
        'Name': name,
        'Radius': str(radius),
        'Reference Frame': reference_frame,
        'Value': True}
    rviz_config['Visualization Manager']['Displays'].append(added_axes)
    
def rviz_point_cloud2(name, topic, rviz_config, enabled, color="255; 255; 255", custom_properties=None):
    added_point_cloud2 = {
        "Alpha": 1,
        "Autocompute Intensity Bounds": True,
        "Autocompute Value Bounds": {
            "Max Value": 10,
            "Min Value": -10,
            "Value": True
        },
        "Axis": "Z",
        "Channel Name": "intensity",
        "Class": "rviz_default_plugins/PointCloud2",
        "Color": color,
        "Color Transformer": "FlatColor",
        "Decay Time": 0.30000001192092896,
        "Enabled": enabled,
        "Invert Rainbow": False,
        "Max Color": "255; 255; 255",
        "Max Intensity": 4096,
        "Min Color": "0; 0; 0",
        "Min Intensity": 0,
        "Name": name,
        "Position Transformer": "XYZ",
        "Selectable": False,
        "Size (Pixels)": 1.5,
        "Size (m)": 0.009999999776482582,
        "Style": "Points",
        "Topic": {
            "Depth": 5,
            "Durability Policy": "Volatile",
            "Filter size": 100,
            "History Policy": "Keep Last",
            "Reliability Policy": "Best Effort",
            "Value": topic
        },
        "Use Fixed Frame": True,
        "Use rainbow": True,
        "Value": True
    }
    # Override default properties with any custom properties provided
    if custom_properties:
        added_point_cloud2.update(custom_properties)
    
    rviz_config['Visualization Manager']['Displays'].append(added_point_cloud2)


def rviz_interactive_marker(name, namespace, rviz_config, enabled):
    added_interactive_marker = {'Class': 'rviz_default_plugins/InteractiveMarkers',
        'Enable Transparency': True,
        'Enabled': enabled,
        'Interactive Markers Namespace': namespace,
        'Name': name,
        'Show Axes': True,
        'Show Descriptions': False,
        'Show Visual Aids': False,
        'Value': True}
    rviz_config['Visualization Manager']['Displays'].append(added_interactive_marker)

def image_stream_display(name, topic,rviz_config, enabled):
    image_config = {
        "Class": "rviz_default_plugins/Image",
        "Enabled": enabled,
        "Max Value": 1,
        "Median window": 5,
        "Min Value": 0,
        "Name": name,
        "Normalize Range": True,
        "Topic": {
            "Depth": 5,
            "Durability Policy": "Volatile",
            "History Policy": "Keep Last",
            "Reliability Policy": "Reliable",
            "Value": topic
        },
        "Value": True
    }
    rviz_config['Visualization Manager']['Displays'].append(image_config)

def imu_display(name, topic,rviz_config, enabled):
    imu_config = {
            "Acceleration properties": {
                "Acc. vector alpha": 0.10000000149011612,
                "Acc. vector color": "255; 0; 0",
                "Acc. vector scale": 0.05000000074505806,
                "Derotate acceleration": True,
                "Enable acceleration": False
            },
            "Axes properties": {
                "Axes scale": 0.2,
                "Enable axes": True
            },
            "Box properties": {
                "Box alpha": 1,
                "Box color": "255; 0; 0",
                "Enable box": False,
                "x_scale": 1,
                "y_scale": 1,
                "z_scale": 1
            },
            "Class": "rviz_imu_plugin/Imu",
            "Enabled": enabled,
            "Name": name,
            "Topic": {
                "Depth": 13,
                "Durability Policy": "Volatile",
                "Filter size": 10,
                "History Policy": "Keep Last",
                "Reliability Policy": "Best Effort",
                "Value": topic
            },
            "Value": True,
            "fixed_frame_orientation": True
        }
    rviz_config['Visualization Manager']['Displays'].append(imu_config)

def generate_random_color(path_type=None, default=False):
    """
    Generates a random color in the "R; G; B" string format.
    
    Returns:
    - str: Random color as "R; G; B".
    """
    if default and path_type=='ref_path':
        default_path_color = "248; 228; 92"       # Yellow for the first robot's Path
        return default_path_color
    if default and path_type=='robot_path': 
        default_traj_color = "224;27;36"        # Red for the first robot's TrajectoryPath
        return default_traj_color
        
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return f"{r}; {g}; {b}"

def rviz_robots_path_configure(robot_prefixes, rviz_config):
    enabled = True
    for idx, prefix in enumerate(robot_prefixes):
        if idx == 0:
            path_color = generate_random_color('ref_path', True)
            traj_color = generate_random_color('robot_path', True)
        else:
            path_color = generate_random_color()
            traj_color = generate_random_color()
        rviz_path_display(f"{prefix}/desiredPath", f"/{prefix}desiredPath", rviz_config, path_color, enabled)
        rviz_path_display(f"{prefix}/robotPath", f"/{prefix}robotPath", rviz_config, traj_color, enabled)

def rviz_states_axes_configure(robot_prefixes, rviz_config):
    for prefix in robot_prefixes:
        base_link = f'{prefix}base_link'
        robot_map_frame = f'{prefix}map'
        rviz_axes_display(base_link, base_link, rviz_config, 0.1, 0.01, True)
        rviz_axes_display(robot_map_frame, f'{prefix}map', rviz_config, 0.1, 0.01, True)
        for i in range(5):
            rviz_axes_display(f'{prefix}joint_{i}', f"{prefix}joint_{i}", rviz_config, 0.1, 0.01, True)
            rviz_axes_display(f'{prefix}dvl_frame', f"{prefix}dvl_link", rviz_config, 0.1, 0.01, True)

def rviz_view_configure(robot_prefixes, robot_base_links, rviz_config, task):
    rviz_config['Visualization Manager']['Views']['Saved'] = []
    original_view = {
        'Class': 'rviz_default_plugins/Orbit',
        'Distance': 8.755668640136719,
        'Enable Stereo Rendering': {
          'Stereo Eye Separation': 0.05999999865889549,
          'Stereo Focal Distance': 1,
          'Swap Stereo Eyes': False,
          'Value': False},
        'Focal Point': {
          'X': 0,
          'Y': 0,
          'Z': 0},
        'Focal Shape Fixed Size': True,
        'Focal Shape Size': 0.05000000074505806,
        'Invert Z Axis': False,
        'Name': 'World Origin',
        'Near Clip Distance': 0.009999999776482582,
        'Pitch': 0.44020360708236694,
        'Target Frame': 'base_link',
        'Value': 'Orbit (rviz)',
        'Yaw': 3.3494811058044434
    }
    rviz_config['Visualization Manager']['Views']['Saved'].append(original_view)
    for i, prefix in enumerate(robot_prefixes):
        new_view = original_view.copy()
        new_view['Name'] = f'{prefix} view'
        new_view['Target Frame'] = robot_base_links[i]
        rviz_config['Visualization Manager']['Views']['Saved'].append(new_view)
    if task == 'interactive':
        new_view = original_view.copy()
        new_view['Name'] = f'marker view'
        new_view['Target Frame'] = 'vehicle_marker_frame'
        rviz_config['Visualization Manager']['Views']['Saved'].append(new_view)


def add_wrench_entries(ix, rviz_config, enabled= True):
    # The existing wrench configuration you want to replicate
    original_wrench = {
        'Accept NaN Values': False,
        'Alpha': 1,
        'Arrow Width': 0.3,
        'Class': 'rviz_default_plugins/Wrench',
        'Enabled': enabled,
        'Force Arrow Scale': 0.7,
        'Force Color': '204; 51; 51',
        'History Length': 1,
        'Name': 'Wrench',
        'Torque Arrow Scale': 0.7,
        'Torque Color': '204; 204; 51',
        'Value': True
    }

    # Add new Wrench entries with the incremented index in the 'Value' field
    for i in ix:
        new_wrench = original_wrench.copy()
        new_wrench['Name'] = f'robot_Wrench_{i}'
        new_wrench['Topic'] = {
            'Depth': 5,
            'Durability Policy': 'Volatile',
            'Filter size': 10,
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': f'/fts_broadcaster_{i}/wrench'
        }
        rviz_config['Visualization Manager']['Displays'].append(new_wrench)

def modify_controller_config(use_vehicle_hardware, use_manipulator_hardware, config_path,new_config_path,sim_robot_count:int=1):
        
        if not use_vehicle_hardware and not use_manipulator_hardware and sim_robot_count==0:
            raise Exception("Invalid configuration: No robots specified. Enable either vehicle or manipulator hardware, or specify at least one simulation robot.")

        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        new_param = copy.deepcopy(controller_param)

        controller_descriptor = new_param['uvms_controller']['ros__parameters']

        dof_efforts = [
                effort 
                for joint in controller_descriptor['joints'] 
                if joint in controller_descriptor 
                for effort in controller_descriptor[joint].get('effort_command_interface', [])
            ]

        ix = []
        robot_prefixes = []
        robot_base_links = []

        new_param['uvms_controller']['ros__parameters']['agents'] = []

        if use_manipulator_hardware or use_vehicle_hardware:
            i = 'real'
            new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix)

        for i in range(1, sim_robot_count + 1):
            new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix)

        with open(new_config_path,'w') as file:
            yaml.dump(new_param,file,Dumper=NoAliasDumper)
        return robot_prefixes, robot_base_links, ix, dof_efforts

def add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix):
    ix.append(i)
    agent_name = f'bluerov_alpha_{i}'
    prefix = f'robot_{i}_'

    robot_prefixes.append(prefix)

    base_link = f'{prefix}base_link'
    robot_base_links.append(base_link)

    IOs = f'{prefix}IOs'
        # Add agent to the uvms_controller parameters
    new_param['uvms_controller']['ros__parameters']['agents'].append(agent_name)

    # Add agent-specific parameters under uvms_controller
    new_param['uvms_controller']['ros__parameters'][agent_name] = {
        'prefix': prefix,
        'base_TF_translation': [0.190, 0.000, -0.120],
        'base_TF_rotation': [3.142, 0.000, 0.000],
    }

    fts_broadcaster_name = f'fts_broadcaster_{i}'
    new_param['controller_manager']['ros__parameters'][fts_broadcaster_name] = {
        'type': 'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster'
    }

    new_param[fts_broadcaster_name] = {'ros__parameters': {
        'frame_id': base_link,
        'interface_names': {
            'force': {
                'x': f'{IOs}/force.x',
                'y': f'{IOs}/force.y',
                'z': f'{IOs}/force.z'
                },
            'torque': {
                'x': f'{IOs}/torque.x',
                'y': f'{IOs}/torque.y',
                'z': f'{IOs}/torque.z'
                }
            }
        }
    }

    for axis_i in ['e','d','c','b','a']:
        gravity_broadcaster_name = f'gravity_broadcaster_{prefix}_axis_{axis_i}'
        new_param['controller_manager']['ros__parameters'][gravity_broadcaster_name] = {
            'type': 'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster'
        }

        new_param[gravity_broadcaster_name] = {'ros__parameters': {
            'frame_id': base_link,
            'interface_names': {
                'force': {
                    'x': f'{prefix}_axis_{axis_i}/gravity_force.x',
                    'y': f'{prefix}_axis_{axis_i}/gravity_force.y',
                    'z': f'{prefix}_axis_{axis_i}/gravity_force.z'
                    },
                'torque': {
                    'x': f'{prefix}_axis_{axis_i}/gravity_torque.x',
                    'y': f'{prefix}_axis_{axis_i}/gravity_torque.y',
                    'z': f'{prefix}_axis_{axis_i}/gravity_torque.z'
                    }
                }
            }
        }
    return new_param, robot_prefixes, robot_base_links, ix

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
            default_value="",
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
    sim_robot_count = int(LaunchConfiguration("sim_robot_count").perform(context))
    record_data = LaunchConfiguration("record_data").perform(context)

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
            TextSubstitution(text=str(sim_robot_count))
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
    record_data_bool = IfCondition(record_data).evaluate(context)

    robot_prefixes, robot_base_links, ix, dof_efforts = modify_controller_config(use_vehicle_hardware_bool,
                                                                                 use_manipulator_hardware_bool,
                                                                                  robot_controllers_read_file,
                                                                                    robot_controllers_modified_file,
                                                                                      sim_robot_count)


    # Read the controllers string and split into a list
    controllers_str = LaunchConfiguration('controllers').perform(context)
    controllers = ['force' if task=='manual' or task=='cli' else c.strip() for c in controllers_str.split(',')]
    
    no_robots = len(robot_prefixes)
    if len(controllers) == 1:
        controllers = [controllers[0]]*no_robots
    elif len(controllers) != no_robots:
        raise Exception("Argument Error: The number of controllers does not match the number of simulation robots.")

    logger.info(f'Controller list: {controllers}')

    
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
    
    rviz_file_configure(use_vehicle_hardware_bool, use_manipulator_hardware_bool,robot_prefixes, robot_base_links, ix, rviz_config_read_file, rviz_config_modified_file, task)

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
        output="both",
    )


    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    # UVMS Controller Spawner (if using mock hardware)
    uvms_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["uvms_controller", "--controller-manager", "/controller_manager"]
    )

    # Spawner Nodes
    fts_spawner_nodes = []
    # Spawn fts and imu broadcasters for each robot
    for k, i in enumerate(ix):
        fts_broadcaster_name = f'fts_broadcaster_{i}'

        # FTS Spawner
        fts_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[fts_broadcaster_name, "--controller-manager", "/controller_manager"],
        )
        fts_spawner_nodes.append(fts_spawner)

        for axis_i in ['e','d','c','b','a']:
            gravity_broadcaster_name = f'gravity_broadcaster_{robot_prefixes[k]}_axis_{axis_i}'
            # gravity visualisation Spawner
            gravity_fts_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=[gravity_broadcaster_name, "--controller-manager", "/controller_manager"],
            )
            fts_spawner_nodes.append(gravity_fts_spawner)

    # Delay RViz start after `fts_broadcaster_`
    delay_rviz_after_fts_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fts_spawner,
            on_exit=[rviz_node],
        )
    )

    # Define other nodes if needed
    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler > /dev/null 2>&1'],
        output='screen',
        shell=True
    )

    mode = OpaqueFunction(function=lambda context: [])

    if task in ['interactive','manual', 'coverage','experimental']:
        try:
            _ = FindPackageShare("simlab").find("simlab")
            simlab_exists = True
        except Exception:
            simlab_exists = False

        if simlab_exists:
            if task == 'interactive':
                interactive_marker_node = Node(
                    package='simlab',
                    executable='interactive_marker_node',
                    name='interactive_marker_mode',
                    parameters=[{
                        'robots_prefix': robot_prefixes,
                        'no_robot': len(robot_prefixes),
                        'no_efforts': len(dof_efforts),
                        'record_data': record_data_bool,
                        'controllers': controllers
                    }]
                )
                mode = interactive_marker_node

            if task == 'manual':
                joystick_controller_node = Node(
                    package='simlab',
                    executable='joystick_controller',
                    name='joystick_controller',
                    parameters=[{
                        'robots_prefix': robot_prefixes,
                        'no_robot': len(robot_prefixes),
                        'no_efforts': len(dof_efforts),
                        'record_data': record_data_bool
                    }]
                )
                mode = joystick_controller_node
            elif task == 'coverage':
                coverage_node = Node(
                    package='simlab',
                    executable='coverage_node',
                    parameters=[{
                        'robots_prefix': robot_prefixes,
                        'no_robot': len(robot_prefixes),
                        'no_efforts': len(dof_efforts),
                        'record_data': record_data_bool,
                        'controllers': controllers
                    }]
                )
                mode = coverage_node
            elif task == 'experimental':
                experimental_node = Node(
                    package='simlab',
                    executable='experimental_node',
                    parameters=[{
                        'robots_prefix': robot_prefixes,
                        'no_robot': len(robot_prefixes),
                        'no_efforts': len(dof_efforts),
                        'record_data': record_data_bool,
                        'controllers': controllers
                    }]
                )
                mode = experimental_node
        else:
            raise Exception("""uvms simlab package not found. If you intend to run
                            the coverage example, interactive marker mode or manual control via PS4 joystick,
                            please install uvms_simlab from https://github.com/edxmorgan/uvms_simlab""")


    thruster_forward_pwm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_pwm_controller",
                "--controller-manager", "/controller_manager"],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("task"), "' == 'manual' and '",
                LaunchConfiguration("use_vehicle_hardware"), "' == 'true'"
            ])
        ),
    )

  # Define the simulator actions
    simulator_actions = [
        joint_state_broadcaster_spawner, #important
        control_node, 
        uvms_spawner,
        thruster_forward_pwm_spawner,
        mode,
        run_plotjuggler,
        robot_state_pub_node,
        delay_rviz_after_fts_broadcaster_spawner,
    ]
 
    # Define simulator_agent
    simulator_agent = GroupAction(
        actions=simulator_actions + fts_spawner_nodes,
    )

    # Launch nodes
    nodes = [
        simulator_agent

    ]
    
    return nodes
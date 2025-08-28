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

import yaml, copy
from launch_ros.actions import Node

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
    
def modify_controller_config(use_vehicle_hardware, use_manipulator_hardware, config_path,new_config_path,sim_robot_count:int=1):
    if not use_vehicle_hardware and not use_manipulator_hardware and sim_robot_count==0:
        raise Exception("Invalid configuration: No robots specified. Enable either vehicle or manipulator hardware, or specify at least one simulation robot.")

    with open(config_path,'r') as file:
        controller_param = yaml.load(file,yaml.SafeLoader)
    new_param = copy.deepcopy(controller_param)

    ix = []
    robot_prefixes = []
    robot_base_links = []

    if use_manipulator_hardware or use_vehicle_hardware:
        i = 'real'
        new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix)

    for i in range(1, sim_robot_count + 1):
        new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix)

    with open(new_config_path,'w') as file:
        yaml.dump(new_param,file,Dumper=NoAliasDumper)
    no_robots = len(robot_prefixes)
    return robot_prefixes, robot_base_links, ix, no_robots

def add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix):
    ix.append(i)
    prefix = f'robot_{i}_'

    robot_prefixes.append(prefix)

    base_link = f'{prefix}base_link'
    robot_base_links.append(base_link)

    vehicle_IOs = f'{prefix}IOs'

    # Force torque sensor broadcaster
    fts_broadcaster_name = f'fts_broadcaster_{i}'
    new_param['controller_manager']['ros__parameters'][fts_broadcaster_name] = {
        'type': 'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster'
    }

    new_param[fts_broadcaster_name] = {'ros__parameters': {
        'frame_id': base_link,
        'interface_names': {
            'force': {
                'x': f'{vehicle_IOs}/force.x',
                'y': f'{vehicle_IOs}/force.y',
                'z': f'{vehicle_IOs}/force.z'
                },
            'torque': {
                'x': f'{vehicle_IOs}/torque.x',
                'y': f'{vehicle_IOs}/torque.y',
                'z': f'{vehicle_IOs}/torque.z'
                }
            }
        }
    }

    # Vehicle effort controller
    vehicle_effort_ctrl_name = f'vehicle_effort_controller_{prefix}'
    new_param['controller_manager']['ros__parameters'][vehicle_effort_ctrl_name] = {
        'type': 'gpio_controllers/GpioCommandController'
    }
    new_param[vehicle_effort_ctrl_name] = {
        'ros__parameters': {
            'gpios': [vehicle_IOs],
            'command_interfaces': {
                vehicle_IOs: [
                    {'interfaces': [
                        'force.x', 'force.y', 'force.z',
                        'torque.x', 'torque.y', 'torque.z'
                    ]}
                ]
            },
        }
    }

    # Vehicle thrusters PWM controller
    thruster_ctrl_name = f'vehicle_thrusters_pwm_controller_{prefix}'
    new_param['controller_manager']['ros__parameters'][thruster_ctrl_name] = {
        'type': 'forward_command_controller/ForwardCommandController'
    }
    new_param[thruster_ctrl_name] = {
        'ros__parameters': {
            'joints': [
                f'{prefix}thruster1_joint',
                f'{prefix}thruster2_joint',
                f'{prefix}thruster3_joint',
                f'{prefix}thruster4_joint',
                f'{prefix}thruster5_joint',
                f'{prefix}thruster6_joint',
                f'{prefix}thruster7_joint',
                f'{prefix}thruster8_joint'
            ],
            'interface_name': 'pwm'
        }
    }

    # Manipulation effort controller
    manip_ctrl_name = f'manipulation_effort_controller_{prefix}'
    new_param['controller_manager']['ros__parameters'][manip_ctrl_name] = {
        'type': 'forward_command_controller/ForwardCommandController'
    }
    new_param[manip_ctrl_name] = {
        'ros__parameters': {
            'joints': [
                f'{prefix}_axis_e',
                f'{prefix}_axis_d',
                f'{prefix}_axis_c',
                f'{prefix}_axis_b',
                f'{prefix}_axis_a'
            ],
            'interface_name': 'effort'
        }
    }

    # Gravity force torque broadcasters for manipulator axes
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

def parse_controller_list(controllers_list_str: str, no_robots: int) -> list[str]:
    """
    Parse a comma-separated controller list.
    - If a single controller is given, replicate it for all robots.
    - If multiple are given, the count must match `no_robots`.
    """
    controllers = [c.strip() for c in controllers_list_str.split(',') if c.strip()]
    if not controllers:
        raise ValueError("No controllers specified (empty list after parsing).")

    if len(controllers) == 1:
        controllers *= no_robots
    elif len(controllers) != no_robots:
        raise ValueError(
            f"Controller count mismatch: got {len(controllers)} for {no_robots} robots. "
            f"Provide one controller (applied to all) or exactly {no_robots}."
        )
    return controllers

def controller_spawner_nodes(robot_i, robot_prefixes_k):
    fts_spawner_nodes = []
    fts_broadcaster_name = f'fts_broadcaster_{robot_i}'

    # FTS Spawner
    fts_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[fts_broadcaster_name, "--controller-manager", "/controller_manager"],
    )
    fts_spawner_nodes.append(fts_spawner)

    # controllers
    thruster_pwm_ctrl_name = f"vehicle_thrusters_pwm_controller_{robot_prefixes_k}"
    vehicle_effort_ctrl_name   = f"vehicle_effort_controller_{robot_prefixes_k}"
    manip_effort_ctrl_name    = f"manipulation_effort_controller_{robot_prefixes_k}"

    thruster_pwm_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[thruster_pwm_ctrl_name, "--controller-manager", "/controller_manager", "--inactive"],
    )
    thruster_pwm_ctrl_spawner.controller_name = thruster_pwm_ctrl_name  # attach

    vehicle_effort_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[vehicle_effort_ctrl_name, "--controller-manager", "/controller_manager", "--inactive"],
    )
    vehicle_effort_ctrl_spawner.controller_name = vehicle_effort_ctrl_name  # attach

    manip_effort_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[manip_effort_ctrl_name, "--controller-manager", "/controller_manager", "--inactive"],
    )
    manip_effort_ctrl_spawner.controller_name = manip_effort_ctrl_name  # attach

    # joint gravity effect broadcaster
    for axis_i in ['e','d','c','b','a']:
        gravity_broadcaster_name = f'gravity_broadcaster_{robot_prefixes_k}_axis_{axis_i}'
        # gravity visualisation Spawner
        gravity_fts_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[gravity_broadcaster_name, "--controller-manager", "/controller_manager"],
        )
        fts_spawner_nodes.append(gravity_fts_spawner)
    return fts_spawner_nodes, thruster_pwm_ctrl_spawner, vehicle_effort_ctrl_spawner, manip_effort_ctrl_spawner

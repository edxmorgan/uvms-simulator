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

import random
import yaml, copy

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
        # Plane billboard cloud from /alpha/image_raw
        rviz_point_cloud2(
            name='alpha_plane_cloud',
            topic='/alpha/points_plane',
            rviz_config=new_rviz_config,
            enabled=False,  # default false as requested
            color='255; 255; 255',
            custom_properties={
                "Alpha": 1,
                "Autocompute Intensity Bounds": True,
                "Autocompute Value Bounds": {"Max Value": 10, "Min Value": -10, "Value": True},
                "Axis": "Z",
                "Channel Name": "intensity",
                "Class": "rviz_default_plugins/PointCloud2",
                "Color Transformer": "RGB8",
                "Decay Time": 0,
                "Invert Rainbow": False,
                "Max Color": "255; 255; 255",
                "Max Intensity": 4096,
                "Min Color": "0; 0; 0",
                "Min Intensity": 0,
                "Position Transformer": "XYZ",
                "Selectable": True,
                "Size (Pixels)": 3,
                "Size (m)": 0.009999999776482582,
                "Style": "Flat Squares",
                "Topic": {
                    "Depth": 5,
                    "Durability Policy": "Volatile",
                    "History Policy": "Keep Last",
                    "Reliability Policy": "Reliable",
                    "Value": "/alpha/points_plane",
                },
                "Use Fixed Frame": True,
                "Use rainbow": True,
                "Value": True,
            }
        )


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
        rviz_axes_display(f'{prefix}dvl_frame', f"{prefix}dvl_link", rviz_config, 0.1, 0.01, True)
        for i in range(5):
            rviz_axes_display(f'{prefix}joint_{i}', f"{prefix}joint_{i}", rviz_config, 0.1, 0.01, True)
        
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

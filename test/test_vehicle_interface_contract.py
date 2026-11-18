"""Check hardware Xacro's GPIO contract without configuring physical devices."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import xacro
import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('manual', ['true', 'false'])
@pytest.mark.parametrize('dvl', ['true', 'false'])
@pytest.mark.parametrize('real', ['true', 'false'])
def test_hardware_gpio_count_matches_driver_initialization(manual, dvl, real):
    description = ROOT / 'description/ros2_control/vehicle_system_multi_interface.ros2_control.xacro'
    doc = xacro.parse(f'''<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{description}"/>
      <xacro:vehicle_system_multi_interface name="real_BlueSystemMultiInterface"
        prefix="robot_real_" serial_port="unused" state_update_frequency="70"
        use_hardware="{real}" world_frame_id="base_link" body_frame_id="robot_real_base_link"
        map_frame_id="robot_real_map" same_initial_conditions="false" use_pwm="false"
        manual_control="{manual}" use_dvl="{dvl}" dvl_host="unused" dvl_port="16171"/>
    </robot>''')
    xacro.process_doc(doc)
    hardware = ET.fromstring(doc.toxml()).find('ros2_control/hardware')
    params = {p.attrib['name']: p.text for p in hardware.findall('param')}
    if real == 'true':
        assert params['manual_control'].lower() == manual
        assert params['use_dvl'].lower() == dvl
    gpio = ET.fromstring(doc.toxml()).find('ros2_control/gpio')
    names = [s.attrib['name'] for s in gpio.findall('state_interface')]
    source = (ROOT / ('hardware/bluerov_system_multi_interface.cpp' if real == 'true'
                      else 'hardware/sim_vehicle_system_multi_interface.cpp')).read_text()
    pattern = (r'expected_gpio_state_interfaces\s*=\s*(\d+)' if real == 'true'
               else r'gpio.state_interfaces.size\(\) != (\d+)')
    expected = int(re.search(pattern, source)[1])
    assert len(names) == len(set(names)) == expected
    assert {f'{kind}_acceleration_estimate.{axis}' for kind in ['linear', 'angular']
            for axis in 'xyz'} <= set(names)
    assert len(gpio.findall('command_interface')) == 28
    assert {f'estimator_noise.intensity.{i}' for i in range(6)} <= set(names)
    assert {'estimator_noise.'+n for n in ['enabled', 'applied_fit_count', 'dropped', 'failed', 'fit_seconds', 'endpoint_step']} <= set(names)
    identification = {'identification.' + name for name in
        ['valid', 'generation', 'step', 'time', 'period', 'depth']
        + [f'attitude.{i}' for i in range(3)]
        + [f'{group}.{i}' for group in ['velocity', 'wrench'] for i in range(6)]}
    assert {n for n in names if n.startswith('identification.')} == identification
    if real == 'false':
        assert {'estimator_model_retained', 'estimator_model_epoch',
                'estimator_model_accepted_time'} <= set(names)


@pytest.mark.parametrize('real', ['true', 'false'])
@pytest.mark.parametrize('locks', ['true', 'false'])
def test_arm_disturbance_gpio_contract(real, locks):
    description = ROOT / 'description/ros2_control/robot_system_multi_interface.ros2_control.xacro'
    doc = xacro.parse(f'''<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{description}"/>
      <xacro:robot_system_multi_interface name="Arm" prefix="robot_1_"
        serial_port="unused" state_update_frequency="100" use_hardware="{real}"
        arm_joint_locks="{locks}"/>
    </robot>''')
    xacro.process_doc(doc)
    hardware = ET.fromstring(doc.toxml()).find('ros2_control/hardware')
    params = {p.attrib['name']: p.text for p in hardware.findall('param')}
    if real == 'false':
        assert params['arm_joint_locks'].lower() == locks
    else:
        assert 'arm_joint_locks' not in params
    gpio = ET.fromstring(doc.toxml()).find('ros2_control/gpio')
    names = [s.attrib['name'] for s in gpio.findall('state_interface')]
    assert len(names) == len(set(names)) == (170 if real == 'true' else 179)
    assert {'identification.time', 'identification.supported'} <= set(names)
    assert {f'identification.{field}.{i}' for field in ('position', 'velocity', 'impulse', 'valid', 'generation') for i in range(5)} <= set(names)
    assert {f'identification.context.{i}' for i in range(15)} <= set(names)
    assert {f'estimated_disturbance.covariance.{i}.{j}' for i in range(4) for j in range(4)} <= set(names)
    if real == 'false':
        assert {'estimator_step', 'estimator_valid'} <= set(names)
        assert {f'identification.{field}.{i}' for field in ('position', 'velocity', 'impulse', 'valid', 'generation') for i in range(5)} <= set(names)
        assert {f'identification.context.{i}' for i in range(15)} <= set(names)
        assert {f'estimator_nominal_parameter.{i}' for i in range(81)} <= set(names)
        assert {f'estimated_disturbance.torque.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimated_disturbance.variance.torque.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimator_joint_locked.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimator_joint_at_limit.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimator_disturbance_observable.{axis}' for axis in 'edcb'} <= set(names)


def test_recording_and_estimated_tf_broadcasters_are_isolated():
    import yaml
    config = yaml.safe_load((ROOT / 'bringup/config/robot_multi_interface_forward_controllers.yaml').read_text())
    raw = config['joint_state_broadcaster']['ros__parameters']
    assert raw['use_local_topics'] is True
    assert raw['map_interface_to_joint_state']['position'] == 'position'
    assert raw['map_interface_to_joint_state']['velocity'] == 'velocity'
    estimated = config['estimated_joint_state_broadcaster']['ros__parameters']
    assert estimated['interfaces'] == ['filtered_position', 'filtered_velocity']
    assert estimated['use_local_topics'] is True
    auxiliary = config['auxiliary_joint_state_broadcaster']['ros__parameters']
    assert auxiliary['interfaces'] == ['position']
    assert auxiliary['use_local_topics'] is True  # never publish to the raw recording topic


@pytest.mark.parametrize('count', ['1', '2'])
def test_standard_launch_resolves_urdf_before_selecting_feedback_joints(tmp_path, monkeypatch, count):
    import importlib.util
    import yaml
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    path = ROOT / 'bringup/launch/robot_system_multi_interface.launch.py'
    spec = importlib.util.spec_from_file_location('state_boundary_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '_runtime_file_path', lambda name: str(tmp_path/name))
    monkeypatch.setattr(module, '_cleanup_stale_processes', lambda _: pytest.fail('must not stop processes'))
    # Exercise the actual constructed node parameters with a non-default rate,
    # so a hard-coded 200 Hz or RSP's implicit 20 Hz cannot satisfy the contract.
    requested_rate = 137 + int(count)
    configure_controllers = module.modify_controller_config

    def configure_at_test_rate(*args, **kwargs):
        result = configure_controllers(*args, **kwargs)
        path = next(tmp_path.glob('robot_multi_interface_forward_controllers_*.yaml'))
        config = yaml.safe_load(path.read_text())
        config['controller_manager']['ros__parameters']['update_rate'] = requested_rate
        path.write_text(yaml.safe_dump(config))
        return result

    node_specs = []
    make_node = module.Node

    def capture_node(**kwargs):
        node_specs.append(kwargs)
        return make_node(**kwargs)

    monkeypatch.setattr(module, 'modify_controller_config', configure_at_test_rate)
    monkeypatch.setattr(module, 'Node', capture_node)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(use_vehicle_hardware='false', use_manipulator_hardware='false',
        sim_robot_count=count, task='interactive', cleanup_stale_nodes='false',
        launch_rviz='false', launch_plotjuggler='false', launch_camera='true')
    assert module.launch_setup(context)
    config = yaml.safe_load(next(tmp_path.glob('robot_multi_interface_forward_controllers_*.yaml')).read_text())
    estimated = config['estimated_joint_state_broadcaster']['ros__parameters']['joints']
    auxiliary = config['auxiliary_joint_state_broadcaster']['ros__parameters']['joints']
    assert len(estimated) == 5 * int(count)
    assert len(auxiliary) == 9 * int(count)
    assert not set(estimated) & set(auxiliary)
    robot_publishers = {
        spec['name']: spec for spec in node_specs
        if spec.get('package') == 'robot_state_publisher'
        and spec.get('name', '').startswith('robot_state_publisher_robot_')
    }
    expected_names = {f'robot_state_publisher_robot_{i}{suffix}'
                      for i in range(1, int(count) + 1)
                      for suffix in ('', '_physical')}
    assert robot_publishers.keys() == expected_names
    for spec in robot_publishers.values():
        parameters = next(p for p in spec['parameters'] if isinstance(p, dict))
        assert parameters['publish_frequency'] == requested_rate


@pytest.mark.parametrize('model,macro', [
    ('bluerov2/urdf.xacro', 'bluerov2_urdf'),
    ('bluerov2_heavy/urdf.xacro', 'bluerov2_heavy_urdf'),
    ('bluerov2_heavy_reach/bruce_urdf.xacro', 'bluerov2_heavy_reach_urdf'),
    ('bluerov2_heavy_reach/dory_urdf.xacro', 'bluerov2_heavy_reach_urdf'),
])
def test_vehicle_urdf_owns_dvl_mount_for_every_robot(model, macro, monkeypatch):
    import ament_index_python.packages as packages
    installed_share = packages.get_package_share_directory
    monkeypatch.setattr(packages, 'get_package_share_directory',
        lambda name: str(ROOT / 'description') if name == 'ros2_control_blue_reach_5'
        else installed_share(name))
    description = ROOT / 'description/blue/xacro' / model
    prefixes = ['robot_real_', 'robot_1_', 'robot_2_']
    instances = ''.join(f'<xacro:{macro} prefix="{prefix}"/>' for prefix in prefixes)
    doc = xacro.parse(f'''<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{description}"/>{instances}
    </robot>''')
    xacro.process_doc(doc)
    robot = ET.fromstring(doc.toxml())
    for prefix in prefixes:
        assert len(robot.findall(f"link[@name='{prefix}dvl_link']")) == 1
        parents = [joint for joint in robot.findall('joint')
                   if joint.find('child').get('link') == prefix + 'dvl_link']
        assert len(parents) == 1
        mount = parents[0]
        assert mount.get('type') == 'fixed'
        assert mount.find('parent').get('link') == prefix + 'base_link'
        assert [float(x) for x in mount.find('origin').get('xyz').split()] == [-.06, 0., -.105]
        assert [float(x) for x in mount.find('origin').get('rpy').split()] == [0., 0., 0.]
        assert robot.find(f"link[@name='{prefix}camera_link']") is not None

        for sensor in ('imu', 'pressure'):
            joint = robot.find(f"joint[@name='{prefix}{sensor}_joint']")
            assert joint.get('type') == 'fixed'
            assert joint.find('parent').get('link') == prefix + 'base_link'
            assert joint.find('child').get('link') == prefix + sensor + '_link'

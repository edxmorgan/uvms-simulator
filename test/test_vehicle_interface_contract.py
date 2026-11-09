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
    assert {n for n in names if n.startswith('identification.')} == (identification if real == 'true' else set())
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
    assert len(names) == len(set(names)) == 128
    assert {f'estimated_disturbance.covariance.{i}.{j}' for i in range(4) for j in range(4)} <= set(names)
    if real == 'false':
        assert {'estimator_step', 'estimator_valid'} <= set(names)
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
    assert len(auxiliary) == 8 * int(count)
    assert not set(estimated) & set(auxiliary)

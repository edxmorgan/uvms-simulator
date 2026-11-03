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
    assert len(gpio.findall('command_interface')) == 28
    identification = {'identification.' + name for name in
        ['valid', 'generation', 'step', 'time', 'period', 'depth']
        + [f'attitude.{i}' for i in range(3)]
        + [f'{group}.{i}' for group in ['velocity', 'wrench'] for i in range(6)]}
    assert {n for n in names if n.startswith('identification.')} == (identification if real == 'true' else set())



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
    assert len(names) == len(set(names)) == 112
    if real == 'false':
        assert {'estimator_step', 'estimator_valid'} <= set(names)
        assert {f'estimator_nominal_parameter.{i}' for i in range(81)} <= set(names)
        assert {f'estimated_disturbance.torque.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimated_disturbance.variance.torque.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimator_joint_locked.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimator_joint_at_limit.{axis}' for axis in 'edcb'} <= set(names)
        assert {f'estimator_disturbance_observable.{axis}' for axis in 'edcb'} <= set(names)

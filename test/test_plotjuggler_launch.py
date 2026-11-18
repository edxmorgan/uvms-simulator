"""Exercise installation selection without launching a GUI or requiring Snap."""
import importlib.util
from pathlib import Path
import pytest

PATH = Path(__file__).resolve().parents[1] / 'bringup/launch/robot_system_multi_interface.launch.py'

@pytest.mark.parametrize('v4,v3,snap,expected', [
    ('/local/bin/plotjuggler4', '/snap/bin/plotjuggler', True, ['/local/bin/plotjuggler4']),
    (None, '/usr/bin/plotjuggler', True, ['/usr/bin/plotjuggler', '--buffer_size', '10']),
    (None, '/snap/bin/plotjuggler', True, ['/snap/bin/plotjuggler', '--buffer_size', '10']),
    (None, None, True, ['/snap/bin/plotjuggler', '--buffer_size', '10']),
    (None, None, False, ['plotjuggler', '--buffer_size', '10']),
])
def test_plotjuggler_installation_selection(monkeypatch, v4, v3, snap, expected):
    spec = importlib.util.spec_from_file_location('plotjuggler_launch', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.shutil, 'which', lambda name: {'plotjuggler4': v4, 'plotjuggler': v3}[name])
    monkeypatch.setattr(module.os, 'access', lambda path, mode: snap and path == '/snap/bin/plotjuggler')
    assert module._plotjuggler_command('10') == expected


def test_launch_math_thread_defaults_respect_user_environment(monkeypatch):
    from launch import LaunchContext
    from launch.actions import SetEnvironmentVariable
    spec = importlib.util.spec_from_file_location('simulation_launch', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in ('OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'BLIS_NUM_THREADS'):
        monkeypatch.delenv(name, raising=False)
    for override in (None, '2'):
        if override is not None:
            monkeypatch.setenv('OPENBLAS_NUM_THREADS', override)
        context = LaunchContext()
        actions = module.generate_launch_description().entities
        for action in actions:
            if isinstance(action, SetEnvironmentVariable):
                action.execute(context)
        assert context.environment['OPENBLAS_NUM_THREADS'] == (override or '1')
        assert context.environment['MKL_NUM_THREADS'] == '1'
        assert context.environment['BLIS_NUM_THREADS'] == '1'


def test_two_robot_launch_descriptions_are_explicit_xml_strings(monkeypatch, tmp_path):
    """Evaluate the actual launch handoffs, including XML comments with colons."""
    import xml.etree.ElementTree as ET
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    spec = importlib.util.spec_from_file_location('description_launch', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'use_vehicle_hardware':'false', 'use_manipulator_hardware':'false',
        'sim_robot_count':'2', 'task':'interactive', 'launch_voxelviz':'true',
        'cleanup_stale_nodes':'false', 'gui':'false'})
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    captured = []
    original_node = module.Node
    def node(*args, **kwargs):
        captured.extend(p for p in kwargs.get('parameters', [])
                        if isinstance(p, dict) and 'robot_description' in p)
        return original_node(*args, **kwargs)
    monkeypatch.setattr(module, 'Node', node)
    monkeypatch.setattr(module, '_runtime_file_path', lambda name: str(tmp_path / name))
    module.launch_setup(context)
    assert len(captured) >= 10
    descriptions = []
    for parameters in captured:
        description = evaluate_parameters(context, normalize_parameters([
            {'robot_description':parameters['robot_description']}]))[0]['robot_description']
        assert isinstance(description, str)
        assert ET.fromstring(description).tag == 'robot'
        descriptions.append(description)
    assert any('Lumen emitter frames:' in value for value in descriptions)

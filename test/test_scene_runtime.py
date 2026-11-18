from pathlib import Path
from types import SimpleNamespace as NS
import threading
import xml.etree.ElementTree as ET

import pytest
from bringup.scene_runtime import SceneParticipant, replace_environment, is_environment

ROOT = Path(__file__).resolve().parents[1]

class Node:
    def create_service(self, *args):
        return None


def send(p, command, revision=1, description='new'):
    return p.handle(NS(command=command, revision=revision, robot_description=description),
                    NS(success=False, message='', revision=0))


def test_scene_handoff_requires_matching_hold_and_prepare():
    commits = []
    p = SceneParticipant(Node(), lambda d:d, commits.append)
    assert not send(p, 'commit').success
    assert send(p, 'hold').success
    assert not send(p, 'release').success
    assert send(p, 'prepare').success
    assert send(p, 'commit').success
    assert send(p, 'commit').success
    assert commits == ['new']
    assert send(p, 'verify').success
    assert send(p, 'release').success
    assert not p.held
    assert not send(p, 'commit').success
    assert send(p, 'hold', 2).success
    assert not send(p, 'prepare', 1).success
    assert not send(p, 'hold', 1).success
    assert commits == ['new']


def test_failed_prepare_or_commit_never_releases():
    def fail(_):
        raise ValueError('geometry unavailable')
    for prepare, commit in [(fail, lambda _:None), (lambda d:d, fail)]:
        p = SceneParticipant(Node(), prepare, commit)
        send(p, 'hold'); send(p, 'prepare'); send(p, 'commit')
        assert p.held
        assert not send(p, 'release').success
        assert p.committed == 0


def test_scene_replacement_preserves_robot_and_uses_stable_geometry_frames():
    original = '<robot name="robot"><link name="world"/><link name="robot_1_base_link"/><link name="bathymetry_old"/><joint name="old" type="fixed"><parent link="world"/><child link="bathymetry_old"/></joint></robot>'
    world = ROOT/'description/xacro/world.xacro'
    a = replace_environment(original, ROOT/'description/scenes/pipeline/world.xacro', 'world', 1, world)
    b = replace_environment(a, ROOT/'description/scenes/modern_ship/world.xacro', 'world', 2, world)
    c = replace_environment(b, ROOT/'description/scenes/pipeline/world.xacro', 'world', 3, world)
    def links(text):
        return {l.get('name') for l in ET.fromstring(text).findall('link')}
    assert links(a) == links(c)
    assert links(a) & links(b) == {'world', 'robot_1_base_link'}
    assert 'bathymetry_old' not in links(a)
    assert len([n for n in links(a) if is_environment(n)]) == 4


def test_manager_failure_never_calls_plant_services(monkeypatch):
    from bringup.scene_manager import SceneManager
    import bringup.scene_manager as module
    monkeypatch.setattr(module, 'replace_environment', lambda *a:'new')
    monkeypatch.setattr(module, 'environment_transforms', lambda *a:[])
    for failure in ('prepare', 'commit', 'verify', 'release'):
        m = SceneManager.__new__(SceneManager)
        m.description = 'old'; m.world_frame = 'world'; m.share = Path('/tmp')
        m.consumers = []
        m.lock = threading.Lock(); m.busy = True; events=[]
        m.node = NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None)),
                    get_logger=lambda:NS(error=lambda _:None))
        m.tf = NS(sendTransform=lambda _:events.append('tf'))
        m._cache_floor_frame = lambda _:None
        m.publish_status = lambda *a:events.append(a[0])
        m.publish_description = lambda _:events.append('description')
        def phase(name, *args):
            events.append(name)
            if name == failure:
                raise RuntimeError('test failure')
        m.phase = phase
        m.switch('new', '/tmp/new', 1)
        assert 'reset' not in events and 'pause' not in events
        assert events.index('hold') < events.index('prepare')
        assert events[-1] == 'failed'
        assert not m.busy


def test_default_scene_preserves_nested_fixed_environment_frames():
    original = '<robot name="robot"><link name="world"/></robot>'
    result = replace_environment(original, '', 'world', 1, ROOT/'description/xacro/world.xacro')
    root = ET.fromstring(result)
    links = {link.get('name') for link in root.findall('link')}
    for joint in root.findall('joint'):
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links
    seabed = next(j for j in root.findall('joint') if j.find('child').get('link').startswith('bathymetry_seafloor'))
    assert seabed.find('parent').get('link').startswith('world_bottom_scene_')


def test_preparation_is_concurrent_and_failure_waits_for_other_consumers():
    from bringup.scene_manager import SceneManager
    import time
    m = SceneManager.__new__(SceneManager)
    m.consumers = [('reject', 'reject'), ('slow', 'slow')]
    m.node = NS(get_logger=lambda: NS(info=lambda _: None))
    barrier = threading.Barrier(2, timeout=2)
    completed = threading.Event()
    def call(client, request):
        barrier.wait()
        if client == 'reject':
            raise RuntimeError('overlap')
        time.sleep(.05)
        completed.set()
        return NS(revision=request.revision)
    m.call = call
    with pytest.raises(RuntimeError, match='overlap'):
        m.phase('prepare', 1, 'description')
    assert completed.is_set()  # no late prepare can race a retry


def test_floor_frame_reuses_geometry_and_refreshes_on_scene_commit(monkeypatch):
    from bringup.scene_manager import SceneManager
    import bringup.scene_manager as module
    from geometry_msgs.msg import TransformStamped
    from builtin_interfaces.msg import Time
    from unittest.mock import Mock
    parsed = []
    def transforms(description, stamp):
        parsed.append(description)
        result = TransformStamped()
        result.header.frame_id = 'world'
        result.child_frame_id = 'world_bottom_scene_' + description
        result.transform.translation.z = -float(description)
        result.transform.rotation.w = 1.
        return [result]
    monkeypatch.setattr(module, 'environment_transforms', transforms)
    manager = SceneManager.__new__(SceneManager)
    manager.node = Mock()
    manager.node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=10)
    manager.floor_tf = Mock()
    manager._cache_floor_frame('8')
    for _ in range(5):
        manager.publish_floor_frame()
    assert parsed == ['8']
    frame = manager.floor_tf.sendTransform.call_args.args[0]
    assert frame.child_frame_id == 'environment_floor' and frame.transform.translation.z == -8.
    manager._cache_floor_frame('3')
    manager.node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=11)
    manager.publish_floor_frame()
    frame = manager.floor_tf.sendTransform.call_args.args[0]
    assert frame.transform.translation.z == -3. and frame.header.stamp.sec == 11
    assert parsed == ['8', '3']


def test_scene_preparation_does_not_block_callback_thread():
    ready = threading.Event()
    entered = threading.Event()
    def prepare(description):
        entered.set()
        assert ready.wait(2.)
        return description
    p = SceneParticipant(Node(), prepare, lambda _:None, prepare_async=True)
    assert send(p, 'hold').success
    try:
        assert send(p, 'prepare').message == 'pending'
        assert entered.wait(1.)
        assert not p.prepared
        assert send(p, 'status').success
        assert not send(p, 'commit').success
        assert not send(p, 'hold', 2).success
    finally:
        ready.set()
    p.preparation.result(timeout=2.)
    assert send(p, 'prepare').message == 'ready'
    assert send(p, 'commit').success
    assert send(p, 'release').success


def test_robot_ownership_follows_tree_including_arm_and_gripper_bases():
    from bringup.scene_runtime import robot_link_owners
    description = '''<robot name="test">
      <link name="world"/><link name="robot_real_base_link"/>
      <link name="robot_real__base_link"/><link name="robot_real__jaws_base_link"/>
      <link name="tool"/><link name="robot_2_base_link"/>
      <joint name="a"><parent link="robot_real_base_link"/><child link="robot_real__base_link"/></joint>
      <joint name="b"><parent link="robot_real__base_link"/><child link="robot_real__jaws_base_link"/></joint>
      <joint name="c"><parent link="robot_real__jaws_base_link"/><child link="tool"/></joint>
    </robot>'''
    owners = robot_link_owners(description)
    assert set(owners.values()) == {'robot_real_', 'robot_2_'}
    assert owners['tool'] == owners['robot_real__base_link'] == 'robot_real_'
    assert 'world' not in owners


def test_geometry_preparation_keeps_single_threaded_ros_timer_running():
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('scene_preparation_timer_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    finished = threading.Event()
    def prepare(description):
        finished.wait(.25)
        return description
    participant = SceneParticipant(node, prepare, lambda _:None, prepare_async=True)
    ticks = []
    def tick():
        ticks.append(time.monotonic())
        if len(ticks) == 1:
            assert send(participant, 'hold').success
            assert send(participant, 'prepare').success
    timer = node.create_timer(.01, tick)
    try:
        deadline = time.monotonic() + .2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert len(ticks) >= 8
        assert not participant.prepared
    finally:
        finished.set()
        if participant.preparation is not None:
            participant.preparation.result(timeout=2.)
        node.destroy_timer(timer)
        executor.remove_node(node)
        node.destroy_node()
        context.shutdown()

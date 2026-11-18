"""Orchestrate shared environment replacement independently of robot state."""
import json
from pathlib import Path
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy
from ros2_control_blue_reach_5.srv import EnvironmentScene
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from bringup.environment_scene import resolve_environment_scene
from bringup.scene_runtime import replace_environment, environment_transforms


class SceneManager:
    def __init__(self, node, description, publish_description):
        self.node = node; self.description = description; self.publish_description = publish_description
        node.declare_parameter('scene_switch_enabled', False)
        node.declare_parameter('scene_consumers', ['interactive_controller', 'planner_action_server_node'])
        node.declare_parameter('environment_scene', 'default')
        node.declare_parameter('world_frame', 'world')
        self.enabled = bool(node.get_parameter('scene_switch_enabled').value)
        self.world_frame = node.get_parameter('world_frame').value
        self.scene = node.get_parameter('environment_scene').value
        self.target_scene = self.scene
        self.share = Path(get_package_share_directory('ros2_control_blue_reach_5'))
        self.lock = threading.Lock(); self.busy = False; self.failed = False
        self.revision = 0; self.state = 'ready'; self.error = ''
        self.consumers = [(name, node.create_client(EnvironmentScene, f'/{name}/environment_scene'))
                          for name in node.get_parameter('scene_consumers').value]
        self.tf = StaticTransformBroadcaster(node)
        self.floor_tf = TransformBroadcaster(node)
        self._cache_floor_frame(description)
        self.floor_timer = node.create_timer(0.1, self.publish_floor_frame)
        self.status_pub = node.create_publisher(String, '/environment_scene/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.service = node.create_service(EnvironmentScene, '/environment_scene/command', self.handle)
        self.publish_status()

    def _cache_floor_frame(self, description):
        transform = next((tf for tf in environment_transforms(
            description, self.node.get_clock().now().to_msg())
            if tf.child_frame_id.startswith('world_bottom')), None)
        if transform is not None:
            transform.child_frame_id = 'environment_floor'
        self._floor_frame = transform

    def publish_floor_frame(self):
        # Stable visualization frame. Scene geometry keeps its versioned static
        # frames; never overwrite the old world's latched world_bottom transform.
        transform = self._floor_frame
        if transform is not None:
            transform.header.stamp = self.node.get_clock().now().to_msg()
            self.floor_tf.sendTransform(transform)

    def publish_status(self, state=None, error=''):
        if state is not None:
            self.state = state; self.error = error
        self.status_pub.publish(String(data=json.dumps(dict(scene=self.scene, revision=self.revision,
            state=self.state, target_scene=self.target_scene, error=self.error, enabled=self.enabled,
            consumers=[n for n, _ in self.consumers]))))

    def handle(self, request, response):
        with self.lock:
            response.revision = self.revision
            if request.command == 'status':
                response.success = True; response.message = f'{self.state}: {self.scene}; {self.error}'
            elif request.command != 'switch':
                response.message = 'Use switch or status'
            elif not self.enabled:
                response.message = 'Runtime scene switching requires an interactive launch'
            elif self.busy:
                response.message = 'Scene switch already running'
            else:
                try:
                    path = resolve_environment_scene(request.scene, self.share)
                except Exception as exc:
                    response.message = str(exc); return response
                self.busy = True
                self.target_scene = request.scene
                self.revision = max(self.revision + 1, time.time_ns())
                response.revision = self.revision
                self.publish_status('loading')
                threading.Thread(target=self.switch, args=(request.scene, path, self.revision),
                                 daemon=True, name='environment_scene_switch').start()
                response.success = True
                response.message = 'Scene change accepted; follow /environment_scene/status for completion'
        return response

    @staticmethod
    def call(client, request, timeout=90.):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        done = threading.Event(); future = client.call_async(request)
        future.add_done_callback(lambda _:done.set())
        if not done.wait(timeout):
            # The request may still execute. Keep scene-dependent requests blocked; do not assume rollback.
            raise RuntimeError(f'Timeout: {client.srv_name}; scene transition remains blocked')
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(f'{client.srv_name}: {getattr(result, "message", "no response")}')
        return result

    def phase(self, command, revision, description=''):
        def request_consumer(consumer):
            name, client = consumer
            request = EnvironmentScene.Request(command=command, revision=revision,
                                                robot_description=description)
            started = time.monotonic()
            result = self.call(client, request)
            if command == 'prepare':
                deadline = time.monotonic() + 15.0
                while result.message == 'pending':
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'{name}: scene preparation timed out')
                    time.sleep(0.05)
                    result = self.call(client, request)
            self.node.get_logger().info(f'Scene timing {command} {name}: {time.monotonic()-started:.3f} s')
            if command in ('commit', 'verify', 'release') and result.revision != revision:
                raise RuntimeError(f'{name}: scene revision acknowledgement mismatch')
        if command == 'prepare' and self.consumers:
            # Independent processes stage their own geometry concurrently. Exit
            # the pool only after every request finishes, even when one fails.
            # Commit remains behind this complete preparation barrier.
            with ThreadPoolExecutor(max_workers=len(self.consumers)) as pool:
                list(pool.map(request_consumer, self.consumers))
        else:
            for consumer in self.consumers:
                request_consumer(consumer)

    def switch(self, scene, path, revision):
        try:
            # Missing consumers must not result in a best-effort partial scene update.
            for name, client in self.consumers:
                if not client.wait_for_service(timeout_sec=3.):
                    raise RuntimeError('Required participant unavailable: '+name)
            self.phase('hold', revision)
            description = replace_environment(self.description, path, self.world_frame, revision,
                                               self.share/'xacro/world.xacro')
            self.publish_status('preparing')
            self.phase('prepare', revision, description)
            self.publish_status('committing')
            self.phase('commit', revision)
            # New frame IDs prevent stale latched /tf_static from previous scenes
            # being mistaken for transforms of the replacement scene.
            self.tf.sendTransform(environment_transforms(description, self.node.get_clock().now().to_msg()))
            self.phase('verify', revision)
            self.description = description; self.scene = scene
            self._cache_floor_frame(description)
            self.publish_description(description)
            self.phase('release', revision)
            self.failed = False
            self.publish_status('ready')
        except Exception as exc:
            self.failed = True
            self.publish_status('failed', str(exc))
            self.node.get_logger().error(f'Scene switch failed: {exc}. Select a scene again to retry; no automatic resume.')
        finally:
            with self.lock:
                self.busy = False

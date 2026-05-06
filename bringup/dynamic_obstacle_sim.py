#!/usr/bin/env python3
# Copyright (C) 2026 Edward Morgan

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from ros2_control_blue_reach_5.msg import DynamicObstacle, DynamicObstacleArray
from ros2_control_blue_reach_5.srv import SetDynamicObstacles
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


def geometry_type(value) -> int:
    if isinstance(value, int):
        return int(value)
    mapping = {
        "none": DynamicObstacle.GEOMETRY_NONE,
        "sphere": DynamicObstacle.GEOMETRY_SPHERE,
        "box": DynamicObstacle.GEOMETRY_BOX,
        "cylinder": DynamicObstacle.GEOMETRY_CYLINDER,
        "mesh": DynamicObstacle.GEOMETRY_MESH,
    }
    key = str(value).strip().lower()
    if key not in mapping:
        raise ValueError(f"unknown obstacle geometry type '{value}'")
    return mapping[key]


def vector(config: dict, key: str, size: int, default: list[float]) -> list[float]:
    values = [float(v) for v in list(config.get(key, default))]
    if len(values) != size:
        raise ValueError(f"obstacle '{key}' must contain exactly {size} values")
    return values


def obstacle_from_config(config: dict, index: int) -> DynamicObstacle:
    if not isinstance(config, dict):
        raise ValueError(f"obstacles[{index}] must be an object")
    obstacle = DynamicObstacle()
    obstacle.id = str(config.get("id", f"obstacle_{index}"))
    obstacle.collision_type = geometry_type(config.get("type", config.get("collision_type", "sphere")))
    obstacle.collision_dimensions = [float(v) for v in list(config.get("dimensions", config.get("collision_dimensions", [])))]
    if not obstacle.collision_dimensions:
        raise ValueError(f"obstacle '{obstacle.id}' is missing dimensions")
    obstacle.visual_type = geometry_type(config.get("visual_type", obstacle.collision_type))
    obstacle.visual_dimensions = [float(v) for v in list(config.get("visual_dimensions", obstacle.collision_dimensions))]
    obstacle.visual_mesh_resource = str(config.get("visual_mesh_resource", ""))

    position = vector(config, "position", 3, [0.0, 0.0, 0.0])
    orientation = vector(config, "orientation", 4, [0.0, 0.0, 0.0, 1.0])
    obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z = position
    obstacle.pose.orientation.x, obstacle.pose.orientation.y, obstacle.pose.orientation.z, obstacle.pose.orientation.w = orientation

    linear = vector(config, "linear_velocity", 3, [0.0, 0.0, 0.0])
    angular = vector(config, "angular_velocity", 3, [0.0, 0.0, 0.0])
    obstacle.twist.linear.x, obstacle.twist.linear.y, obstacle.twist.linear.z = linear
    obstacle.twist.angular.x, obstacle.twist.angular.y, obstacle.twist.angular.z = angular

    color = vector(config, "color", 4, [0.95, 0.42, 0.12, 0.75])
    obstacle.color.r, obstacle.color.g, obstacle.color.b, obstacle.color.a = color
    return obstacle


def obstacle_array_from_profile(profile: dict, default_frame_id: str) -> DynamicObstacleArray:
    obstacles_config = profile.get("obstacles", profile)
    frame_id = default_frame_id
    items = obstacles_config
    if isinstance(obstacles_config, dict):
        frame_id = str(obstacles_config.get("frame_id", default_frame_id) or default_frame_id)
        items = obstacles_config.get("items", [])
    if not isinstance(items, list):
        raise ValueError("obstacles must be a list or an object with an items list")

    msg = DynamicObstacleArray()
    msg.header.frame_id = frame_id
    msg.obstacles = [obstacle_from_config(item, index) for index, item in enumerate(items)]
    validate_unique_obstacle_ids(msg.obstacles)
    return msg


def effective_obstacle_ids(obstacles) -> list[str]:
    return [(obstacle.id.strip() or f"obstacle_{index}") for index, obstacle in enumerate(obstacles)]


def validate_unique_obstacle_ids(obstacles) -> None:
    seen = set()
    duplicates = set()
    for obstacle_id in effective_obstacle_ids(obstacles):
        if obstacle_id in seen:
            duplicates.add(obstacle_id)
        seen.add(obstacle_id)
    if duplicates:
        duplicate_list = ", ".join(sorted(duplicates))
        raise ValueError(f"duplicate dynamic obstacle id(s): {duplicate_list}")


class DynamicObstacleSimNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_sim_node")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("profile_file", "")
        self.declare_parameter("robot_base_frames", [""])
        self.declare_parameter("robot_collision_radius", 0.574)
        self.declare_parameter("robot_overlap_safety_margin", 0.0)
        self.declare_parameter("reject_robot_overlaps", True)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.robot_base_frames = [
            str(frame)
            for frame in self.get_parameter("robot_base_frames").value
            if str(frame).strip()
        ]
        self.robot_collision_radius = max(0.0, float(self.get_parameter("robot_collision_radius").value))
        self.robot_overlap_safety_margin = max(0.0, float(self.get_parameter("robot_overlap_safety_margin").value))
        self.reject_robot_overlaps = bool(self.get_parameter("reject_robot_overlaps").value)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        publish_rate = max(0.1, float(self.get_parameter("publish_rate").value))
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(DynamicObstacleArray, "/dynamic_obstacles", qos)
        self.marker_publisher = self.create_publisher(MarkerArray, "/dynamic_obstacle_markers", qos)
        self.service = self.create_service(
            SetDynamicObstacles,
            "~/set_dynamic_obstacles",
            self._handle_set_dynamic_obstacles,
        )
        self.obstacles = DynamicObstacleArray()
        self.obstacles.header.frame_id = self.world_frame
        self._last_time = self.get_clock().now()
        self._last_marker_count = 0
        self._needs_publish = True

        profile_file = str(self.get_parameter("profile_file").value).strip()
        if profile_file:
            self._load_profile_file(profile_file)

        self.timer = self.create_timer(1.0 / publish_rate, self._publish_callback)
        self.get_logger().info(
            "dynamic_obstacle_sim_node publishing "
            f"{len(self.obstacles.obstacles)} obstacles on /dynamic_obstacles "
            "and /dynamic_obstacle_markers"
        )
        if self.reject_robot_overlaps:
            self.get_logger().info(
                "dynamic obstacle robot-overlap rejection enabled: "
                f"robot_base_frames={self.robot_base_frames}, "
                f"robot_collision_radius={self.robot_collision_radius:.3f} m, "
                f"safety_margin={self.robot_overlap_safety_margin:.3f} m"
            )

    def _load_profile_file(self, profile_file: str) -> None:
        path = Path(profile_file).expanduser()
        loaded = json.loads(path.read_text())
        self.obstacles = obstacle_array_from_profile(loaded, self.world_frame)

    def _handle_set_dynamic_obstacles(self, request, response):
        frame_id = request.obstacles.header.frame_id or self.world_frame
        if frame_id != self.world_frame:
            response.success = False
            response.message = f"expected frame_id '{self.world_frame}', got '{frame_id}'"
            self.get_logger().warn(f"rejected dynamic obstacle update: {response.message}")
            return response

        try:
            validate_unique_obstacle_ids(request.obstacles.obstacles)
            if request.obstacles.obstacles:
                self._validate_obstacles_do_not_overlap_robots(request.obstacles.obstacles)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warn(f"rejected dynamic obstacle update: {response.message}")
            return response

        self.obstacles = DynamicObstacleArray()
        self.obstacles.header.frame_id = self.world_frame
        self.obstacles.obstacles = list(request.obstacles.obstacles)
        self._last_time = self.get_clock().now()
        self._needs_publish = True
        self._publish_snapshot()
        ids = effective_obstacle_ids(self.obstacles.obstacles)
        response.success = True
        response.message = f"configured {len(self.obstacles.obstacles)} dynamic obstacles"
        self.get_logger().info(f"received dynamic obstacle update: count={len(ids)}, ids={ids}")
        return response

    def _validate_obstacles_do_not_overlap_robots(self, obstacles) -> None:
        if not self.reject_robot_overlaps:
            return
        if not self.robot_base_frames:
            raise ValueError("robot-overlap validation is enabled but robot_base_frames is empty")

        robot_positions = self._current_robot_positions()
        for obstacle_index, obstacle in enumerate(obstacles):
            obstacle_id = obstacle.id.strip() or f"obstacle_{obstacle_index}"
            obstacle_center = (
                float(obstacle.pose.position.x),
                float(obstacle.pose.position.y),
                float(obstacle.pose.position.z),
            )
            obstacle_radius = self._obstacle_bounding_radius(obstacle)
            for base_frame, robot_position in robot_positions.items():
                clearance = (
                    self._distance(obstacle_center, robot_position)
                    - obstacle_radius
                    - self.robot_collision_radius
                    - self.robot_overlap_safety_margin
                )
                if clearance < 0.0:
                    raise ValueError(
                        f"obstacle '{obstacle_id}' overlaps {base_frame} by {-clearance:.3f} m "
                        f"(obstacle_radius={obstacle_radius:.3f} m, "
                        f"robot_radius={self.robot_collision_radius:.3f} m)"
                    )

    def _current_robot_positions(self) -> dict[str, tuple[float, float, float]]:
        positions: dict[str, tuple[float, float, float]] = {}
        for base_frame in self.robot_base_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    base_frame,
                    Time(),
                    timeout=Duration(nanoseconds=50_000_000),
                )
            except Exception as exc:
                raise ValueError(
                    f"robot-overlap validation unavailable: missing transform "
                    f"{self.world_frame} <- {base_frame}: {exc}"
                ) from exc
            translation = transform.transform.translation
            positions[base_frame] = (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )
        return positions

    @staticmethod
    def _obstacle_bounding_radius(obstacle: DynamicObstacle) -> float:
        geometry = int(obstacle.collision_type)
        dimensions = [max(0.0, float(value)) for value in obstacle.collision_dimensions]
        if geometry == DynamicObstacle.GEOMETRY_SPHERE:
            return dimensions[0] if dimensions else 0.0
        if geometry == DynamicObstacle.GEOMETRY_BOX and len(dimensions) >= 3:
            return 0.5 * math.sqrt(dimensions[0] ** 2 + dimensions[1] ** 2 + dimensions[2] ** 2)
        if geometry == DynamicObstacle.GEOMETRY_CYLINDER and len(dimensions) >= 2:
            return math.sqrt(dimensions[0] ** 2 + (0.5 * dimensions[1]) ** 2)
        if geometry == DynamicObstacle.GEOMETRY_MESH:
            if len(dimensions) == 1:
                return dimensions[0]
            if len(dimensions) >= 3:
                return 0.5 * math.sqrt(dimensions[0] ** 2 + dimensions[1] ** 2 + dimensions[2] ** 2)
        return 0.0

    @staticmethod
    def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def _publish_callback(self) -> None:
        if not self._needs_publish and not self._obstacles_have_motion():
            return

        now = self.get_clock().now()
        dt = max(0.0, (now - self._last_time).nanoseconds * 1.0e-9)
        self._last_time = now
        if dt > 0.0:
            self._integrate_obstacles(dt)
        self._publish_snapshot()

    def _obstacles_have_motion(self) -> bool:
        for obstacle in self.obstacles.obstacles:
            twist = obstacle.twist
            if (
                abs(twist.linear.x) > 1.0e-9
                or abs(twist.linear.y) > 1.0e-9
                or abs(twist.linear.z) > 1.0e-9
                or abs(twist.angular.x) > 1.0e-9
                or abs(twist.angular.y) > 1.0e-9
                or abs(twist.angular.z) > 1.0e-9
            ):
                return True
        return False

    def _integrate_obstacles(self, dt: float) -> None:
        for obstacle in self.obstacles.obstacles:
            obstacle.pose.position.x += obstacle.twist.linear.x * dt
            obstacle.pose.position.y += obstacle.twist.linear.y * dt
            obstacle.pose.position.z += obstacle.twist.linear.z * dt
            angular = obstacle.twist.angular
            angular_velocity = (float(angular.x), float(angular.y), float(angular.z))
            if any(abs(value) > 1.0e-9 for value in angular_velocity):
                self._integrate_angular_velocity(obstacle.pose, angular_velocity, dt)

    @staticmethod
    def _integrate_angular_velocity(pose: Pose, angular_velocity: tuple[float, float, float], dt: float) -> None:
        wx, wy, wz = angular_velocity
        speed = math.sqrt(wx * wx + wy * wy + wz * wz)
        if speed < 1.0e-9:
            return
        half_angle = 0.5 * speed * dt
        scale = math.sin(half_angle) / speed
        dx = wx * scale
        dy = wy * scale
        dz = wz * scale
        dw = math.cos(half_angle)
        q = pose.orientation
        x = dw * q.x + dx * q.w + dy * q.z - dz * q.y
        y = dw * q.y - dx * q.z + dy * q.w + dz * q.x
        z = dw * q.z + dx * q.y - dy * q.x + dz * q.w
        w = dw * q.w - dx * q.x - dy * q.y - dz * q.z
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-9:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            return
        pose.orientation.x = x / norm
        pose.orientation.y = y / norm
        pose.orientation.z = z / norm
        pose.orientation.w = w / norm

    def _publish_snapshot(self) -> None:
        self.obstacles.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.obstacles)
        self._publish_markers()
        self._needs_publish = False

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        for index, obstacle in enumerate(self.obstacles.obstacles):
            marker = Marker()
            marker.header = self.obstacles.header
            marker.ns = "dynamic_obstacles"
            marker.id = index
            marker.action = Marker.ADD
            marker.pose = obstacle.pose
            marker.type = self._marker_type(obstacle)
            self._set_marker_scale(marker, obstacle)
            marker.color = obstacle.color
            if marker.color.a <= 0.0:
                marker.color.r = 0.95
                marker.color.g = 0.42
                marker.color.b = 0.12
                marker.color.a = 0.75
            if marker.type == Marker.MESH_RESOURCE:
                marker.mesh_resource = obstacle.visual_mesh_resource
                marker.mesh_use_embedded_materials = False
            marker_array.markers.append(marker)

        current_count = len(self.obstacles.obstacles)
        for stale_index in range(current_count, self._last_marker_count):
            marker = Marker()
            marker.header = self.obstacles.header
            marker.ns = "dynamic_obstacles"
            marker.id = stale_index
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self._last_marker_count = current_count
        self.marker_publisher.publish(marker_array)

    @staticmethod
    def _marker_type(obstacle: DynamicObstacle) -> int:
        geometry = int(obstacle.visual_type or obstacle.collision_type)
        if geometry == DynamicObstacle.GEOMETRY_BOX:
            return Marker.CUBE
        if geometry == DynamicObstacle.GEOMETRY_CYLINDER:
            return Marker.CYLINDER
        if geometry == DynamicObstacle.GEOMETRY_MESH and obstacle.visual_mesh_resource:
            return Marker.MESH_RESOURCE
        return Marker.SPHERE

    @staticmethod
    def _set_marker_scale(marker: Marker, obstacle: DynamicObstacle) -> None:
        geometry = int(obstacle.visual_type or obstacle.collision_type)
        dimensions = list(obstacle.visual_dimensions or obstacle.collision_dimensions)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        if geometry == DynamicObstacle.GEOMETRY_SPHERE:
            radius = float(dimensions[0]) if dimensions else 0.25
            marker.scale.x = marker.scale.y = marker.scale.z = max(0.001, 2.0 * radius)
        elif geometry == DynamicObstacle.GEOMETRY_BOX and len(dimensions) >= 3:
            marker.scale.x = max(0.001, float(dimensions[0]))
            marker.scale.y = max(0.001, float(dimensions[1]))
            marker.scale.z = max(0.001, float(dimensions[2]))
        elif geometry == DynamicObstacle.GEOMETRY_CYLINDER and len(dimensions) >= 2:
            radius = float(dimensions[0])
            marker.scale.x = marker.scale.y = max(0.001, 2.0 * radius)
            marker.scale.z = max(0.001, float(dimensions[1]))
        elif geometry == DynamicObstacle.GEOMETRY_MESH and len(dimensions) >= 3:
            marker.scale.x = max(0.001, float(dimensions[0]))
            marker.scale.y = max(0.001, float(dimensions[1]))
            marker.scale.z = max(0.001, float(dimensions[2]))


def main():
    rclpy.init()
    node = DynamicObstacleSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

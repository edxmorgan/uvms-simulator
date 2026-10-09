#!/usr/bin/env python3
# Copyright (C) 2026 Edward Morgan

import json
import math
from pathlib import Path

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from ros2_control_blue_reach_5.msg import DynamicObstacle, DynamicObstacleArray
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from bringup.collision_geometry import geometry_from_obstacle


from bringup.obstacle_description import (
    obstacle_array_from_profile, visual_geometry,
)


class DynamicObstacleTransport(Node):
    """Shared ROS transport and markers; motion is owned by the simlab source host."""
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
        self.obstacles = DynamicObstacleArray()
        self.obstacles.header.frame_id = self.world_frame
        self._last_time = self.get_clock().now()
        self._last_marker_count = 0

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
            return geometry_from_obstacle(obstacle)[1]
        return 0.0

    @staticmethod
    def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def _publish_callback(self):
        raise NotImplementedError('The selected behavior source owns motion')

    def _publish_snapshot(self) -> None:
        self.obstacles.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.obstacles)
        self._publish_markers()

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
                marker.mesh_resource = self._marker_mesh_resource(obstacle)
                marker.mesh_use_embedded_materials = not marker.mesh_resource.lower().endswith('.stl')
                if marker.mesh_use_embedded_materials:
                    # Zero RGBA tells RViz to use the asset materials without tint.
                    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 0.
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
    def _marker_mesh_resource(obstacle):
        resource = visual_geometry(obstacle)[2]
        return Path(resource).as_uri() if resource.startswith('/') else resource

    @staticmethod
    def _marker_type(obstacle: DynamicObstacle) -> int:
        geometry, _, resource = visual_geometry(obstacle)
        if geometry == DynamicObstacle.GEOMETRY_BOX:
            return Marker.CUBE
        if geometry == DynamicObstacle.GEOMETRY_CYLINDER:
            return Marker.CYLINDER
        if geometry == DynamicObstacle.GEOMETRY_MESH and resource:
            return Marker.MESH_RESOURCE
        if geometry == DynamicObstacle.GEOMETRY_MESH:
            raise ValueError('mesh visualization requires a mesh resource')
        return Marker.SPHERE

    @staticmethod
    def _set_marker_scale(marker: Marker, obstacle: DynamicObstacle) -> None:
        geometry, dimensions, _ = visual_geometry(obstacle)
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

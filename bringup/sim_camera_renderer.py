#!/usr/bin/env python3
# Copyright (C) 2026 Edward Morgan

import copy
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d
import pyvista as pv
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

WATER_SURFACE_Z = 0.0


def camera_topic_base(prefix):
    return "/" + prefix.rstrip("_") + "/camera"


def parse_vec3(text, default):
    if not text:
        return np.array(default, dtype=float)
    values = [float(value) for value in text.split()]
    return np.array(values[:3] if len(values) >= 3 else default, dtype=float)


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def quaternion_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1.0e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1 - xx - yy],
        ],
        dtype=float,
    )


def transform_to_matrix(transform: TransformStamped):
    t = transform.transform.translation
    q = transform.transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix((q.x, q.y, q.z, q.w))
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


def resolve_mesh_uri(uri):
    path = uri
    if path.startswith("file://"):
        path = path[len("file://") :]
    if path.startswith("package://"):
        rest = path[len("package://") :]
        package, _, suffix = rest.partition("/")
        return os.path.join(get_package_share_directory(package), suffix)
    marker = "$(find "
    start = path.find(marker)
    if start >= 0:
        package_start = start + len(marker)
        package_end = path.find(")", package_start)
        if package_end >= 0:
            package = path[package_start:package_end]
            return path[:start] + get_package_share_directory(package) + path[package_end + 1 :]
    return path


def read_triangle_mesh(path, max_triangles):
    loaded = trimesh.load(path, force="scene")
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.dump(concatenate=True)
    if loaded.is_empty:
        return o3d.geometry.TriangleMesh()

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(loaded.vertices, dtype=float))
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(loaded.faces, dtype=np.int32))
    if getattr(loaded, "vertex_normals", None) is not None and len(loaded.vertex_normals) == len(loaded.vertices):
        mesh.vertex_normals = o3d.utility.Vector3dVector(np.array(loaded.vertex_normals, dtype=float, copy=True))
    else:
        mesh.compute_vertex_normals()
    if max_triangles > 0 and len(mesh.triangles) > max_triangles:
        mesh = mesh.simplify_quadric_decimation(max_triangles)
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()
        mesh.compute_vertex_normals()
    return mesh


def open3d_to_polydata(mesh):
    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.triangles, dtype=np.int64)
    if faces.size == 0:
        return pv.PolyData(vertices)
    vtk_faces = np.column_stack([np.full((faces.shape[0], 1), 3, dtype=np.int64), faces]).reshape(-1)
    return pv.PolyData(vertices, vtk_faces)


class SimCameraRendererNode(Node):
    def __init__(self):
        super().__init__("sim_camera_renderer_node")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robots_prefix", ["robot_1_"])
        self.declare_parameter("camera_prefixes", [""])
        self.declare_parameter("render_prefixes", [""])
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("width", 480)
        self.declare_parameter("height", 360)
        self.declare_parameter("render_rate", 5.0)
        self.declare_parameter("renderer_backend", "pyvista")
        self.declare_parameter("sim_camera_max_mesh_triangles", 12000)
        self.declare_parameter("sim_camera_render_all_cameras", True)
        self.declare_parameter("sim_camera_underwater_effect", True)
        self.declare_parameter("sim_camera_underwater_haze", 0.35)
        self.declare_parameter("sim_camera_underwater_tint", 0.55)
        self.declare_parameter("horizontal_fov_deg", 75.0)
        self.declare_parameter("selected_prefix", "")
        self.declare_parameter("publish_selected_output", True)

        self.robot_description = self.get_parameter("robot_description").value
        self.robot_prefixes = list(self.get_parameter("robots_prefix").value)
        self.camera_prefixes = [p for p in self.get_parameter("camera_prefixes").value if p]
        if not self.camera_prefixes:
            self.camera_prefixes = [p for p in self.robot_prefixes if p != "robot_real_"]
        self.render_prefixes = [p for p in self.get_parameter("render_prefixes").value if p]
        if not self.render_prefixes:
            self.render_prefixes = list(self.camera_prefixes)
        self.world_frame = self.get_parameter("world_frame").value
        self.width = max(1, int(self.get_parameter("width").value))
        self.height = max(1, int(self.get_parameter("height").value))
        self.render_rate = max(0.1, float(self.get_parameter("render_rate").value))
        self.renderer_backend = str(self.get_parameter("renderer_backend").value).strip().lower()
        if self.renderer_backend not in {"pyvista", "open3d"}:
            raise RuntimeError("renderer_backend must be one of: pyvista, open3d.")
        self.max_mesh_triangles = max(0, int(self.get_parameter("sim_camera_max_mesh_triangles").value))
        self.render_all_cameras = bool(self.get_parameter("sim_camera_render_all_cameras").value)
        self.underwater_effect = bool(self.get_parameter("sim_camera_underwater_effect").value)
        self.underwater_haze = float(np.clip(float(self.get_parameter("sim_camera_underwater_haze").value), 0.0, 1.0))
        self.underwater_tint = float(np.clip(float(self.get_parameter("sim_camera_underwater_tint").value), 0.0, 1.0))
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.selected_prefix = self.get_parameter("selected_prefix").value
        if not self.selected_prefix and self.camera_prefixes:
            self.selected_prefix = self.camera_prefixes[0]
        self.publish_selected_output = bool(self.get_parameter("publish_selected_output").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_qos = QoSProfile(depth=1)
        self.info_qos = QoSProfile(depth=1)
        self.image_publishers = {}
        self.info_publishers = {}
        for prefix in self.render_prefixes:
            base = camera_topic_base(prefix)
            self.image_publishers[prefix] = self.create_publisher(Image, base + "/image_raw", self.image_qos)
            self.info_publishers[prefix] = self.create_publisher(CameraInfo, base + "/camera_info", self.info_qos)
        self.selected_image_publisher = self.create_publisher(Image, "/alpha/image_raw", self.image_qos)
        self.selected_info_publisher = self.create_publisher(CameraInfo, "/alpha/camera_info", self.info_qos)
        self.passthrough_subscriptions = []
        self._create_passthrough_subscribers()

        self.mesh_frames = {}
        self.mesh_cache = {}
        self.scene_bounds_min = None
        self.scene_bounds_max = None
        self.pyvista_plotter = None
        self.open3d_renderer = None
        if self.renderer_backend == "pyvista":
            self._setup_pyvista_backend()
        else:
            self._setup_open3d_backend()

        self.fx = 0.5 * self.width / math.tan(0.5 * math.radians(self.horizontal_fov_deg))
        self.fy = self.fx
        self.cx = 0.5 * (self.width - 1)
        self.cy = 0.5 * (self.height - 1)
        self._underwater_row = np.linspace(0.0, 1.0, self.height, dtype=np.float32).reshape(self.height, 1, 1)
        self._underwater_color = np.array([0.05, 0.36, 0.48], dtype=np.float32).reshape(1, 1, 3)
        self._underwater_tint_color = np.array([0.58, 0.92, 1.0], dtype=np.float32).reshape(1, 1, 3)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.timer = self.create_timer(1.0 / self.render_rate, self.render_all)
        self.get_logger().info(
            f"{self.renderer_backend} sim_camera_renderer_node render cameras={len(self.render_prefixes)} "
            f"selectable cameras={len(self.camera_prefixes)} scene_meshes={len(self.mesh_frames)}"
        )

    def destroy_node(self):
        if self.pyvista_plotter is not None:
            try:
                self.pyvista_plotter.close()
            except Exception:
                pass
        super().destroy_node()

    def _setup_open3d_backend(self):
        self.open3d_renderer = o3d.visualization.rendering.OffscreenRenderer(self.width, self.height)
        self.scene = self.open3d_renderer.scene
        self.scene.set_background([0.02, 0.13, 0.20, 1.0])
        self.material = o3d.visualization.rendering.MaterialRecord()
        self.material.shader = "defaultLit"
        self.material.base_color = [0.65, 0.65, 0.62, 1.0]
        self._load_open3d_urdf_visual_meshes()
        self._add_open3d_water_surface()

    def _setup_pyvista_backend(self):
        self.pyvista_plotter = pv.Plotter(off_screen=True, window_size=(self.width, self.height))
        self.pyvista_plotter.set_background("#022032", top="#001018")
        self.pyvista_plotter.enable_lightkit()
        self._load_pyvista_urdf_visual_meshes()
        self._add_pyvista_water_surface()

    def _create_passthrough_subscribers(self):
        for prefix in self.camera_prefixes:
            if prefix in self.render_prefixes:
                continue
            base = camera_topic_base(prefix)
            self.passthrough_subscriptions.append(
                self.create_subscription(Image, base + "/image_raw", lambda msg, p=prefix: self._passthrough_image(p, msg), self.image_qos)
            )
            self.passthrough_subscriptions.append(
                self.create_subscription(CameraInfo, base + "/camera_info", lambda msg, p=prefix: self._passthrough_info(p, msg), self.info_qos)
            )

    def _passthrough_image(self, prefix, msg):
        if self.publish_selected_output and prefix == self.selected_prefix:
            self.selected_image_publisher.publish(msg)

    def _passthrough_info(self, prefix, msg):
        if self.publish_selected_output and prefix == self.selected_prefix:
            self.selected_info_publisher.publish(msg)

    def _on_set_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "selected_prefix":
                selected = str(parameter.value)
                if selected not in self.camera_prefixes:
                    return SetParametersResult(successful=False, reason="selected_prefix is not selectable")
                self.selected_prefix = selected
                self.get_logger().info(f"Selected camera feed set to {selected}")
            elif parameter.name == "publish_selected_output":
                self.publish_selected_output = bool(parameter.value)
            elif parameter.name == "sim_camera_render_all_cameras":
                self.render_all_cameras = bool(parameter.value)
            elif parameter.name == "sim_camera_underwater_effect":
                self.underwater_effect = bool(parameter.value)
            elif parameter.name == "sim_camera_underwater_haze":
                self.underwater_haze = float(np.clip(float(parameter.value), 0.0, 1.0))
            elif parameter.name == "sim_camera_underwater_tint":
                self.underwater_tint = float(np.clip(float(parameter.value), 0.0, 1.0))
        return SetParametersResult(successful=True)

    def _visual_mesh_entries(self):
        if not self.robot_description:
            self.get_logger().warning("robot_description is empty; no scene meshes loaded.")
            return
        root = ET.fromstring(self.robot_description)
        for link in root.findall("link"):
            frame = link.attrib.get("name", "")
            if not frame:
                continue
            for visual in link.findall("visual"):
                mesh_element = visual.find("./geometry/mesh")
                if mesh_element is None or "filename" not in mesh_element.attrib:
                    continue
                mesh_path = resolve_mesh_uri(mesh_element.attrib["filename"])
                scale = parse_vec3(mesh_element.attrib.get("scale"), [1.0, 1.0, 1.0])
                origin = visual.find("origin")
                xyz = parse_vec3(origin.attrib.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
                rpy = parse_vec3(origin.attrib.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0])
                yield frame, mesh_path, scale, xyz, rpy

    def _record_scene_bounds(self, points):
        if points.size == 0:
            return
        bounds_min = np.min(points, axis=0)
        bounds_max = np.max(points, axis=0)
        if self.scene_bounds_min is None:
            self.scene_bounds_min = bounds_min
            self.scene_bounds_max = bounds_max
        else:
            self.scene_bounds_min = np.minimum(self.scene_bounds_min, bounds_min)
            self.scene_bounds_max = np.maximum(self.scene_bounds_max, bounds_max)

    def _water_surface_size(self):
        if self.scene_bounds_min is None or self.scene_bounds_max is None:
            return 60.0, 40.0
        extent = np.maximum(self.scene_bounds_max[:2] - self.scene_bounds_min[:2], [60.0, 40.0])
        return float(extent[0] * 1.2), float(extent[1] * 1.2)

    def _add_open3d_water_surface(self):
        width, height = self._water_surface_size()
        vertices = np.array(
            [
                [-0.5 * width, -0.5 * height, WATER_SURFACE_Z],
                [0.5 * width, -0.5 * height, WATER_SURFACE_Z],
                [0.5 * width, 0.5 * height, WATER_SURFACE_Z],
                [-0.5 * width, 0.5 * height, WATER_SURFACE_Z],
            ],
            dtype=float,
        )
        triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        material.base_color = [0.08, 0.42, 0.58, 0.28]
        self.scene.add_geometry("water_surface", mesh, material)

    def _add_pyvista_water_surface(self):
        width, height = self._water_surface_size()
        water = pv.Plane(
            center=(0.0, 0.0, WATER_SURFACE_Z),
            direction=(0.0, 0.0, 1.0),
            i_size=width,
            j_size=height,
        )
        self.pyvista_plotter.add_mesh(
            water,
            color="#2f9bbd",
            opacity=0.22,
            ambient=0.65,
            diffuse=0.25,
            specular=0.1,
        )

    def _load_open3d_urdf_visual_meshes(self):
        visual_index = 0
        loaded_triangles = 0
        for frame, mesh_path, scale, xyz, rpy in self._visual_mesh_entries():
            cached_mesh = self.mesh_cache.get(mesh_path)
            if cached_mesh is None:
                cached_mesh = read_triangle_mesh(mesh_path, self.max_mesh_triangles)
                self.mesh_cache[mesh_path] = cached_mesh
            mesh = copy.deepcopy(cached_mesh)
            if mesh.is_empty():
                self.get_logger().warning(f"Skipping empty mesh {mesh_path} on {frame}")
                continue
            vertices = np.asarray(mesh.vertices)
            vertices *= scale.reshape(1, 3)
            vertices[:] = vertices @ rpy_matrix(rpy).T + xyz.reshape(1, 3)
            self._record_scene_bounds(vertices)
            mesh.vertices = o3d.utility.Vector3dVector(vertices)
            mesh.compute_vertex_normals()
            name = f"visual_{visual_index}_{frame}"
            visual_index += 1
            self.scene.add_geometry(name, mesh, self.material)
            self.mesh_frames[name] = frame
            loaded_triangles += len(mesh.triangles)
        self.get_logger().info(
            f"Loaded Open3D scene meshes={visual_index} triangles={loaded_triangles}"
        )

    def _load_pyvista_urdf_visual_meshes(self):
        visual_index = 0
        loaded_triangles = 0
        for frame, mesh_path, scale, xyz, rpy in self._visual_mesh_entries():
            cached_poly = self.mesh_cache.get(mesh_path)
            if cached_poly is None:
                cached_poly = open3d_to_polydata(read_triangle_mesh(mesh_path, self.max_mesh_triangles))
                self.mesh_cache[mesh_path] = cached_poly
            poly = cached_poly.copy(deep=True)
            if poly.n_points == 0 or poly.n_cells == 0:
                self.get_logger().warning(f"Skipping empty mesh {mesh_path} on {frame}")
                continue
            points = np.asarray(poly.points)
            points *= scale.reshape(1, 3)
            points[:] = points @ rpy_matrix(rpy).T + xyz.reshape(1, 3)
            self._record_scene_bounds(points)
            poly.points = points
            actor = self.pyvista_plotter.add_mesh(
                poly,
                color="#c9b56d",
                smooth_shading=True,
                ambient=0.4,
                diffuse=0.7,
                specular=0.12,
            )
            name = f"visual_{visual_index}_{frame}"
            visual_index += 1
            self.mesh_frames[name] = (frame, actor)
            loaded_triangles += poly.n_cells
        self.get_logger().info(
            f"Loaded PyVista scene meshes={visual_index} triangles={loaded_triangles}"
        )

    def _lookup_matrix(self, frame):
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, frame, Time(), timeout=Duration(seconds=0.0))
            return transform_to_matrix(transform)
        except TransformException:
            return None

    def _update_scene_transforms(self):
        for name, frame_or_actor in self.mesh_frames.items():
            frame = frame_or_actor[0] if self.renderer_backend == "pyvista" else frame_or_actor
            matrix = self._lookup_matrix(frame)
            if matrix is None:
                continue
            if self.renderer_backend == "pyvista":
                frame, actor = frame_or_actor
                actor.user_matrix = matrix
            else:
                self.scene.set_geometry_transform(name, matrix)

    def camera_info(self, prefix, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = prefix + "camera_link"
        msg.width = self.width
        msg.height = self.height
        msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        msg.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        return msg

    def _apply_underwater_effect(self, rgb, camera_z):
        image = np.ascontiguousarray(rgb[:, :, :3], dtype=np.uint8)
        if not self.underwater_effect or camera_z >= WATER_SURFACE_Z:
            return image

        submergence = min(1.0, max(0.0, WATER_SURFACE_Z - camera_z) / 2.0)
        normalized = image.astype(np.float32) / 255.0

        tint_strength = self.underwater_tint * submergence
        haze_strength = self.underwater_haze * submergence * (0.65 + 0.35 * self._underwater_row)
        normalized = normalized * ((1.0 - tint_strength) + tint_strength * self._underwater_tint_color)
        normalized = normalized * (1.0 - haze_strength) + self._underwater_color * haze_strength
        normalized = (normalized - 0.5) * 0.78 + 0.5
        return np.ascontiguousarray((np.clip(normalized, 0.0, 1.0) * 255.0).astype(np.uint8))

    def _has_subscribers(self, publisher):
        return publisher.get_subscription_count() > 0

    def _should_render_prefix(self, prefix):
        if self.render_all_cameras:
            return True
        if prefix != self.selected_prefix:
            return False
        return (
            self._has_subscribers(self.image_publishers[prefix])
            or self._has_subscribers(self.info_publishers[prefix])
            or (self.publish_selected_output and self._has_subscribers(self.selected_image_publisher))
            or (self.publish_selected_output and self._has_subscribers(self.selected_info_publisher))
        )

    def render_one(self, prefix, stamp):
        camera_pose = self._lookup_matrix(prefix + "camera_link")
        if camera_pose is None:
            return None
        eye = camera_pose[:3, 3]
        forward = camera_pose[:3, :3] @ np.array([0.0, 0.0, 1.0])
        up = camera_pose[:3, :3] @ np.array([0.0, -1.0, 0.0])
        if self.renderer_backend == "pyvista":
            vertical_fov_deg = math.degrees(
                2.0 * math.atan(math.tan(0.5 * math.radians(self.horizontal_fov_deg)) * self.height / self.width)
            )
            self.pyvista_plotter.camera.position = tuple(eye.tolist())
            self.pyvista_plotter.camera.focal_point = tuple((eye + forward).tolist())
            self.pyvista_plotter.camera.up = tuple(up.tolist())
            self.pyvista_plotter.camera.view_angle = vertical_fov_deg
            self.pyvista_plotter.camera.clipping_range = (0.03, 100.0)
            self.pyvista_plotter.camera.Modified()
            self.pyvista_plotter.renderer.ResetCameraClippingRange()
            self.pyvista_plotter.render()
            rgb = np.asarray(self.pyvista_plotter.screenshot(return_img=True))[:, :, :3]
        else:
            self.open3d_renderer.setup_camera(self.horizontal_fov_deg, eye + forward, eye, up)
            rgb = np.asarray(self.open3d_renderer.render_to_image())[:, :, :3]
        rgb = self._apply_underwater_effect(rgb, eye[2])
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = prefix + "camera_link"
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = self.width * 3
        msg.data = rgb.tobytes()
        return msg

    def render_all(self):
        stamp = self.get_clock().now().to_msg()
        prefixes = self.render_prefixes if self.render_all_cameras else [self.selected_prefix]
        active_prefixes = [prefix for prefix in prefixes if prefix in self.render_prefixes and self._should_render_prefix(prefix)]
        if not active_prefixes:
            return
        self._update_scene_transforms()
        for prefix in active_prefixes:
            image = self.render_one(prefix, stamp)
            if image is None:
                continue
            info = self.camera_info(prefix, stamp)
            if self.render_all_cameras or self._has_subscribers(self.image_publishers[prefix]):
                self.image_publishers[prefix].publish(image)
            if self.render_all_cameras or self._has_subscribers(self.info_publishers[prefix]):
                self.info_publishers[prefix].publish(info)
            if self.publish_selected_output and prefix == self.selected_prefix:
                if self._has_subscribers(self.selected_image_publisher):
                    self.selected_image_publisher.publish(image)
                if self._has_subscribers(self.selected_info_publisher):
                    self.selected_info_publisher.publish(info)


def main():
    rclpy.init()
    node = SimCameraRendererNode()
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

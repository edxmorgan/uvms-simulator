"""Validated collision assets shared by obstacle producers and consumers."""
from functools import lru_cache
from pathlib import Path

import fcl
import numpy as np
import trimesh
from ament_index_python.packages import get_package_share_directory


def resolve_collision_mesh(resource):
    if resource.startswith("package://"):
        package, separator, relative = resource[10:].partition("/")
        if not separator:
            raise ValueError("package mesh URI must include a file path")
        try:
            path = Path(get_package_share_directory(package)) / relative
        except Exception as exc:
            raise ValueError(f"cannot resolve collision mesh package: {package}") from exc
    else:
        path = Path(resource.removeprefix("file://"))
        if not resource or not path.is_absolute():
            raise ValueError("collision mesh requires a package://, file:// or absolute path")
    if not path.is_file():
        raise ValueError(f"collision mesh does not exist: {resource}")
    return path


@lru_cache(maxsize=128)
def collision_geometry(kind, dimensions, resource="", scale=()):
    """Return (FCL geometry, enclosing radius about the obstacle origin).

    Geometry is immutable and cached; each consumer owns its collision objects.
    Meshes use triangle-surface semantics, just like the bathymetry.
    """
    dimensions = tuple(dimensions)
    required = {1: 1, 2: 3, 3: 2}
    if kind in required:
        if len(dimensions) != required[kind] or not np.all(np.isfinite(dimensions)) or min(dimensions) <= 0:
            raise ValueError(f"invalid dimensions for collision geometry {kind}: {dimensions}")
        if kind == 1:
            return fcl.Sphere(dimensions[0]), dimensions[0]
        if kind == 2:
            return fcl.Box(*dimensions), float(np.linalg.norm(dimensions) / 2)
        return fcl.Cylinder(*dimensions), float(np.hypot(dimensions[0], dimensions[1] / 2))
    if kind != 4:
        raise ValueError(f"unsupported collision geometry: {kind}")
    scale = tuple(scale)
    if len(scale) != 3 or not np.all(np.isfinite(scale)) or min(scale) <= 0:
        raise ValueError("collision_mesh_scale must contain three finite positive values")
    try:
        mesh = collision_surface_mesh(kind, dimensions, resource, scale)
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        faces = np.asarray(mesh.faces, dtype=np.int32)
        if not len(vertices) or not len(faces) or not np.all(np.isfinite(vertices)):
            raise ValueError("mesh is empty or non-finite")
        geometry = fcl.BVHModel()
        geometry.beginModel(len(vertices), len(faces))
        geometry.addSubModel(vertices, faces)
        geometry.endModel()
        return geometry, float(np.max(np.linalg.norm(vertices, axis=1)))
    except Exception as exc:
        raise ValueError(f"could not load collision mesh {resource}: {exc}") from exc


@lru_cache(maxsize=128)
def collision_surface_mesh(kind, dimensions, resource="", scale=()):
    """Shared local-frame surface for FCL meshes and voxel visualization.

    Returned meshes are cached: callers must copy before modifying them.
    Primitive surfaces are tessellations for display only; FCL uses exact shapes.
    """
    if kind != 4:
        collision_geometry(kind, dimensions, resource, scale)  # common validation
        if kind == 1:
            return trimesh.creation.icosphere(subdivisions=3, radius=dimensions[0])
        if kind == 2:
            return trimesh.creation.box(extents=dimensions)
        return trimesh.creation.cylinder(radius=dimensions[0], height=dimensions[1], sections=64)
    if len(scale) != 3 or not np.all(np.isfinite(scale)) or min(scale) <= 0:
        raise ValueError("collision_mesh_scale must contain three finite positive values")
    loaded = trimesh.load(str(resolve_collision_mesh(resource)), force="scene")
    mesh = loaded.to_geometry() if isinstance(loaded, trimesh.Scene) else loaded
    mesh = mesh.copy()
    mesh.apply_scale(scale)
    if not len(mesh.vertices) or not len(mesh.faces) or not np.all(np.isfinite(mesh.vertices)):
        raise ValueError("mesh is empty or non-finite")
    return mesh


def geometry_from_obstacle(obstacle):
    p, q = obstacle.pose.position, obstacle.pose.orientation
    v, w = obstacle.twist.linear, obstacle.twist.angular
    if not np.all(np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z])):
        raise ValueError(f"non-finite state for obstacle {obstacle.id}")
    return collision_geometry(
        int(obstacle.collision_type), tuple(obstacle.collision_dimensions),
        obstacle.collision_mesh_resource, tuple(obstacle.collision_mesh_scale),
    )


def distance_with_nearest(first, second):
    """Return distance and world-frame endpoints in caller order.

    FCL's BVH/shape dispatch orders results mesh-first even when arguments are
    reversed. Some builds additionally return BVH/sphere endpoints in each
    object's local frame. Validate the sphere endpoint before accepting it.
    """
    a, b = first, second
    swapped = not isinstance(a['geom'], fcl.BVHModel) and isinstance(b['geom'], fcl.BVHModel)
    if swapped:
        a, b = b, a
    result = fcl.DistanceResult()
    distance = float(fcl.distance(a['fcl_obj'], b['fcl_obj'],
                     fcl.DistanceRequest(enable_nearest_points=True), result))
    points = np.asarray(result.nearest_points, dtype=float)
    if points.shape != (2, 3) or not np.isfinite(distance) or not np.all(np.isfinite(points)):
        raise ValueError('FCL returned invalid nearest points')
    if distance > 0 and isinstance(a['geom'], fcl.BVHModel) and isinstance(b['geom'], fcl.Sphere):
        center = b['fcl_obj'].getTranslation()
        radius = b['geom'].radius

        def valid(candidate):
            delta = candidate[0] - center
            length = np.linalg.norm(delta)
            if length <= radius:
                return False
            expected = center + radius * delta / length
            return (np.allclose(candidate[1], expected, atol=1e-5, rtol=1e-6)
                    and np.isclose(np.linalg.norm(candidate[0] - candidate[1]), distance, atol=1e-5))

        if not valid(points):
            points = np.array([body['fcl_obj'].getRotation() @ point + body['fcl_obj'].getTranslation()
                               for body, point in zip((a, b), points)])
            if not valid(points):
                raise ValueError('FCL mesh/sphere nearest points failed world-frame validation')
    if swapped:
        points = points[::-1]
    return distance, points[0], points[1]

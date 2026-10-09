"""Shared obstacle parsing, visual defaults, and commit-time validation."""
import copy
import math
from ros2_control_blue_reach_5.msg import DynamicObstacle, DynamicObstacleArray
from bringup.collision_geometry import collision_geometry, geometry_from_obstacle


def geometry_type(value) -> int:
    if isinstance(value, int):
        if value not in range(5):
            raise ValueError(f"unknown obstacle geometry type '{value}'")
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
    if len(values) != size or not all(math.isfinite(v) for v in values):
        raise ValueError(f"obstacle '{key}' must contain exactly {size} values")
    return values


def obstacle_from_config(config: dict, index: int) -> DynamicObstacle:
    if not isinstance(config, dict):
        raise ValueError(f"obstacles[{index}] must be an object")
    obstacle = DynamicObstacle()
    obstacle.id = str(config.get("id", f"obstacle_{index}"))
    obstacle.collision_type = geometry_type(config.get("type", config.get("collision_type", "sphere")))
    obstacle.collision_dimensions = [float(v) for v in list(config.get("dimensions", config.get("collision_dimensions", [])))]
    if not obstacle.collision_dimensions and obstacle.collision_type not in (DynamicObstacle.GEOMETRY_MESH, DynamicObstacle.GEOMETRY_NONE):
        raise ValueError(f"obstacle '{obstacle.id}' is missing dimensions")
    obstacle.visual_type = geometry_type(config.get("visual_type", obstacle.collision_type))
    if "visual_dimensions" in config and not config["visual_dimensions"]:
        raise ValueError("explicit visual_dimensions must not be empty")
    if "visual_mesh_resource" in config and obstacle.visual_type == DynamicObstacle.GEOMETRY_MESH and not config["visual_mesh_resource"]:
        raise ValueError("explicit visual_mesh_resource must not be empty")
    obstacle.visual_dimensions = [float(v) for v in list(config.get("visual_dimensions", obstacle.collision_dimensions))]
    obstacle.visual_mesh_resource = str(config.get("visual_mesh_resource", ""))
    obstacle.collision_mesh_resource = str(config.get("collision_mesh_resource", ""))
    obstacle.collision_mesh_scale = vector(config, "collision_mesh_scale", 3, [1.0, 1.0, 1.0])
    if obstacle.visual_type == DynamicObstacle.GEOMETRY_MESH:
        obstacle.visual_mesh_resource = str(config.get("visual_mesh_resource", obstacle.collision_mesh_resource))
        obstacle.visual_dimensions = list(config.get("visual_dimensions", obstacle.collision_mesh_scale))

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
    return normalize_obstacle(obstacle)


def obstacle_array_from_profile(profile: dict, default_frame_id: str) -> DynamicObstacleArray:
    obstacles_config = profile.get("obstacles", profile)
    frame_id = str(profile.get("frame_id", default_frame_id) or default_frame_id)
    items = obstacles_config
    if isinstance(obstacles_config, dict):
        frame_id = str(obstacles_config.get("frame_id", frame_id) or frame_id)
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



def visual_geometry(obstacle):
    """Resolve absent visual fields from collision geometry without changing input."""
    kind = geometry_type(int(obstacle.visual_type or obstacle.collision_type))
    if kind == DynamicObstacle.GEOMETRY_MESH:
        resource = obstacle.visual_mesh_resource or obstacle.collision_mesh_resource
        dimensions = tuple(obstacle.visual_dimensions or obstacle.collision_mesh_scale)
    else:
        resource = ""
        dimensions = tuple(obstacle.visual_dimensions or obstacle.collision_dimensions)
    return kind, dimensions, resource


def normalize_obstacle(obstacle):
    """Validate an independent message before committing it to the world."""
    result = copy.deepcopy(obstacle)
    geometry_type(int(result.collision_type))
    if result.collision_type != DynamicObstacle.GEOMETRY_NONE:
        geometry_from_obstacle(result)
    kind, dimensions, resource = visual_geometry(result)
    if kind != DynamicObstacle.GEOMETRY_NONE:
        collision_geometry(kind, dimensions if kind != DynamicObstacle.GEOMETRY_MESH else (),
                           resource, dimensions if kind == DynamicObstacle.GEOMETRY_MESH else ())
    result.visual_type = kind
    result.visual_dimensions = list(dimensions)
    result.visual_mesh_resource = resource
    p, q = result.pose.position, result.pose.orientation
    linear, angular = result.twist.linear, result.twist.angular
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("obstacle pose and twist must be finite")
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        # Preserve the existing convention for an unspecified ROS quaternion.
        q.x = q.y = q.z = 0.0
        q.w = 1.0
    else:
        q.x, q.y, q.z, q.w = (v / norm for v in (q.x, q.y, q.z, q.w))
    return result

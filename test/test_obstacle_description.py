"""Shared obstacle defaults and renderer geometry regressions (no GPU needed)."""
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh

from bringup.obstacle_description import normalize_obstacle, obstacle_from_config
from bringup.sim_camera_renderer import SimCameraRendererNode
from ros2_control_blue_reach_5.msg import DynamicObstacle


def test_collision_only_visual_defaults_do_not_mutate_input():
    obstacle = DynamicObstacle(collision_type=DynamicObstacle.GEOMETRY_BOX,
                               collision_dimensions=[1.0, 2.0, 3.0])
    obstacle.pose.orientation.w = 0.0
    result = normalize_obstacle(obstacle)
    assert result.visual_type == DynamicObstacle.GEOMETRY_BOX
    assert list(result.visual_dimensions) == [1.0, 2.0, 3.0]
    assert result.pose.orientation.w == 1.0
    assert not obstacle.visual_dimensions
    assert obstacle.pose.orientation.w == 0.0


@pytest.mark.parametrize('override', [
    {'dimensions': [-1.0]},
    {'dimensions': [float('nan')]},
    {'visual_type': 'box', 'visual_dimensions': [1.0]},
    {'visual_dimensions': []},
    {'visual_type': 'mesh', 'visual_mesh_resource': ''},
    {'visual_type': 'mesh', 'visual_mesh_resource': '/missing/obstacle.obj'},
    {'position': [float('inf'), 0.0, 0.0]},
])
def test_invalid_geometry_or_pose_rejected(override):
    with pytest.raises((ValueError, FileNotFoundError)):
        obstacle_from_config({'type': 'sphere', 'dimensions': [1.0], **override}, 0)


@pytest.mark.parametrize('backend', ['open3d', 'pyvista'])
def test_collision_only_mesh_uses_nonuniform_scale_in_camera(tmp_path, backend):
    path = tmp_path / 'box.obj'
    trimesh.creation.box(extents=[1.0, 2.0, 3.0]).export(path)
    obstacle = DynamicObstacle(collision_type=DynamicObstacle.GEOMETRY_MESH,
                               collision_mesh_resource=str(path),
                               collision_mesh_scale=[2.0, 3.0, 4.0])
    node = SimpleNamespace(
        max_mesh_triangles=0,
        _dynamic_obstacle_type_and_dimensions=SimCameraRendererNode._dynamic_obstacle_type_and_dimensions,
    )
    build = getattr(SimCameraRendererNode, f'_{backend}_dynamic_obstacle_mesh')
    for _ in range(2):
        mesh = build(node, obstacle)
        points = np.asarray(mesh.vertices if backend == 'open3d' else mesh.points)
        np.testing.assert_allclose(np.ptp(points, axis=0), [2.0, 6.0, 12.0])
    assert not obstacle.visual_dimensions

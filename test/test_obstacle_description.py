"""Shared obstacle defaults and renderer geometry regressions (no GPU needed)."""
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh

from bringup.obstacle_description import normalize_obstacle, obstacle_from_config
from bringup.sim_camera_renderer import SimCameraRendererNode
from ros2_control_blue_reach_5.msg import DynamicObstacle


def test_omitted_config_visual_defaults_and_explicit_none():
    config = {'id': 'box', 'type': 'box', 'dimensions': [1.,2.,3.]}
    defaulted = obstacle_from_config(config, 0)
    assert defaulted.visual_type == DynamicObstacle.GEOMETRY_BOX
    assert list(defaulted.visual_dimensions) == [1.,2.,3.]
    hidden = obstacle_from_config({**config, 'visual_type': 'none', 'visual_dimensions': []}, 0)
    assert hidden.visual_type == DynamicObstacle.GEOMETRY_NONE
    assert not hidden.visual_dimensions


def test_zero_quaternion_is_rejected_without_mutating_input():
    obstacle = obstacle_from_config({'id':'box', 'type':'sphere', 'dimensions':[1.]}, 0)
    obstacle.pose.orientation.w = 0.
    with pytest.raises(ValueError, match='nonzero norm'):
        normalize_obstacle(obstacle)
    assert obstacle.pose.orientation.w == 0.


def test_ids_are_canonical_before_reserved_and_duplicate_checks():
    from bringup.obstacle_description import obstacle_array_from_profile
    obstacle = obstacle_from_config({'id':' box ', 'type':'sphere', 'dimensions':[1.]}, 0)
    assert obstacle.id == 'box'
    with pytest.raises(ValueError, match='generated'):
        obstacle_from_config({'id':' robot/robot_1_', 'type':'sphere', 'dimensions':[1.]}, 0)
    with pytest.raises(ValueError, match='duplicate'):
        obstacle_array_from_profile({'obstacles':[
            {'id':name,'type':'sphere','dimensions':[1.]} for name in ['box',' box ']]}, 'world')


def test_startup_profile_rejects_foreign_frame_without_relabelling(tmp_path):
    import json
    from bringup.dynamic_obstacle_sim import DynamicObstacleTransport
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps({'frame_id':'offset_map','obstacles':[]}))
    original = object()
    node = SimpleNamespace(world_frame='world', obstacles=original)
    with pytest.raises(ValueError, match='does not match'):
        DynamicObstacleTransport._load_profile_file(node, str(path))
    assert node.obstacles is original


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
def test_config_mesh_visual_defaults_use_nonuniform_scale_in_camera(tmp_path, backend):
    path = tmp_path / 'box.obj'
    trimesh.creation.box(extents=[1.0, 2.0, 3.0]).export(path)
    obstacle = obstacle_from_config({'id':'mesh', 'type':'mesh',
        'collision_mesh_resource':str(path), 'collision_mesh_scale':[2.,3.,4.]}, 0)
    node = SimpleNamespace(
        max_mesh_triangles=0,
        _dynamic_obstacle_type_and_dimensions=SimCameraRendererNode._dynamic_obstacle_type_and_dimensions,
    )
    build = getattr(SimCameraRendererNode, f'_{backend}_dynamic_obstacle_mesh')
    for _ in range(2):
        mesh = build(node, obstacle)
        points = np.asarray(mesh.vertices if backend == 'open3d' else mesh.points)
        np.testing.assert_allclose(np.ptp(points, axis=0), [2.0, 6.0, 12.0])
    assert list(obstacle.visual_dimensions) == [2.,3.,4.]


def test_unknown_source_is_rejected_at_edit_admission():
    obstacle = obstacle_from_config({'id':'box', 'type':'sphere', 'dimensions':[1.]}, 0)
    obstacle.source = 'unknown'
    with pytest.raises(ValueError, match='unknown editable obstacle source'):
        normalize_obstacle(obstacle)

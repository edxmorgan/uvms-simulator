"""Static scene integration: shared URDF, frames, geometry and provenance."""
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import trimesh
import xacro

from bringup.environment_scene import resolve_environment_scene

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / 'description/scenes/pipeline'
BUNDLES = sorted(p.parent for p in (ROOT / 'description/scenes').glob('*/world.xacro'))
spec = importlib.util.spec_from_file_location('scene_import', ROOT / 'tools/import_stonefish_scene.py')
IMPORTER = importlib.util.module_from_spec(spec)
spec.loader.exec_module(IMPORTER)


def expand(scene=''):
    doc = xacro.parse(f'''<robot name="scene" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{ROOT / 'description/xacro/world.xacro'}"/>
      <xacro:sim_world world_frame="world" scene_file="{scene}"/>
    </robot>''')
    xacro.process_doc(doc)
    return ET.fromstring(doc.toxml())


def test_scene_resolution():
    assert resolve_environment_scene('default', ROOT / 'description') == ''
    assert resolve_environment_scene('pipeline', ROOT / 'description') == str(BUNDLE / 'world.xacro')
    assert resolve_environment_scene(BUNDLE, '') == str(BUNDLE / 'world.xacro')
    for invalid in ('../escape', 'missing', 'some/path'):
        with pytest.raises(ValueError):
            resolve_environment_scene(invalid, ROOT / 'description')


def test_default_environment_is_preserved():
    root = expand()
    assert {l.get('name') for l in root.findall('link')} == {
        'world', 'world_bottom', 'bathymetry_shipwreck', 'bathymetry_seafloor', 'base_link'}
    assert float(root.find("joint[@name='world_bottom_joint']/origin").get('xyz').split()[2]) == -8


def transform(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    m[:3, 3] = xyz
    return m


@pytest.mark.parametrize('bundle', BUNDLES, ids=lambda p:p.name)
def test_imported_geometry_alignment_and_frame_conversion(bundle):
    root = expand(bundle / 'world.xacro')
    text, _ = IMPORTER.expand_object_includes((bundle/'source.scn.template').read_text(),
        lambda p:(bundle/'source_includes'/p).read_text())
    source = ET.fromstring(text)
    meta = json.loads((bundle / 'manifest.json').read_text())
    flip = np.diag([1., -1., -1., 1.])
    included = [obj for i, obj in enumerate(source.findall('static'))
                if i not in meta.get('excluded_static_indices', [])]
    for record, upstream in zip(meta['objects'], included, strict=True):
        link = root.find(f"link[@name='{record['name']}']")
        joint = next(j for j in root.findall('joint') if j.find('child').get('link') == record['name'])
        origin = joint.find('origin')
        world = transform(np.fromstring(origin.get('xyz'), sep=' '), np.fromstring(origin.get('rpy'), sep=' '))
        original = upstream.find('world_transform')
        expected = flip @ transform(np.fromstring(original.get('xyz'), sep=' '), np.fromstring(original.get('rpy'), sep=' '))
        expected[2, 3] += meta.get('vertical_offset_m', 0.) + record.get('placement_offset_z_m', 0.)
        expected[:3, 3] += record.get('placement_offset_xyz_m', [0., 0., 0.])
        np.testing.assert_allclose(world, expected, atol=1e-12)
        meshes = []
        for kind in ('visual', 'collision'):
            node = link.find(kind)
            path = Path(node.find('geometry/mesh').get('filename').removeprefix('file://'))
            assert path.is_file()
            mesh = trimesh.load(path, force='mesh', process=False)
            assert len(mesh.faces) and np.isfinite(mesh.vertices).all()
            o = node.find('origin')
            mesh.apply_transform(world @ transform(np.fromstring(o.get('xyz'), sep=' '), np.fromstring(o.get('rpy'), sep=' ')))
            meshes.append(mesh)
        # Material/UV conversion must not change even one collision triangle.
        np.testing.assert_allclose(meshes[0].triangles, meshes[1].triangles, atol=1e-12)
    assert not root.findall("link[@name='bathymetry_shipwreck']")


@pytest.mark.parametrize('bundle', BUNDLES, ids=lambda p:p.name)
def test_bundle_hashes(bundle):
    meta = json.loads((bundle / 'manifest.json').read_text())
    for name, expected in meta['artifact_sha256'].items():
        assert hashlib.sha256((bundle / name).read_bytes()).hexdigest() == expected


def test_converter_rejects_moving_scene(tmp_path):
    spec = importlib.util.spec_from_file_location('scene_import', ROOT / 'tools/import_stonefish_scene.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(ValueError, match='Moving objects'):
        module.convert('<scenario><dynamic/></scenario>', lambda _: b'', tmp_path, {})


def test_object_include_arguments_and_missing_support():
    text = '<scenario><include file="$(find stonefish_bluerov2)/scenarios/objects/t.scn"><arg name="position" value="1 2 3"/></include></scenario>'
    resolved, sources = IMPORTER.expand_object_includes(text, lambda _: '<scenario><static name="test"><world_transform xyz="$(arg position)"/></static></scenario>')
    assert ET.fromstring(resolved).find('static/world_transform').get('xyz') == '1 2 3'
    assert 'objects/t.scn' in sources
    with pytest.raises(ValueError, match='Missing include argument'):
        IMPORTER.expand_object_includes(text, lambda _: '<scenario><static name="$(arg missing)"/></scenario>')


def test_automatic_crop_accounts_for_scale_and_origin():
    root = ET.fromstring('''<scenario>
      <static name="Seabed"><physical><mesh filename="bed.obj" scale="2"/></physical></static>
      <static name="Object"><physical><mesh filename="object.obj"/></physical>
        <world_transform xyz="20 30 0"/></static>
    </scenario>''')
    mesh = b'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n'
    bounds = IMPORTER.automatic_seabed_crop(root, lambda _:mesh, 10)
    np.testing.assert_allclose(bounds, [-5,15.5,-5,20.5])
    with pytest.raises(ValueError, match='margin'):
        IMPORTER.automatic_seabed_crop(root, lambda _:mesh, -1)


def test_automatic_crop_includes_world_grid_after_scale_and_translation():
    root = ET.fromstring('<scenario><static name="Seabed"><physical><mesh filename="bed.obj" scale="2"/></physical><world_transform xyz="10 20 5"/></static></scenario>')
    bounds = IMPORTER.automatic_seabed_crop(root, lambda _: b'', 10, grid_half_extent=25)
    np.testing.assert_allclose(bounds, [-17.5, 7.5, -22.5, 2.5])
    with pytest.raises(ValueError, match='Grid'):
        IMPORTER.automatic_seabed_crop(root, lambda _: b'', 10, grid_half_extent=-1)


def test_depth_limit_preserves_relative_geometry_and_shallow_scenes():
    macro = ET.fromstring('<macro><joint><parent link="${world_frame}"/><origin xyz="3 4 -20.5"/></joint><joint><parent link="${world_frame}"/><origin xyz="0 0 -25"/></joint></macro>')
    objects = [{'world_xyz_nwu': [3., 4., -20.5]}]
    floor, offset = IMPORTER.limit_seabed_depth(macro, objects, -25., 8.)
    assert (floor, offset) == (-8., 17.)
    assert objects[0]['world_xyz_nwu'] == [3., 4., -3.5]
    assert [joint.find('origin').get('xyz') for joint in macro] == ['3 4 -3.5', '0 0 -8']
    before = ET.tostring(macro)
    assert IMPORTER.limit_seabed_depth(macro, objects, -3.5, 8.) == (-3.5, 0.)
    assert ET.tostring(macro) == before
    for invalid in (-1., 0., float('nan')):
        with pytest.raises(ValueError):
            IMPORTER.limit_seabed_depth(macro, objects, -25., invalid)


@pytest.mark.parametrize('bundle', BUNDLES, ids=lambda p:p.name)
def test_bundled_seabed_depth_matches_shared_floor(bundle):
    meta = json.loads((bundle/'manifest.json').read_text())
    root = expand(bundle/'world.xacro')
    floor = float(root.find("joint[@name='world_bottom_joint']/origin").get('xyz').split()[2])
    assert floor == meta['floor_z'] and -8. <= floor < 0.


@pytest.mark.parametrize('scene,indices', [('turbines', (1, 2)), ('lake', (15, 16, 17))])
def test_corrected_assemblies_touch_local_terrain(scene, indices):
    bundle = ROOT / 'description/scenes' / scene
    meta = json.loads((bundle / 'manifest.json').read_text())

    def world_mesh(record):
        shape = record['collision']
        mesh = trimesh.load(bundle / 'assets' / shape['asset'], force='mesh', process=False)
        mesh.apply_scale(shape['scale'])
        mesh.apply_transform(transform(record['world_xyz_nwu'], record['world_rpy_nwu']) @
                             transform(shape['xyz'], shape['rpy']))
        return mesh

    terrain = world_mesh(meta['objects'][0]).triangles
    vertices = np.concatenate([world_mesh(meta['objects'][i]).triangles.reshape(-1, 3)
                               for i in indices])
    bottom = np.unique(vertices[vertices[:, 2] < vertices[:, 2].min() + 1e-5], axis=0)
    # Vertical triangle intersection without an optional spatial-index dependency.
    a = terrain[:, 0, :2]
    b, c = terrain[:, 1, :2] - a, terrain[:, 2, :2] - a
    det = b[:, 0] * c[:, 1] - b[:, 1] * c[:, 0]
    valid = np.abs(det) > 1e-14
    gaps = []
    for point in bottom:
        v = point[:2] - a
        u = np.full(len(terrain), np.nan)
        w = u.copy()
        u[valid] = (v[valid, 0] * c[valid, 1] - v[valid, 1] * c[valid, 0]) / det[valid]
        w[valid] = (b[valid, 0] * v[valid, 1] - b[valid, 1] * v[valid, 0]) / det[valid]
        inside = (u >= -1e-9) & (w >= -1e-9) & (u + w <= 1 + 1e-9)
        assert inside.any(), 'Assembly extends beyond terrain'
        heights = (terrain[:, 0, 2] + u * (terrain[:, 1, 2] - terrain[:, 0, 2]) +
                   w * (terrain[:, 2, 2] - terrain[:, 0, 2]))
        gaps.append(point[2] - heights[inside].max())
    assert min(gaps) == pytest.approx(0., abs=1e-6)
    assert min(gaps) >= -1e-6
    offsets = [meta['objects'][i]['placement_offset_z_m'] for i in indices]
    assert len(set(offsets)) == 1, 'Connected parts must move together'


def test_tank_retains_support_without_above_water_plane():
    root = expand(ROOT / 'description/scenes/pipe_tank/world.xacro')
    assert root.find("link[@name='bathymetry_wavetank_floor_2']") is None
    assert root.find("link[@name='bathymetry_wavetank_1']") is not None


@pytest.mark.parametrize('bundle', BUNDLES, ids=lambda p:p.name)
def test_collision_loading_discards_materials_without_changing_triangles(bundle):
    from bringup.collision_geometry import collision_surface_mesh
    meta = json.loads((bundle / 'manifest.json').read_text())
    shape = meta['objects'][1]['visual']
    path = str(bundle / 'assets' / shape['asset'])
    original = trimesh.load(path, force='scene').to_geometry()
    original.apply_scale(shape['scale'])
    surface = collision_surface_mesh(4, (), path, (shape['scale'],) * 3)
    np.testing.assert_allclose(surface.triangles, original.triangles, atol=1e-12)
    assert getattr(surface.visual, 'material', None) is None


def test_modern_ship_leaves_reset_origins_clear():
    bundle = ROOT / 'description/scenes/modern_ship'
    meta = json.loads((bundle / 'manifest.json').read_text())
    ship = next(o for o in meta['objects'] if o['name'] == 'bathymetry_ship_iron_1')
    spec = ship['collision']
    mesh = trimesh.load(bundle / 'assets' / spec['asset'], force='mesh', process=False)
    mesh.apply_scale(spec['scale'])
    mesh.apply_transform(transform(ship['world_xyz_nwu'], ship['world_rpy_nwu'])
                         @ transform(spec['xyz'], spec['rpy']))
    # Lateral separation guarantees a clear descent column, not just one pose.
    for origin_y in (0., -.4820780440595218):
        assert mesh.bounds[0, 1] - origin_y > .574 + .01 + 1.
    assert np.all(mesh.bounds[:, :2] >= -25.)
    assert np.all(mesh.bounds[:, :2] <= 25.)

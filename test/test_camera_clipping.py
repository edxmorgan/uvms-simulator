"""Exercise the onboard camera projection without starting ROS or a GPU renderer."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time

from bringup.sim_camera_renderer import SimCameraRendererNode, LAMP_INTENSITY


class CameraClippingTests(unittest.TestCase):
    def test_close_arm_geometry_is_not_clipped_at_ten_centimeters(self):
        renderer = Mock()
        renderer.render_to_image.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        node = SimpleNamespace(
            _lookup_matrix=lambda frame: np.eye(4),
            renderer_backend="open3d",
            open3d_renderer=renderer,
            horizontal_fov_deg=75.0,
            _apply_underwater_effect=lambda image, depth: image,
            width=3,
            height=2,
        )
        stamp = Time(sec=1)
        message = SimCameraRendererNode.render_one(node, "robot_1_", stamp)
        args, kwargs = renderer.setup_camera.call_args
        self.assertEqual(kwargs, {"near_clip": 0.01})
        self.assertEqual(args[0], 75.0)
        np.testing.assert_array_equal(args[1], [0, 0, 1])
        np.testing.assert_array_equal(args[2], [0, 0, 0])
        np.testing.assert_array_equal(args[3], [0, -1, 0])
        self.assertEqual(message.header.frame_id, "robot_1_camera_link")
        self.assertEqual(message.header.stamp, stamp)
        self.assertEqual((message.width, message.height, len(message.data)), (3, 2, 18))

    def test_physical_camera_buffer_uses_plant_pose_and_fixed_geometry(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer
        physical = SimpleNamespace(tf_buffer=Buffer(), world_frame='world',
            robot_link_owners={'robot_1_camera_link':'robot_1_'}, physical_feedback_timeout=.5,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)))
        body = TransformStamped()
        body.header.frame_id, body.child_frame_id = 'world', 'robot_1_base_link'
        body.header.stamp = Time(sec=1)
        body.transform.translation.x = 2.
        body.transform.rotation.w = 1.
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'robot_1_base_link', 'robot_1_camera_link'
        mount.transform.translation.x = .21
        mount.transform.rotation.w = 1.
        SimCameraRendererNode._static_tf_callback(physical, TFMessage(transforms=[mount]))
        # Fixed geometry alone must not fabricate a camera pose.
        self.assertIsNone(SimCameraRendererNode._lookup_matrix(physical, 'robot_1_camera_link'))
        SimCameraRendererNode._physical_tf_callback(physical, TFMessage(transforms=[body]))
        pose = SimCameraRendererNode._lookup_matrix(physical, 'robot_1_camera_link')
        np.testing.assert_allclose(pose[:3, 3], [2.21, 0., 0.])
        # An unrelated operational estimate lives in a different TF buffer.
        operational = Buffer()
        body.transform.translation.x = 20.
        operational.set_transform(body, 'ekf')
        np.testing.assert_allclose(
            SimCameraRendererNode._lookup_matrix(physical, 'robot_1_camera_link')[:3, 3],
            [2.21, 0., 0.])


if __name__ == "__main__":
    unittest.main()


def test_scene_mesh_loading_defers_render_until_scene_is_complete():
    import pyvista as pv
    node = SimpleNamespace(
        _visual_mesh_entries=lambda: iter([('bathymetry_test', '/mesh', np.ones(3), np.zeros(3), np.zeros(3), None)]),
        mesh_cache={'/mesh': pv.Cube()}, _record_scene_bounds=lambda _: None,
        pyvista_plotter=Mock(), mesh_frames={}, get_logger=lambda: Mock())
    SimCameraRendererNode._load_pyvista_urdf_visual_meshes(node)
    assert node.pyvista_plotter.add_mesh.call_args.kwargs['render'] is False
    assert len(node.mesh_frames) == 1


def test_robot_lamps_follow_each_emitter_and_missing_tf_disables_light():
    import pyvista as pv
    description = '<robot name="scene">'
    for prefix in ['robot_real_', 'robot_2_']:
        description += f'''<link name="{prefix}base_link"/><link name="{prefix}lumen_1_link"/>
          <joint name="{prefix}lamp" type="fixed"><parent link="{prefix}base_link"/>
          <child link="{prefix}lumen_1_link"/></joint>'''
    description += '</robot>'
    poses = {'robot_real_lumen_1_link': np.eye(4), 'robot_2_lumen_1_link':np.eye(4)}
    poses['robot_2_lumen_1_link'][:3,3] = [2.,3.,4.]
    node = SimpleNamespace(robot_description=description, renderer_backend='pyvista',
        pyvista_plotter=Mock(), _lookup_matrix=poses.get)
    SimCameraRendererNode._setup_robot_lights(node)
    assert len(node.rov_lights) == 2
    assert all(light.intensity == 0. for light, _ in node.rov_lights)
    SimCameraRendererNode._update_rov_lights(node)
    for light, frame in node.rov_lights:
        assert light.on and light.intensity == LAMP_INTENSITY
        np.testing.assert_allclose(light.position, poses[frame][:3,3])
        np.testing.assert_allclose(light.focal_point, poses[frame][:3,3] + [1,0,0])
    poses.clear()
    SimCameraRendererNode._update_rov_lights(node)
    assert all(light.intensity == 0. for light, _ in node.rov_lights)
    SimCameraRendererNode._setup_robot_lights(node)
    assert len(node.rov_lights) == 2
    assert node.pyvista_plotter.remove_all_lights.call_count == 2


def test_scene_light_illuminates_robot_mesh_in_rendered_image():
    import pyvista as pv
    plotter = pv.Plotter(off_screen=True, window_size=(96, 96), lighting='none')
    try:
        plotter.set_background('black')
        node = SimpleNamespace(
            _visual_mesh_entries=lambda: iter([('robot_2_base_link', '/mesh', np.ones(3), np.array([2.,0.,0.]), np.zeros(3), None)]),
            mesh_cache={'/mesh':pv.Cube()}, _record_scene_bounds=lambda _:None,
            pyvista_plotter=plotter, mesh_frames={}, get_logger=lambda:Mock(),
            renderer_backend='pyvista',
            robot_description='''<robot name="scene"><link name="robot_real_base_link"/>
              <link name="robot_real_lumen_1_link"/><joint name="lamp" type="fixed">
              <parent link="robot_real_base_link"/><child link="robot_real_lumen_1_link"/></joint></robot>''',
            _lookup_matrix=lambda _:np.eye(4))
        SimCameraRendererNode._load_pyvista_urdf_visual_meshes(node)
        SimCameraRendererNode._setup_robot_lights(node)
        plotter.camera_position = [(-2.,-3.,2.), (2.,0.,0.), (0.,0.,1.)]
        next(iter(node.mesh_frames.values()))[1].SetVisibility(True)
        dark = plotter.screenshot(return_img=True)
        SimCameraRendererNode._update_rov_lights(node)
        plotter.render()
        lit = plotter.screenshot(return_img=True)
        assert float(lit.mean()) > float(dark.mean()) + 1.
    finally:
        plotter.close()


def test_camera_does_not_render_robot_proxy_over_urdf_visuals():
    from types import SimpleNamespace as NS
    robot = NS(id='robot/robot_1_', source='robot')
    scripted = NS(id='rock', source='scripted', pose=NS())
    old_proxy = {'signature': ('proxy',)}
    node = NS(world_frame='world', obstacle_snapshot_timeout=1.,
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000)), dynamic_obstacle_entries={'dynamic_obstacle_robot/robot_1_':old_proxy},
              _dynamic_obstacle_signature=lambda obstacle: ('rock',),
              _add_dynamic_obstacle=lambda *args: {'signature': ('rock',)},
              _remove_dynamic_obstacle=Mock())
    from unittest.mock import patch
    with patch('bringup.sim_camera_renderer.pose_to_matrix', return_value=np.eye(4)):
        SimCameraRendererNode._dynamic_obstacles_callback(node, NS(
            header=NS(frame_id='world',stamp=NS(sec=1,nanosec=0)), obstacles=[robot, scripted]))
    assert set(node.dynamic_obstacle_entries) == {'dynamic_obstacle_rock'}
    node._remove_dynamic_obstacle.assert_called_once_with('dynamic_obstacle_robot/robot_1_',old_proxy)


def test_urdf_visual_material_resolves_named_and_inline_colors():
    node = SimpleNamespace(robot_description='''<robot name="scene">
      <material name="black"><color rgba="0.3 0.3 0.3 1"/></material>
      <link name="robot_2_arm"><visual><geometry><mesh filename="/arm.stl"/></geometry>
      <material name="black"/></visual></link>
      <link name="robot_2_tool"><visual><geometry><mesh filename="/tool.stl"/></geometry>
      <material name="blue"><color rgba="0 0 1 0.5"/></material></visual></link>
      </robot>''')
    entries = list(SimCameraRendererNode._visual_mesh_entries(node))
    np.testing.assert_allclose(entries[0][-1], [.3,.3,.3,1.])
    np.testing.assert_allclose(entries[1][-1], [0.,0.,1.,.5])


def test_robot_material_and_distance_response_in_rendered_image():
    import pyvista as pv
    plotter = pv.Plotter(off_screen=True, window_size=(96,96), lighting='none')
    try:
        plotter.set_background('black')
        node = SimpleNamespace(
            _visual_mesh_entries=lambda: iter([('robot_2_arm', '/mesh', np.ones(3), np.zeros(3), np.zeros(3), np.array([.1,.3,.8,1.]))]),
            mesh_cache={'/mesh':pv.Cube()}, _record_scene_bounds=lambda _:None,
            pyvista_plotter=plotter, mesh_frames={}, get_logger=lambda:Mock(),
            renderer_backend='pyvista',
            robot_description='''<robot name="scene"><link name="robot_1_base_link"/>
              <link name="robot_1_lumen_1_link"/><joint name="lamp" type="fixed">
              <parent link="robot_1_base_link"/><child link="robot_1_lumen_1_link"/></joint></robot>''')
        SimCameraRendererNode._load_pyvista_urdf_visual_meshes(node)
        SimCameraRendererNode._setup_robot_lights(node)
        next(iter(node.mesh_frames.values()))[1].SetVisibility(True)
        plotter.camera_position=[(-3.,0.,0.),(0.,0.,0.),(0.,0.,1.)]
        light = node.rov_lights[0][0]
        light.focal_point=(0.,0.,0.); light.intensity=LAMP_INTENSITY
        images=[]
        for distance in (1.,5.):
            light.position=(-distance,0.,0.)
            plotter.render()
            images.append(plotter.screenshot(return_img=True)[40:56,40:56,:3].astype(float))
        near,far=images
        assert near.mean() > far.mean() > 0.
        assert near[:,:,2].mean() > near[:,:,0].mean()*1.5
        print('material render near/far means:',near.mean(),far.mean(),'near peak:',near.max())
    finally:
        plotter.close()


def test_constant_pbr_color_with_uvs_is_expanded_per_vertex():
    import trimesh
    from unittest.mock import patch
    from bringup.sim_camera_renderer import read_triangle_mesh
    mesh = trimesh.creation.box()
    mesh.visual = trimesh.visual.TextureVisuals(
        uv=np.zeros((len(mesh.vertices), 2)),
        material=trimesh.visual.material.PBRMaterial(baseColorFactor=[32,64,128,255]))
    with patch('bringup.sim_camera_renderer.trimesh.load', return_value=trimesh.Scene(mesh)):
        loaded = read_triangle_mesh('constant-pbr', 0, preserve_materials=True)
    colors = np.asarray(loaded.vertex_colors)
    assert colors.shape == (len(loaded.vertices),3)
    np.testing.assert_allclose(colors, np.tile(np.array([32,64,128])/255.,(len(colors),1)))


def test_stale_or_future_physical_camera_pose_is_unavailable():
    from tf2_ros import Buffer
    from geometry_msgs.msg import TransformStamped
    physical = SimpleNamespace(tf_buffer=Buffer(), world_frame='world',
        robot_link_owners={'robot_1_camera_link':'robot_1_'}, physical_feedback_timeout=.5,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_000_000_000)))
    pose=TransformStamped()
    pose.header.frame_id='world'; pose.child_frame_id='robot_1_camera_link'
    pose.transform.rotation.w=1.
    for stamp, valid in ((1.,False),(1.75,True),(3.,False)):
        pose.header.stamp=Time(sec=int(stamp),nanosec=int((stamp%1)*1e9))
        physical.tf_buffer.set_transform(pose,'plant')
        assert (SimCameraRendererNode._lookup_matrix(physical,'robot_1_camera_link') is not None) == valid
    unavailable = SimpleNamespace(_lookup_matrix=lambda _:None)
    assert SimCameraRendererNode.render_one(unavailable,'robot_1_',Time(sec=2)) is None


def test_missing_geometry_and_expired_obstacles_are_hidden_then_recover():
    from unittest.mock import patch
    actor,obstacle_actor=Mock(),Mock()
    transforms={}
    node=SimpleNamespace(renderer_backend='pyvista',mesh_frames={'robot':('robot_1_arm',actor)},
        _lookup_matrix=transforms.get, _update_rov_lights=Mock(),
        obstacle_snapshot_received=10.,obstacle_snapshot_age=.25,obstacle_snapshot_timeout=1.,
        dynamic_obstacle_entries={'rock':{'actor':obstacle_actor,'matrix':np.eye(4)}})
    with patch('bringup.sim_camera_renderer.time.monotonic',return_value=11.):
        SimCameraRendererNode._update_scene_transforms(node)
    actor.SetVisibility.assert_called_with(False)
    obstacle_actor.SetVisibility.assert_called_with(False)
    transforms['robot_1_arm']=np.eye(4); node.obstacle_snapshot_received=11.
    with patch('bringup.sim_camera_renderer.time.monotonic',return_value=11.1):
        SimCameraRendererNode._update_scene_transforms(node)
    actor.SetVisibility.assert_called_with(True)
    obstacle_actor.SetVisibility.assert_called_with(True)


def test_colored_mesh_obstacle_respects_configured_alpha():
    import pyvista as pv
    import open3d as o3d
    mesh=pv.Cube(); mesh.point_data['asset_rgb']=np.ones((mesh.n_points,3))
    node=SimpleNamespace(_pyvista_dynamic_obstacle_mesh=lambda _:mesh,
        _dynamic_obstacle_color=lambda _:(.2,.3,.4,.25),pyvista_plotter=Mock())
    SimCameraRendererNode._add_pyvista_dynamic_obstacle(node,object())
    assert node.pyvista_plotter.add_mesh.call_args.kwargs['opacity']==.25
    other=o3d.geometry.TriangleMesh.create_box();other.paint_uniform_color([.2,.3,.4])
    node._open3d_dynamic_obstacle_mesh=lambda _:other
    _,material=SimCameraRendererNode._open3d_dynamic_obstacle_mesh_and_material(node,object())
    assert material.base_color[3]==.25
    assert material.shader=='defaultLitTransparency'


def test_zero_obstacle_alpha_remains_transparent():
    from ros2_control_blue_reach_5.msg import DynamicObstacle
    obstacle=DynamicObstacle()
    obstacle.color.r=.2;obstacle.color.g=.3;obstacle.color.b=.4;obstacle.color.a=0.
    assert SimCameraRendererNode._dynamic_obstacle_color(None,obstacle)[3]==0.


def test_old_retained_obstacle_snapshot_does_not_refresh_visuals():
    from ros2_control_blue_reach_5.msg import DynamicObstacleArray
    node=SimpleNamespace(world_frame='world',obstacle_snapshot_received=0.,
        obstacle_snapshot_timeout=1.,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=3_000_000_000)))
    msg=DynamicObstacleArray();msg.header.frame_id='world';msg.header.stamp.sec=1
    SimCameraRendererNode._dynamic_obstacles_callback(node,msg)
    assert node.obstacle_snapshot_received is None


def test_scene_handoff_preserves_robot_materials():
    from unittest.mock import patch
    from bringup.scene_camera import attach_scene_camera
    import open3d as o3d
    node=SimpleNamespace(robot_description='old',mesh_cache={},renderer_backend='pyvista',
        max_mesh_triangles=12000,
        _visual_mesh_entries=lambda:iter([('robot_1_arm','/mesh',None,None,None,None)]))
    with patch('bringup.scene_camera.SceneParticipant') as participant, \
         patch('bringup.sim_camera_renderer.read_triangle_mesh',return_value=o3d.geometry.TriangleMesh.create_box()) as load:
        attach_scene_camera(node)
        prepare=participant.call_args.args[1]
        prepare('new')
        assert load.call_args.kwargs['preserve_materials'] is True
        assert node.robot_description=='old'

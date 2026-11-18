"""Scene handoff on the camera node's normal rendering thread."""
from bringup.scene_runtime import SceneParticipant, seed_environment_tf, is_environment


def attach_scene_camera(node):
    from bringup.sim_camera_renderer import read_triangle_mesh, open3d_to_polydata
    def prepare(description):
        old = node.robot_description
        cache = {}
        try:
            node.robot_description = description
            for frame, path, *_ in node._visual_mesh_entries():
                if path in node.mesh_cache:
                    cache[path] = node.mesh_cache[path]
                elif path not in cache:
                    mesh = read_triangle_mesh(path, node.max_mesh_triangles,
                                              preserve_materials=True)
                    if mesh.is_empty():
                        raise ValueError('Empty camera mesh: '+path)
                    cache[path] = open3d_to_polydata(mesh) if node.renderer_backend == 'pyvista' else mesh
        finally:
            node.robot_description = old
        return description, cache
    def commit(staged):
        description, cache = staged
        seed_environment_tf(node.tf_buffer, description, node)
        if node.renderer_backend == 'pyvista':
            node.pyvista_plotter.clear()
        else:
            node.scene.clear_geometry()
        node.robot_description = description
        from bringup.scene_runtime import robot_link_owners
        node.robot_link_owners = robot_link_owners(description)
        node.obstacle_snapshot_received = None
        node.mesh_cache = cache; node.mesh_frames.clear(); node.dynamic_obstacle_entries.clear()
        node.scene_bounds_min = node.scene_bounds_max = None
        if node.renderer_backend == 'pyvista':
            node._load_pyvista_urdf_visual_meshes(); node._add_pyvista_water_surface()
            node._setup_robot_lights()
        else:
            node._load_open3d_urdf_visual_meshes(); node._add_open3d_water_surface()
    def verify():
        for value in node.mesh_frames.values():
            frame = value[0] if node.renderer_backend == 'pyvista' else value
            if is_environment(frame) and node._lookup_matrix(frame) is None:
                raise ValueError('Camera environment transform unavailable: '+frame)
        node._update_scene_transforms()
    node.scene_participant = SceneParticipant(node, prepare, commit, verify=verify)

"""Versioned static-scene handoff shared by simulator and SimLab consumers.

Consumers stage and commit one revision while new scene-dependent requests are
blocked. Robot control and plant state are independent of this transaction.
"""
import copy
import hashlib
import xml.etree.ElementTree as ET


def is_environment(name):
    return name.startswith(('bathymetry_', 'world_bottom'))


def robot_link_owners(description):
    """Map articulated descendants to their robot prefix from the shared URDF.

    Ownership follows joints, not mesh filenames or camera selection. Separate
    robots can therefore use identical geometry without becoming one object.
    """
    root = ET.fromstring(description)
    names = {link.get('name') for link in root.findall('link')}
    parents = {j.find('child').get('link'): j.find('parent').get('link')
               for j in root.findall('joint')}
    bases = {name: name[:-len('base_link')] for name in names
             if name.startswith('robot_') and name.endswith('_base_link')}
    # Arm and gripper base links are descendants of the vehicle base, not
    # independent robots. Keep only outermost roots in the articulated tree.
    candidates = set(bases)
    for name in candidates:
        current, seen = parents.get(name), set()
        while current is not None:
            if current in seen:
                raise ValueError('Cycle in scene joint tree: ' + name)
            seen.add(current)
            if current in candidates:
                bases.pop(name, None)
                break
            current = parents.get(current)
    owners = {}
    for name in names:
        current, seen = name, set()
        while current not in bases and current in parents:
            if current in seen:
                raise ValueError('Cycle in scene joint tree: ' + name)
            seen.add(current)
            current = parents[current]
        if current in bases:
            owners[name] = bases[current]
    return owners


def replace_environment(description, scene_file, world_frame, revision, world_xacro):
    import xacro
    from xml.sax.saxutils import quoteattr
    # Xacro recognizes its literal prefix; ElementTree's automatic ns0 prefix
    # leaves macro calls unexpanded in a fresh process.
    doc = xacro.parse('<robot name="scene" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:include filename=' + quoteattr(str(world_xacro)) + '/>'
        '<xacro:sim_world world_frame=' + quoteattr(world_frame) +
        ' scene_file=' + quoteattr(str(scene_file)) + '/></robot>')
    xacro.process_doc(doc)
    scene = ET.fromstring(doc.toxml())
    root = ET.fromstring(description)
    for child in list(root):
        if child.tag == 'link' and is_environment(child.get('name', '')):
            root.remove(child)
        elif child.tag == 'joint' and is_environment(child.find('child').get('link', '')):
            root.remove(child)
    scene_id = hashlib.sha256(ET.tostring(scene)).hexdigest()[:16]
    names = {l.get('name'): l.get('name') + f'_scene_{scene_id}'
             for l in scene.findall('link') if is_environment(l.get('name', ''))}
    if not names:
        raise ValueError('Scene has no environment geometry')
    for link in scene.findall('link'):
        if link.get('name') in names:
            link.set('name', names[link.get('name')]); root.append(copy.deepcopy(link))
    for joint in scene.findall('joint'):
        child = joint.find('child')
        if child.get('link') not in names:
            continue
        parent = joint.find('parent')
        if joint.get('type') != 'fixed' or parent.get('link') not in {world_frame, *names}:
            raise ValueError('Static environment links must attach to world or another environment link')
        if parent.get('link') in names:
            parent.set('link', names[parent.get('link')])
        child.set('link', names[child.get('link')])
        joint.set('name', joint.get('name') + f'_scene_{scene_id}')
        root.append(copy.deepcopy(joint))
    return ET.tostring(root, encoding='unicode')


def environment_transforms(description, stamp):
    from geometry_msgs.msg import TransformStamped
    from scipy.spatial.transform import Rotation
    transforms = []
    for joint in ET.fromstring(description).findall('joint'):
        name = joint.find('child').get('link')
        if not is_environment(name):
            continue
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        t = TransformStamped(); t.header.stamp = stamp
        t.header.frame_id = joint.find('parent').get('link'); t.child_frame_id = name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        q = Rotation.from_euler('xyz', rpy).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
        transforms.append(t)
    return transforms


def seed_environment_tf(buffer, description, node):
    for transform in environment_transforms(description, node.get_clock().now().to_msg()):
        buffer.set_transform_static(transform, 'environment_scene')


class SceneParticipant:
    def __init__(self, node, prepare, commit, *, hold=lambda:None, release=lambda:None,
                 verify=lambda:None, prepare_async=False):
        from ros2_control_blue_reach_5.srv import EnvironmentScene
        self.prepare = prepare; self.commit = commit
        self.on_hold = hold; self.on_release = release; self.verify = verify
        self.prepare_async = prepare_async
        self.preparation = None
        self.revision = 0; self.committed = 0; self.held = False
        self.staged = None; self.prepared = False
        self.service = node.create_service(EnvironmentScene, '~/environment_scene', self.handle)

    def handle(self, request, response):
        try:
            cmd, rev = request.command, request.revision
            if cmd == 'status':
                response.message = f'held={self.held}, committed={self.committed}'
            elif cmd == 'hold':
                if rev < self.revision:
                    raise ValueError('Stale scene revision')
                if rev > self.revision:
                    if self.preparation is not None and not self.preparation.done():
                        raise ValueError('Scene preparation still running')
                    self.preparation = None
                    self.staged = None; self.prepared = False
                self.revision = rev; self.held = True
                self.on_hold()
            else:
                if rev != self.revision or not self.held:
                    raise ValueError('Scene command requires matching held revision')
                if cmd == 'prepare':
                    if self.prepare_async:
                        if self.preparation is None:
                            from concurrent.futures import ThreadPoolExecutor
                            pool = ThreadPoolExecutor(max_workers=1)
                            self.preparation = pool.submit(self.prepare, request.robot_description)
                            pool.shutdown(wait=False)
                        if self.preparation.done():
                            self.staged = self.preparation.result()
                            self.prepared = True
                        response.message = 'ready' if self.prepared else 'pending'
                    else:
                        self.staged = self.prepare(request.robot_description)
                        self.prepared = True
                elif cmd == 'commit':
                    if self.committed != rev:
                        if not self.prepared:
                            raise ValueError('Scene not prepared')
                        self.commit(self.staged)
                        self.committed = rev; self.staged = None
                elif cmd == 'verify':
                    if self.committed != rev:
                        raise ValueError('Scene not committed')
                    self.verify()
                elif cmd == 'release':
                    if self.committed != rev:
                        raise ValueError('Scene not committed')
                    self.verify(); self.on_release(); self.held = False
                else:
                    raise ValueError('Unknown scene phase: '+cmd)
            response.success = True
        except Exception as exc:
            response.success = False; response.message = str(exc)
        response.revision = self.committed
        return response

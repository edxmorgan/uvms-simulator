#!/usr/bin/env python3
"""Stage static OBJ geometry from a pinned Stonefish scene into the URDF world.

Only static models are imported. Robot includes are deliberately excluded.
Unsupported static geometry/expressions fail explicitly; dynamics and water
physics are not translated. No runtime network access or simulator dependency.
"""
import argparse
import hashlib
import json
import io
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

XACRO = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO)


def vector(text, count=3):
    values = [float(x) for x in text.split()]
    if len(values) != count or not np.isfinite(values).all():
        raise ValueError(f'Expected {count} finite values: {text}')
    return values


def pose(element):
    if element is None:
        return [0., 0., 0.], [0., 0., 0.]
    return vector(element.get('xyz', '0 0 0')), vector(element.get('rpy', '0 0 0'))


def ned_to_nwu(xyz, rpy):
    flip = np.diag([1., -1., -1.])
    return (flip @ xyz).tolist(), Rotation.from_matrix(
        flip @ Rotation.from_euler('xyz', rpy).as_matrix()).as_euler('xyz').tolist()


def safe_path(path):
    p = PurePosixPath(path)
    if p.is_absolute() or '..' in p.parts or '$' in path or not p.parts:
        raise ValueError(f'Unsupported asset path: {path}')
    return p.as_posix()


def format_vec(v):
    return ' '.join(f'{x:.17g}' for x in v)


def expand_object_includes(scene_text, fetch):
    """Resolve object fragments and literal arguments; keep robot includes excluded."""
    sources = {}
    def expand(text, stack=()):
        root = ET.fromstring(text)
        for inc in list(root.findall('include')):
            path = inc.get('file', '')
            prefix = '$(find stonefish_bluerov2)/scenarios/'
            if path.startswith(prefix + 'robots/'):
                continue
            if not path.startswith(prefix + 'objects/'):
                raise ValueError('Unsupported scene include: ' + path)
            relative = safe_path(path[len(prefix):])
            if relative in stack:
                raise ValueError('Recursive scene include: ' + relative)
            raw = fetch(relative)
            sources[relative] = raw
            args = {a.get('name'): a.get('value') for a in inc.findall('arg')}
            def substitute(match):
                if match[1] not in args:
                    raise ValueError('Missing include argument: ' + match[1])
                return args[match[1]]
            resolved = re.sub(r'\$\(arg ([\w_]+)\)', substitute, raw)
            children = expand(resolved, stack + (relative,))
            index = list(root).index(inc)
            root.remove(inc)
            for child in reversed(list(children)):
                root.insert(index, child)
        return root
    return ET.tostring(expand(scene_text), encoding='unicode'), sources


def automatic_seabed_crop(root, fetch, margin, grid_half_extent=0.):
    """Bounds in seabed mesh-local XY, enclosing structures and world origin."""
    if not np.isfinite(margin) or margin <= 0:
        raise ValueError('Seabed margin must be finite and positive')
    if not np.isfinite(grid_half_extent) or grid_half_extent < 0:
        raise ValueError('Grid half extent must be finite and nonnegative')
    seabeds = [o for o in root.findall('static') if o.get('name', '').lower() == 'seabed']
    if len(seabeds) != 1:
        raise ValueError('Automatic crop requires exactly one Seabed')
    def matrix(element):
        xyz, rpy = pose(element)
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        m[:3, 3] = xyz
        return m
    bed = seabeds[0]
    spec = bed.find('physical')
    scale = float(spec.find('mesh').get('scale', '1'))
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid seabed scale')
    # Visual and physical bed must have the same placement to share one crop.
    visual = bed.find('visual')
    if visual is not None and (pose(visual.find('origin')) != pose(spec.find('origin')) or
            float(visual.find('mesh').get('scale', '1')) != scale):
        raise ValueError('Automatic crop requires matching seabed visual/collision placement')
    inverse = np.linalg.inv(matrix(bed.find('world_transform')) @ matrix(spec.find('origin')))
    points = [trimesh.transform_points([[0, 0, 0]], inverse) / scale]
    for obj in root.findall('static'):
        if obj is bed:
            continue
        for kind in ('physical', 'visual'):
            part = obj.find(kind)
            if part is None:
                continue
            source = part.find('mesh')
            mesh = trimesh.load(io.BytesIO(fetch(safe_path(source.get('filename')))),
                                file_type='obj', force='mesh', process=False)
            vertices = np.asarray(mesh.vertices) * float(source.get('scale', '1'))
            transform = inverse @ matrix(obj.find('world_transform')) @ matrix(part.find('origin'))
            points.append(trimesh.transform_points(vertices, transform) / scale)
    points = np.concatenate(points)
    lo = points[:, :2].min(axis=0) - margin / scale
    hi = points[:, :2].max(axis=0) + margin / scale
    if grid_half_extent:
        # RViz's world-centred square: include it without adding the structure
        # margin again. NED/NWU have the same symmetric XY square.
        corners = [[x, y, 0.] for x in (-grid_half_extent, grid_half_extent)
                              for y in (-grid_half_extent, grid_half_extent)]
        grid = trimesh.transform_points(corners, inverse) / scale
        lo = np.minimum(lo, grid[:, :2].min(axis=0))
        hi = np.maximum(hi, grid[:, :2].max(axis=0))
    return (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]))


def limit_seabed_depth(macro, objects, floor, max_depth):
    """Translate the complete static world upward; never scale its geometry."""
    if max_depth is None:
        return floor, 0.
    if not np.isfinite(max_depth) or max_depth <= 0:
        raise ValueError('Maximum seabed depth must be finite and positive')
    offset = max(0., -float(max_depth) - floor)
    if offset:
        for joint in macro.findall('joint'):
            if joint.find('parent').get('link') != '${world_frame}':
                raise ValueError('Depth translation requires world-parented scene joints')
            origin = joint.find('origin')
            xyz = vector(origin.get('xyz'))
            xyz[2] += offset
            origin.set('xyz', format_vec(xyz))
        for record in objects:
            record['world_xyz_nwu'][2] += offset
    return floor + offset, offset


def convert(scene_text, fetch, output, provenance, *, project_missing_uv=False, seabed_half_extent=None, missing_look_rgb=None, seabed_margin=None, seabed_grid_half_extent=0., max_seabed_depth=None):
    """fetch(relative_path) returns bytes from the scenario's data directory."""
    if seabed_half_extent is not None and (not np.isfinite(seabed_half_extent) or seabed_half_extent <= 0):
        raise ValueError('Seabed half extent must be finite and positive')
    root = ET.fromstring(scene_text)
    if root.findall('dynamic') or root.findall('animated'):
        raise ValueError('Moving objects require explicit dynamic-world adaptation')
    for inc in root.findall('include'):
        if '/scenarios/robots/' not in inc.get('file', ''):
            raise ValueError('Expand object includes before conversion')
    looks = {}
    for look in root.findall('looks/look'):
        name = look.get('name')
        if name in looks and looks[name] != look.attrib:
            raise ValueError(f'Conflicting look: {name}')
        looks[name] = look.attrib
    assets = output / 'assets'; assets.mkdir()
    originals = {}; generated = {}; objects = []; adaptations = []

    def download(path):
        path = safe_path(path)
        if path not in originals:
            data = fetch(path)
            originals[path] = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
            cache[path] = data
        return cache[path]

    cache = {}

    if seabed_grid_half_extent and seabed_margin is None:
        raise ValueError('Grid coverage requires automatic seabed margin cropping')
    if seabed_margin is not None and seabed_half_extent is not None:
        raise ValueError('Choose seabed margin or half extent, not both')
    crop_bounds = automatic_seabed_crop(root, download, seabed_margin, seabed_grid_half_extent) if seabed_margin is not None else (
        (-seabed_half_extent, seabed_half_extent, -seabed_half_extent, seabed_half_extent)
        if seabed_half_extent is not None else None)

    def mesh_asset(path, look=None, crop=None):
        path = safe_path(path)
        if not path.lower().endswith('.obj'):
            raise ValueError('Only static OBJ meshes are supported: ' + path)
        key = (path, json.dumps(look, sort_keys=True), crop)
        if key in generated:
            return generated[key]
        token = hashlib.sha256(repr(key).encode()).hexdigest()[:12]
        name = Path(path).stem + '_' + token
        data = download(path).decode('utf-8')
        if crop is not None:
            mesh = trimesh.load(io.StringIO(data), file_type='obj', force='mesh', process=False)
            xmin, xmax, ymin, ymax = crop
            for normal, origin in (([1,0,0],[xmin,0,0]), ([-1,0,0],[xmax,0,0]),
                                   ([0,1,0],[0,ymin,0]), ([0,-1,0],[0,ymax,0])):
                mesh = mesh.slice_plane(origin, normal)
            if mesh.is_empty:
                raise ValueError('Seabed crop removed all geometry')
            data = trimesh.exchange.obj.export_obj(mesh, include_texture=False, digits=17)
            note = f'Seabed clipped in local XY to {crop} mesh units: {path}'
            if note not in adaptations:
                adaptations.append(note)
        # Stonefish looks supply materials separately; prevent unresolved upstream MTLs.
        data = '\n'.join(l for l in data.splitlines() if not l.startswith(('mtllib ', 'usemtl ')))+'\n'
        if look is not None:
            rgb = vector(look['rgb']) if 'rgb' in look else [float(look.get('gray', 1))]*3
            if not np.isfinite(rgb).all() or min(rgb) < 0 or max(rgb) > 1:
                raise ValueError('Invalid material color')
            mtl = f'newmtl surface\nKd {format_vec(rgb)}\n'
            if look.get('texture'):
                texpath = safe_path(look['texture']); texdata = download(texpath)
                texname = hashlib.sha256(texdata).hexdigest()[:16]+Path(texpath).suffix
                (assets/texname).write_bytes(texdata)
                if not any(l.startswith('vt ') for l in data.splitlines()):
                    vertices = np.array([vector(l[2:]) for l in data.splitlines() if l.startswith('v ')])
                    extents = np.ptp(vertices, axis=0)
                    axes = np.argsort(extents)[-2:]
                    if not project_missing_uv or np.min(extents[axes]) <= 0:
                        raise ValueError('Textured mesh needs explicit --project-missing-uv: '+path)
                    uv = vertices[:, axes]
                    rows = ['vt '+format_vec(v) for v in uv]
                    for line in data.splitlines():
                        if line.startswith('f '):
                            tokens = []
                            for token in line[2:].split():
                                parts = token.split('/')
                                tokens.append(parts[0]+'/'+parts[0]+('/'+parts[2] if len(parts)==3 else ''))
                            line = 'f '+' '.join(tokens)
                        rows.append(line)
                    data = '\n'.join(rows)+'\n'
                    adaptations.append('Planar UV coordinates generated (1 local metre repeat): '+path)
                mtl += 'map_Kd '+texname+'\n'
            (assets/(name+'.mtl')).write_text(mtl)
            data = f'mtllib {name}.mtl\nusemtl surface\n'+data
        (assets/(name+'.obj')).write_text(data)
        generated[key] = name+'.obj'
        return name+'.obj'

    doc = ET.Element('robot')
    macro = ET.SubElement(doc, '{'+XACRO+'}macro', name='scene_environment', params='world_frame asset_directory')
    floor = None
    for index, obj in enumerate(root.findall('static')):
        if obj.get('type') != 'model':
            raise ValueError('Unsupported static type: '+str(obj.attrib))
        name = 'bathymetry_'+re.sub('[^a-zA-Z0-9_]', '_', obj.get('name','object')).lower()+f'_{index}'
        xyz, rpy = pose(obj.find('world_transform'))
        xyz, rpy = ned_to_nwu(xyz, rpy)
        link = ET.SubElement(macro, 'link', name=name)
        joint = ET.SubElement(macro, 'joint', name=name+'_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='${world_frame}')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=format_vec(xyz), rpy=format_vec(rpy))
        physical = obj.find('physical')
        if physical is None or physical.find('mesh') is None:
            raise ValueError('Missing physical mesh: '+name)
        visual = obj.find('visual')
        look_ref = obj.find('look')
        look = looks.get(look_ref.get('name')) if look_ref is not None else None
        if look_ref is not None and look is None:
            if missing_look_rgb is None:
                raise ValueError('Missing look for '+name)
            look = {'rgb': format_vec(missing_look_rgb)}
            adaptations.append(f"Undefined look {look_ref.get('name')} on {name}: explicit fallback RGB {format_vec(missing_look_rgb)}")
        record = {'name':name, 'world_xyz_nwu':xyz, 'world_rpy_nwu':rpy}
        for tag, spec in [('collision',physical),('visual',visual if visual is not None else physical)]:
            mesh = spec.find('mesh'); scale = float(mesh.get('scale','1'))
            if not np.isfinite(scale) or scale <= 0:
                raise ValueError('Mesh scale must be finite and positive')
            local_xyz, local_rpy = pose(spec.find('origin'))
            asset = mesh_asset(mesh.get('filename'), look if tag=='visual' else None,
                               crop_bounds if obj.get('name','').lower()=='seabed' else None)
            node = ET.SubElement(link,tag)
            ET.SubElement(node,'origin',xyz=format_vec(local_xyz),rpy=format_vec(local_rpy))
            geom = ET.SubElement(node,'geometry')
            ET.SubElement(geom,'mesh',filename='file://${asset_directory}/assets/'+asset,scale=format_vec([scale]*3))
            record[tag] = {'source_mesh':mesh.get('filename'), 'asset':asset,
                           'scale':scale, 'xyz':local_xyz, 'rpy':local_rpy}
        objects.append(record)
        if obj.get('name','').lower()=='seabed':
            floor = xyz[2]
    if not objects or floor is None:
        raise ValueError('Scene needs static meshes and a Seabed floor reference')
    ET.SubElement(macro,'link',name='world_bottom')
    j=ET.SubElement(macro,'joint',name='world_bottom_joint',type='fixed')
    ET.SubElement(j,'parent',link='${world_frame}');ET.SubElement(j,'child',link='world_bottom')
    ET.SubElement(j,'origin',xyz=f'0 0 {floor:.17g}',rpy='0 0 0')
    original_floor = floor
    floor, vertical_offset = limit_seabed_depth(macro, objects, floor, max_seabed_depth)
    if vertical_offset:
        adaptations.append(f'Entire static scene translated upward by {vertical_offset:g} m; seabed depth limited to {max_seabed_depth:g} m; geometry sizes preserved')
    ET.indent(doc)
    ET.ElementTree(doc).write(output/'world.xacro',encoding='utf-8',xml_declaration=True)
    (output/'source.scn.template').write_text(scene_text)
    metadata = dict(schema_version=1, source=provenance, source_frame='NED', world_frame='NWU',
                    floor_z=floor, source_floor_z=original_floor, vertical_offset_m=vertical_offset, max_seabed_depth_m=max_seabed_depth, seabed_crop_local_xy=crop_bounds, seabed_margin_m=seabed_margin, seabed_grid_half_extent_m=seabed_grid_half_extent, objects=objects, upstream_assets=originals, adaptations=adaptations,
                    excluded_includes=[e.get('file') for e in root.findall('include')],
                    limitations=['Static geometry only; no Stonefish ocean/lighting/physics',
                                 'Visual/collision surfaces share crop; no collision decimation',
                                 'Camera texture sampling uses existing vertex-color renderer'])
    metadata['artifact_sha256']={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted(output.rglob('*')) if p.is_file()}
    (output/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository',required=True,help='GitHub owner/repository')
    parser.add_argument('--revision',required=True,help='Pinned 40-character Git commit')
    parser.add_argument('--scenario',required=True)
    parser.add_argument('--data-root',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--missing-look-rgb',type=float,nargs=3,help='Explicit fallback color for undefined upstream looks')
    parser.add_argument('--max-seabed-depth', type=float, help='Raise the entire scene if its seabed is deeper than this many metres; preserve geometry sizes')
    parser.add_argument('--seabed-margin',type=float,help='Crop seabed around all structures and world origin, with this margin in metres')
    parser.add_argument('--seabed-grid-half-extent',type=float,default=0.,help='Also cover a world-centred square of this half extent in metres; requires --seabed-margin')
    parser.add_argument('--seabed-half-extent',type=float,help='Clip Seabed mesh local XY to +/- this many mesh units')
    parser.add_argument('--project-missing-uv',action='store_true',help='Explicitly allow planar texture projection for missing UVs')
    args=parser.parse_args()
    if not re.fullmatch(r'[\w.-]+/[\w.-]+',args.repository) or not re.fullmatch('[0-9a-f]{40}',args.revision):
        parser.error('Use owner/repository and a pinned commit SHA')
    destination=args.output.resolve()
    if destination.exists():parser.error('Output already exists; choose a new directory')
    base=f'https://raw.githubusercontent.com/{args.repository}/{args.revision}/'
    def read(path):
        with urllib.request.urlopen(base+safe_path(path),timeout=30) as response:return response.read()
    scene=read(args.scenario).decode('utf-8')
    original_scene = scene
    scenario_root = safe_path(args.data_root).rsplit('/', 1)[0] + '/scenarios/'
    scene, includes = expand_object_includes(scene, lambda p: read(scenario_root+p).decode('utf-8'))
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent,prefix='.scene-import-') as staging:
        out=Path(staging)/'scene';out.mkdir()
        meta=convert(scene,lambda p:read(safe_path(args.data_root)+'/'+p),out,
                     {'repository':args.repository,'revision':args.revision,'scenario':args.scenario,
                      'data_root':args.data_root}, project_missing_uv=args.project_missing_uv, seabed_half_extent=args.seabed_half_extent, missing_look_rgb=args.missing_look_rgb, seabed_margin=args.seabed_margin, seabed_grid_half_extent=args.seabed_grid_half_extent, max_seabed_depth=args.max_seabed_depth)
        (out/'source.scn.template').write_text(original_scene)
        for name, text in includes.items():
            target = out/'source_includes'/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(text)
        (out/'LICENSE.upstream').write_bytes(read('LICENSE'))
        (out/'NOTICE').write_text(f'Scene assets adapted from https://github.com/{args.repository}\n'
            f'Commit: {args.revision}\nRetain upstream LICENSE.upstream and this notice.\n'
            'Modifications: NED-to-NWU placement; OBJ materials reconstructed from Stonefish looks.\n'
            'Robot assets and behavior are excluded.\n' + '\n'.join(meta['adaptations']) + '\n')
        meta['artifact_sha256'] = {str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
        (out/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
        shutil.move(str(out),str(destination))
    print(f'Imported {len(meta["objects"])} static objects into {destination}')


if __name__=='__main__':main()

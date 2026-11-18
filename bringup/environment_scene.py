"""Resolve static scene bundles; all consumers still use the shared URDF."""
from pathlib import Path
import re


def resolve_environment_scene(selection, package_share):
    selection = str(selection).strip()
    if selection in ('', 'default'):
        return ''
    path = Path(selection).expanduser()
    if not path.is_absolute():
        if not re.fullmatch(r'[A-Za-z0-9_-]+', selection):
            raise ValueError('environment_scene must be a bundled name or absolute world.xacro path')
        path = Path(package_share) / 'scenes' / selection / 'world.xacro'
    elif path.is_dir():
        path = path / 'world.xacro'
    if not path.is_file() or path.suffix != '.xacro':
        raise ValueError(f'Environment scene does not exist: {path}')
    return str(path.resolve())

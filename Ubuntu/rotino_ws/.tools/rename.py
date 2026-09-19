"""sebaju -> rotino rename, applied to file contents. Longest patterns first."""
import pathlib, sys

BASE = [
    ('sebaju_dashboard', 'rotino_dashboard'),
    ('sebaju_world', 'rotino_world'),
    ('sebaju_controllers', 'rotino_controllers'),
    ('sebaju_plotjuggler_layout', 'rotino_plotjuggler_layout'),
    ('sebaju.urdf.xacro', 'rotino.urdf.xacro'),
    ('SeBaJu', 'RoTino'),
    ('SEBAJU', 'ROTINO'),
    ('Sebaju', 'Rotino'),
    ('/sebaju/', '/rotino/'),
    ('sebaju::', 'rotino::'),
]

def convert(text, pkg_map=None, extra=None):
    for a, b in (extra or []):
        text = text.replace(a, b)
    for a, b in (pkg_map or []):
        text = text.replace(a, b)
    for a, b in BASE:
        text = text.replace(a, b)
    return text.replace('sebaju', 'rotino')   # catch-all, last

def apply(paths, pkg_map=None, extra=None):
    for p in paths:
        f = pathlib.Path(p)
        if not f.is_file():
            continue
        try:
            t = f.read_text()
        except UnicodeDecodeError:
            continue
        n = convert(t, pkg_map, extra)
        if n != t:
            f.write_text(n)
            print(f'  {f.name}')

if __name__ == '__main__':
    root = pathlib.Path(sys.argv[1])
    pkg = sys.argv[2] if len(sys.argv) > 2 else 'rotino_description'
    apply(sorted(root.rglob('*')), pkg_map=[('sebaju_gazebo', pkg)])

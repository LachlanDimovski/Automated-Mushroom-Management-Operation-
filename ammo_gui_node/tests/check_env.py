#!/usr/bin/env python3
"""Check this machine can actually run the simulation, and say what to fix.

    python3 src/RS1_Team5_/ammo_gui_node/tests/check_env.py

Nothing in this repo declares what the detector needs - `perception`'s
package.xml lists rclpy, sensor_msgs, vision_msgs and cv_bridge, while
`yolo_node.py` imports torch, cv2 and ultralytics - so `rosdep install` has
never been able to produce a working machine. Everybody pip-installed by hand
on a different day and ended up with a different environment, which is why a
run that works for one person fails for the next with no obvious difference
between them.

This is that missing declaration, as a script. Every check prints what it
found and, when something is wrong, the one command that fixes it. It needs
no simulation and no display, and it degrades gracefully: run it before ROS
is sourced and it will tell you that too.
"""

import os
import shutil
import subprocess
import sys

OK, WARN, BAD = 'ok  ', 'warn', 'FAIL'
results = []


def report(level, what, detail='', fix=''):
    results.append((level, what, detail, fix))
    line = f'  [{level}] {what}'
    if detail:
        line += f'  -  {detail}'
    print(line)
    if fix and level != OK:
        print(f'         fix: {fix}')


def section(title):
    print(f'\n{title}\n' + '-' * len(title))


def version_of(module):
    try:
        mod = __import__(module)
    except Exception as exc:          # noqa: BLE001 - any import failure counts
        return None, str(exc).splitlines()[0][:120]
    return getattr(mod, '__version__', 'unknown'), None


# ---------------------------------------------------------------- python ----
section('python')
report(OK if sys.version_info[:2] >= (3, 10) else WARN,
       'python', sys.version.split()[0],
       'ROS 2 Humble expects 3.10 - a different interpreter means pip installs '
       'land somewhere ROS will not look')

# ----------------------------------------------------------------- numpy ----
section('numpy, and what it breaks')
numpy_version, numpy_error = version_of('numpy')
if numpy_error:
    report(BAD, 'numpy', numpy_error, 'sudo apt install -y python3-numpy')
else:
    major = int(str(numpy_version).split('.')[0] or 0)
    # The one that bites. cv_bridge and the torch wheels on Humble are C
    # extensions built against the numpy 1.x ABI. pip install ultralytics
    # happily pulls numpy 2.x on top of them, and the failure is not a
    # version complaint - it is an ImportError deep inside cv_bridge, or
    # torch printing a wall of text about a module compiled with NumPy 1.x.
    report(WARN if major >= 2 else OK, 'numpy', numpy_version,
           'numpy 2.x breaks cv_bridge and the Humble torch wheels. '
           'pip3 install "numpy<2"' if major >= 2 else '')

# ------------------------------------------------------------------ ROS ----
section('ROS')
distro = os.environ.get('ROS_DISTRO')
report(OK if distro else BAD, 'ROS_DISTRO', distro or 'not set',
       'source /opt/ros/humble/setup.bash')
prefixes = os.environ.get('AMENT_PREFIX_PATH', '')
report(OK if 'install' in prefixes else BAD, 'workspace sourced',
       'yes' if 'install' in prefixes else 'no',
       'cd ~/41068_ws && source install/setup.bash')

for mod, needed_by in (('rclpy', 'everything'),
                       ('cv_bridge', 'the detectors'),
                       ('vision_msgs', 'detection boxes')):
    v, err = version_of(mod)
    # cv_bridge is the canary for the numpy ABI: it is the first thing to
    # fail, and it fails with a message that never mentions numpy.
    hint = ('pip3 install "numpy<2"  (this is the numpy 2 breakage, '
            'not a missing package)' if err and 'numpy' in (err or '').lower()
            else 'source /opt/ros/humble/setup.bash')
    report(OK if not err else BAD, mod, v or err, '' if not err else hint)

# ------------------------------------------------------------ the stack ----
section('the detector stack (perception, quad_zcams)')
for mod, fix, needed in (
        ('torch', 'pip3 install torch --index-url https://download.pytorch.org/whl/cpu', True),
        ('ultralytics', 'pip3 install ultralytics', True),
        ('cv2', 'pip3 install opencv-python', True),
        ('openvino', 'pip3 install openvino', False),
        ('scipy', 'sudo apt install -y python3-scipy', False),
        ('PIL', 'sudo apt install -y python3-pil', False)):
    v, err = version_of(mod)
    level = OK if not err else (BAD if needed else WARN)
    note = '' if not err else (fix if needed else fix + '   (only needed for a fleet)')
    report(level, mod, v or err, note)

# -------------------------------------------------------------- the GUI ----
section('the GUI')
v, err = version_of('PySide6')
report(OK if not err else BAD, 'PySide6', v or err, 'pip3 install PySide6')
if not err:
    try:
        env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
        probe = subprocess.run(
            [sys.executable, '-c',
             'from PySide6.QtWidgets import QApplication; QApplication([]); print("ok")'],
            capture_output=True, text=True, timeout=60, env=env)
        good = 'ok' in probe.stdout
        report(OK if good else BAD, 'Qt can start',
               'yes' if good else (probe.stderr or '').strip().splitlines()[-1][:120]
               if probe.stderr else 'no',
               'sudo apt install -y libxcb-cursor0')
    except Exception as exc:          # noqa: BLE001
        report(WARN, 'Qt can start', str(exc)[:100])
for var, want in (('QT_QPA_PLATFORM', 'xcb'), ('LIBGL_ALWAYS_SOFTWARE', '1')):
    got = os.environ.get(var)
    report(OK if got else WARN, var, got or 'not set',
           f'export {var}={want}   (needed in the terminal that launches the GUI)')

# ------------------------------------------------------------- the sim -----
section('Gazebo')
ign = shutil.which('ign') or shutil.which('gz')
report(OK if ign else BAD, 'ign/gz on PATH', ign or 'not found',
       'sudo apt install -y ignition-fortress')

# ------------------------------------------------------------ git lfs ------
section('assets (Git LFS)')
lfs = shutil.which('git-lfs')
report(OK if lfs else BAD, 'git-lfs installed', lfs or 'not found',
       'sudo apt install -y git-lfs && git lfs install && git lfs pull')

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
POINTER = b'version https://git-lfs.github.com/spec/v1'
ASSETS = [
    ('mushroom mesh', 'models/mushroom_amanita/meshes/mushroom.obj'),
    ('mushroom texture', 'models/mushroom_amanita/meshes/material_0.png'),
    ('flat terrain heightmap', 'models/forest_flat_terrain/heightmap.png'),
    ('random terrain heightmap', 'models/forest_random_terrain/heightmap.png'),
    ('oak bark texture', 'models/tree_oak/materials/textures/bark_diffuse.png'),
]


def check_assets(root, where):
    """A pointer here is why mushrooms spawn invisibly and worlds load black."""
    if not os.path.isdir(root):
        report(WARN, f'{where} models', 'not found at ' + root)
        return
    for name, rel in ASSETS:
        path = os.path.join(root, rel)
        if not os.path.exists(path):
            report(WARN, f'{name} ({where})', 'missing')
            continue
        try:
            with open(path, 'rb') as fh:
                head = fh.read(len(POINTER))
        except OSError as exc:
            report(WARN, f'{name} ({where})', str(exc)[:80])
            continue
        size = os.path.getsize(path)
        if head == POINTER:
            report(BAD, f'{name} ({where})', f'Git LFS pointer, {size} bytes',
                   'git lfs install && git lfs pull'
                   + (' && colcon build --packages-select 41068_ignition_bringup'
                      if where == 'installed' else ''))
        else:
            report(OK, f'{name} ({where})', f'{size / 1e6:.1f} MB')


check_assets(os.path.join(REPO, '41068_ignition_bringup'), 'source')

installed = None
for prefix in prefixes.split(':'):
    candidate = os.path.join(prefix, 'share', '41068_ignition_bringup')
    if prefix and os.path.isdir(candidate):
        installed = candidate
        break
if installed:
    # The build copies models into install/, so pulling LFS after a build
    # leaves pointers here however good the source tree looks.
    check_assets(installed, 'installed')
else:
    report(WARN, 'installed models', 'not found - workspace not built or not sourced',
           'cd ~/41068_ws && colcon build')

# --------------------------------------------------------- the weights -----
section('detector weights')
for prefix in prefixes.split(':'):
    share = os.path.join(prefix, 'share', 'perception', 'models')
    if prefix and os.path.isdir(share):
        found = []
        for base, _dirs, files in os.walk(share):
            found += [os.path.join(base, f) for f in files if f.endswith('.pt')]
        if found:
            biggest = max(found, key=os.path.getsize)
            size = os.path.getsize(biggest)
            report(OK if size > 100_000 else BAD, 'YOLO weights',
                   f'{os.path.basename(biggest)}, {size / 1e6:.1f} MB',
                   'git lfs pull && colcon build --packages-select perception'
                   if size <= 100_000 else '')
        else:
            report(WARN, 'YOLO weights', 'none found under ' + share)
        break

# ----------------------------------------------------------------- done ----
bad = [r for r in results if r[0] == BAD]
warn = [r for r in results if r[0] == WARN]
print('\n' + '=' * 64)
if bad:
    print(f'{len(bad)} thing(s) will stop this working:')
    for _l, what, _d, fix in bad:
        print(f'  - {what}' + (f'\n      {fix}' if fix else ''))
elif warn:
    print('Nothing is broken. Worth a look:')
    for _l, what, detail, fix in warn:
        print(f'  - {what}: {detail}' + (f'\n      {fix}' if fix else ''))
else:
    print('Everything checks out.')
print('=' * 64)
sys.exit(1 if bad else 0)

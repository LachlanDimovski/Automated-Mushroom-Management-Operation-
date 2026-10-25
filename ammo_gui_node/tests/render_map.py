"""Render the map offscreen against both world-meta shapes, and look at it.

The river meta is built here to match what forest_worlds/build_world.py
writes, because no river world is committed to the repo - they are generated
by setup.sh.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: F401,E402  (sets sys.path and the headless Qt platform)

from PySide6.QtCore import QSize                               # noqa: E402
from PySide6.QtGui import QImage                               # noqa: E402
from PySide6.QtWidgets import QApplication                     # noqa: E402

from ammo_gui_node import contract as C                        # noqa: E402
from ammo_gui_node import theme                                # noqa: E402
from ammo_gui_node.map_view import MapView                     # noqa: E402

app = QApplication(sys.argv)
app.setStyleSheet(theme.STYLESHEET)


def river_meta(half=29.1, size=56.2):
    """The shape build_world.py writes: trees only, plus river/fords/natives."""
    line = []
    for i in range(0, 900, 4):
        x = -half - 4 + (2 * half + 8) * i / 900.0
        y = 6.0 * math.sin(x / 11.0) - 2.0
        r = round(2.6 + 0.7 * math.sin(x / 7.0 + 1.3) + 1.4, 2)
        line.append([round(x, 2), round(y, 2), r])
    fords, tangents = [], []
    for frac in (0.3, 0.55, 0.8):
        i = int(frac * len(line))
        fords.append([line[i][0], line[i][1]])
        dx = line[min(i + 1, len(line) - 1)][0] - line[max(i - 1, 0)][0]
        dy = line[min(i + 1, len(line) - 1)][1] - line[max(i - 1, 0)][1]
        n = math.hypot(dx, dy) or 1.0
        tangents.append([round(dx / n, 3), round(dy / n, 3)])
    trees = []
    for k in range(65):
        a = k * 2.39996
        rad = 4.0 + 24.0 * (k / 65.0) ** 0.5
        tx, ty = rad * math.cos(a), rad * math.sin(a)
        if abs(ty - (6.0 * math.sin(tx / 11.0) - 2.0)) < 5.0:
            continue
        trees.append({'type': 'tree', 'x': round(tx, 3), 'y': round(ty, 3), 'radius': 0.3})
    return {
        'world': {'name': 'river_moderate_aus', 'playable_size_m': size, 'terrain_size_m': 60.0},
        'hedge': {'half_size': half},
        'spawn_clearing': {'center': [0.0, 0.0], 'radius': 4.0, 'ground_z': 0.0},
        'mushrooms': [{'name': f'mushroom_{i:02d}', 'x': x, 'y': y, 'z': 0.0}
                      for i, (x, y) in enumerate([(12.0, 9.0), (-14.0, 7.5), (18.0, -11.0),
                                                  (-9.0, -13.0), (6.0, 14.0), (-20.0, -4.0)])],
        'natives': [{'name': f'native_{i:02d}', 'x': x, 'y': y, 'z': 0.0}
                    for i, (x, y) in enumerate([(3.0, 11.0), (-6.0, -9.0)])],
        'fords': fords,
        'river': {'line': line, 'ford_tangents': tangents},
        'objects': trees,
    }


def render(view, path, size=(960, 760)):
    view.resize(QSize(*size))
    image = QImage(size[0], size[1], QImage.Format_RGB32)
    image.fill(0)
    view.render(image)
    image.save(path)
    print('wrote', path, f'({os.path.getsize(path)} bytes)')


# --- a river world, mid-mission --------------------------------------------
view = MapView(interactive=True, compact=False)
view.set_world(river_meta())
view.set_rover(7.0, 3.0, 0.6)
for i in range(60):
    view.set_rover(7.0 - i * 0.12, 3.0 + i * 0.05, 0.6)
view.set_quad(-4.0, 8.0, 2.2, 1.1)
for i in range(80):
    view.set_quad(-4.0 + i * 0.16, 8.0 - i * 0.07, 2.2, 1.1)
view.set_mushrooms([
    {'n': 1, 'x': 12.0, 'y': 9.0, 'conf': 0.91, 'status': 'collected'},
    {'n': 2, 'x': -14.0, 'y': 7.5, 'conf': 0.74, 'status': 'found'},
    {'n': 3, 'x': 18.0, 'y': -11.0, 'conf': 0.63, 'status': 'abandoned'},
    {'n': 4, 'x': -9.0, 'y': -13.0, 'conf': 0.55, 'status': 'missed'},
])
view.set_natives([[3.0, 11.0], [-6.0, -9.0]])
view.set_markers([
    {'kind': C.MK_FORD, 'x': view.fords[1][0], 'y': view.fords[1][1]},
    {'kind': C.MK_GAVE_UP, 'x': 15.0, 'y': -6.0},
    {'kind': C.MK_EMPTY, 'x': -2.0, 'y': -16.0},
    {'kind': C.MK_WEDGE, 'x': 20.0, 'y': 4.0},
])
view.set_route({'sx': -12.5, 'sy': 6.8, 'gx': -14.0, 'gy': 7.5, 'vias': 2})
view.set_goal((9.0, -8.0))
render(view, _paths.out_dir() + '/map_river.png')
print('river key:', [k for k, _ in view.key_entries()])

# --- the committed forest meta, which has nine object types ----------------
with open(_paths.FOREST_META) as fh:
    forest = json.load(fh)
view2 = MapView(interactive=False, compact=False)
view2.set_world(forest)
view2.set_rover(2.0, 1.0, 1.2)
view2.set_quad(-5.0, 4.0, 0.3, 2.0)
view2.set_mushrooms([{'n': 1, 'x': m['x'], 'y': m['y'], 'conf': 0.8, 'status': 'found'}
                     for m in forest['mushrooms'][:2]])
render(view2, _paths.out_dir() + '/map_forest.png')
print('forest key:', [k for k, _ in view2.key_entries()])
print('forest object types drawn:', sorted(view2.objects))

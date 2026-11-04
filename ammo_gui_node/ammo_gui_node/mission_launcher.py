"""Starts, prepares, runs, pauses and tears down the mission.

A run has two stages now.

**Prepare** starts the team's own `cage_mission.launch.py` - Gazebo, Nav2,
both robots, perception and the mission nodes - and then, once everything is
actually up, freezes the two mission node processes. The simulation keeps
running; nothing is searching or driving. Mushrooms are spawned after that
freeze, so the quadcopter cannot have seen any of them before you look.

**Launch** thaws those two processes and starts the clock.

Ending a mission freezes them again and removes the mushrooms, but leaves
Gazebo up, so the next run costs nothing. Only changing the world tears the
simulation down.

That split needs no change to anyone else's launch file, which matters: the
alternative was a second launch path that would drift from theirs the first
time they edited it. What makes it work is that a frozen process is exactly
a process that has started, connected and then stopped - which is precisely
the state "prepared" should mean.
"""
import glob
import json
import math
import os
import re
import random
import signal
import subprocess
import time

from ammo_gui_node import contract as C

# executables inside the launch, by role
ROVER_NODES = ('rover_capture_node', 'cage_capture_node')
GUARD_NODES = ('collision_guard_node',)
# `fleet_quad_node` is the fleet's own quad executable (a FleetQuad subclass
# of ZCamQuad). Leaving it out meant `mission_nodes_up()` never saw a quad on
# a fleet run, so the checklist sat on "Mission nodes" forever with
# everything else green.
QUAD_NODES = ('zcam_quad_node', 'quad_search_node', 'fleet_quad_node')
MISSION_NODES = ROVER_NODES + QUAD_NODES          # the guard stays awake
STOP_TIMEOUT_S = 20.0
SIM_WARMUP_S = 45.0       # below this, an exit means it failed rather than finished

# states
IDLE = 'idle'             # nothing running
PREPARING = 'preparing'   # the launch is up but not everything has arrived
READY = 'ready'           # everything is up and the mission nodes are frozen
RUNNING = 'running'       # the mission nodes are thawed and the clock is going

# name -> folder holding <name>.sdf (None = 41068_ignition_bringup/worlds)
#
# The team settled on two worlds for the demo, so these are the only two
# offered: `simple_trees` for a quick run that always works, and
# `river_moderate_aus` for the real thing.
#
# `simple_trees` ships with `41068_ignition_bringup` and is in git, so it is
# on every machine. It has no ponds and no river, nothing to bog down on, and
# it loads in seconds - which is what makes it the world to fall back to when
# something else is being debugged.
#
# `river_moderate_aus` is the mushroom demo world: a real river with three
# rocky crossings, no ponds, Australian flora, and the only world the
# two-class `amanita_river_v6` weights were trained against - so it is the
# only world where natives can be detected at all. It is also the world
# `forest_worlds/demo_river.sh` runs.
#
# It is NOT in git and never can be. `forest_worlds/.gitignore` ignores
# `worlds/`, `models/`, `world_assets/`, `textures/`, `assets_src/` and
# `dem/`, so what is committed is only the generator scripts. Each machine
# builds its own copy once with `forest_worlds/setup.sh` (internet, Blender
# >= 4.2, numpy, pillow); the last line of that script,
# `build_world.py --preset moderate --flora australian`, is what writes
# `forest_worlds/worlds/river_moderate_aus.sdf`. Until someone runs it the
# world simply does not appear in the dropdown, because `world_available`
# looks for the file - that is why the teammates who have it all built it
# themselves.
#
# `forest_flat`, `forest_random` and `large_demo` are deliberately not
# listed. Their metas and map drawing still work if anything asks for them
# by name; they are just off the menu.
WORLDS = {
    'simple_trees': None,
    'river_moderate_aus': 'forest_worlds',
}

# The search-pattern arguments the setup page can set, with the launch file's
# own defaults. Keeping the defaults here means the page shows what a run
# would actually do rather than the node's unused built-ins.
SEARCH_DEFAULTS = {
    'center_out': True,       # square loops out from the spawn, else rows
    'row_spacing': 3.0,       # metres between rows or loops
    'cruise_speed': 1.4,      # quad top speed on planned paths, m/s
    'cruise_alt': 1.0,        # quad height above ground, m
    'strafe': False,          # keep moving through corners
    'planner': True,          # plan around the world file, else straight legs
    'rover_speed': 1.3,       # rover Nav2 cruise speed, m/s
}

PATTERNS = (('center_out', 'Square loops outward', 'Loops out from the spawn clearing. Reaches '
             'the middle early, which is where the demo mushrooms usually are.'),
            ('rows', 'Lawnmower rows', 'Straight rows across the world. Covers evenly, and '
             'takes longest to reach the far corners.'))

# A spawned mushroom's name has to contain one of the words cage_capture_node
# looks for, or it will not recognise what it has collected.
MUSHROOM_PREFIX = 'demo_mushroom_'

# Shuffle: a mushroom is only worth placing somewhere the quad can see it and
# the rover can reach it, so it is kept clear of everything solid, out of the
# water, and away from the robots' spawn point.
#
# The rover needs a 1.7 m standoff and then creeps in, so a mushroom tucked
# against a trunk or a log is one it circles and gives up on. These margins
# are from the mushroom to the edge of the obstacle, not to its centre.
SHUFFLE_CLEAR_M = 2.5      # from any solid object's own radius outward
SHUFFLE_TRUNK_M = 2.5      # trees, when the meta gives no radius
SHUFFLE_POND_M = 1.5       # beyond the pond's shore_radius, not its waterline
SHUFFLE_RIVER_M = 2.5      # from the river's own per-sample width
SHUFFLE_APART_M = 4.0
SHUFFLE_HOME_M = 6.0
SHUFFLE_TRIES = 900        # tighter rules need more darts thrown

# How each world-file object type is treated. The radius in the meta is the
# object's own, so the clearance is added to it; a type with no radius falls
# back to a nominal one. Ponds are handled separately - they carry a
# shore_radius, and the shore is the part the rover bogs down on.
SOLID_TYPES = {
    'tree': 0.0,           # meta gives a real radius
    'fallen_tree': 0.4,    # a long one: the radius is its thickness, so pad it
    'log': 0.4,
    'stump': 0.2,
    'rock': 0.2,
    'clutter': 0.2,
    'decoy': 0.3,          # not solid, but a mushroom beside one confuses YOLO
}
# A person is a hazard, not a target, so its name must contain none of the
# words `cage_capture_node._is_mushroom` looks for ('mushroom', 'demo_m',
# 'cage_test', 'amanita') - otherwise the rover would count one as collected
# the moment it drove near enough. 'demo_person_' shares no substring with
# any of them, and the test beside this file checks that it stays that way.
PERSON_PREFIX = 'demo_person_'
PERSON_MODEL = 'walking_person'

# Where a person may stand. Looser than the mushroom rules in one way and
# much tighter in another: a person does not have to be collectable, so the
# standoff and apart distances can shrink, but one standing inside a trunk or
# in the middle of the river reads as a broken world rather than a hazard.
PERSON_CLEAR_M = 1.2       # added to a solid object's own radius
PERSON_APART_M = 3.0
# The rover spawns at (0, 0) and the hazard gate stops it at about three
# metres. A person closer than this means the rover is already stopped before
# the mission starts, which looks like the rover being broken rather than the
# gate working.
PERSON_HOME_M = 5.0

# The first person is placed deliberately, on the line from the rover's spawn
# to the nearest mushroom, so the hazard stop actually happens in a demo
# instead of only possibly happening. Not at the far end of that line: a
# person standing on the mushroom makes it uncollectable, and the run then
# shows a mushroom that is never picked up rather than a rover that stopped
# for a human.
PERSON_PATH_FRAC = 0.55    # how far along the line, before the clamps below
PERSON_PATH_KEEP_M = 2.5   # ...and never closer than this to the mushroom
PERSON_LIFT_M = 0.0        # the mesh already has its feet at z = 0

# Spawned a few centimetres high on purpose. The height estimate is good to
# about that, and the two errors are not equal: a mushroom a little above the
# ground looks fine and the detector still sees it, while one a little below
# it is invisible and the run quietly has one fewer mushroom in it than the
# counter says.
GROUND_LIFT_M = 0.06

# Where fleet_mission.launch.py puts each robot: inside the 4 m spawn
# clearing, the quads clear of the rovers' axes. Copied from its SPAWNS and
# ROVER_SPAWNS so the ready page can show you where everything starts.
QUAD_SPAWNS = [(2.0, 0.0), (-2.0, 0.0), (0.0, 2.0), (0.0, -2.0)]
ROVER_SPAWNS = [(0.0, 0.0), (2.5, -2.5), (-2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)]


def topdown_height(world):
    """How high the overhead camera sits over a world.

    A module function as well as a method because the Debug UI has no
    launcher and still has to project world coordinates into the picture -
    and it must arrive at exactly the same number, or the circles land in
    the wrong place.
    """
    size = 2.0 * max(8.0, world_meta(world)[2] + 4.5)
    return max(12.0, size / (2.0 * math.tan(TOPDOWN_HFOV / 2.0)) + 2.0)


def spawn_points(quads=1, rovers=1):
    """[(x, y, label)] for the robots a run will start with."""
    out = [(x, y, f'Q{i}') for i, (x, y) in
           enumerate(QUAD_SPAWNS[:max(1, int(quads))], 1)]
    out += [(x, y, f'S{i}') for i, (x, y) in
            enumerate(ROVER_SPAWNS[:max(1, int(rovers))], 1)]
    return out


TOPDOWN_NAME = 'ammo_topdown_cam'
TOPDOWN_TOPIC = C.TOPDOWN_TOPIC
TOPDOWN_PX = C.TOPDOWN_PX
TOPDOWN_HFOV = C.TOPDOWN_HFOV


def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        return None


def _forest_worlds_dir():
    """forest_worlds is not a ROS package; find it in the workspace's src."""
    share = _share('ammo_gui_node')
    if not share:
        return None
    ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    hits = glob.glob(os.path.join(ws, 'src', '**', 'forest_worlds', 'worlds'), recursive=True)
    return os.path.dirname(hits[0]) if hits else None


def world_path(world):
    """Path without .sdf for cage_mission's world:= (a bare name for bringup worlds)."""
    if WORLDS.get(world) == 'forest_worlds':
        base = _forest_worlds_dir()
        return os.path.join(base, 'worlds', world) if base else None
    return world


def world_available(world):
    path = world_path(world)
    if path is None:
        return False
    if os.path.isabs(path):
        return os.path.exists(path + '.sdf')
    share = _share('41068_ignition_bringup')
    return bool(share) and os.path.exists(os.path.join(share, 'worlds', world + '.sdf'))


def _meta_path(world):
    path = world_path(world)
    if path and not os.path.isabs(path):
        share = _share('41068_ignition_bringup')
        path = os.path.join(share, 'worlds', world) if share else None
    return path + '.meta.json' if path else None


_META_CACHE = {}


def world_meta_full(world):
    """The whole .meta.json for a world, or {} if it has none. Cached."""
    if world in _META_CACHE:
        return _META_CACHE[world]
    data = {}
    meta = _meta_path(world)
    if meta and os.path.exists(meta):
        try:
            with open(meta) as fh:
                loaded = json.load(fh)
            if isinstance(loaded, dict):
                data = loaded
        except Exception:
            data = {}
    _META_CACHE[world] = data
    return data


def _meta(world):
    return world_meta_full(world) or None


def world_meta(world):
    """Trunk positions, ground height and playable size (simple_trees defaults otherwise)."""
    trunks, ground_z, half = [0.0, 3.0, 5.0, 0.0], 0.0, 6.5
    data = _meta(world)
    if data:
        trunks = [v for o in data.get('objects', [])
                  if o.get('type') == 'tree' for v in (o['x'], o['y'])]
        try:
            half = data['world']['playable_size_m'] / 2.0 - 4.5
        except (KeyError, TypeError):
            pass
        ground_z = (data.get('spawn_clearing') or {}).get('ground_z', 0.0)
    return trunks, float(ground_z), float(half)


def world_mushrooms(world):
    """Mushrooms already placed in the world file, as [[x, y], ...]."""
    data = _meta(world) or {}
    return [[round(m['x'], 2), round(m['y'], 2)] for m in data.get('mushrooms', [])]


def world_mushroom_names(world):
    """The model names the world file gave its own mushrooms.

    Both forest generators name them `mushroom_00`, `mushroom_01`, ... and
    write the name into the meta, which is what lets the GUI take them out
    again when you ask for a different arrangement. A world whose meta has no
    names gives none back, and the shuffle then refuses rather than leaving a
    doubled-up set of mushrooms in the world.
    """
    data = _meta(world) or {}
    names = [m.get('name') for m in data.get('mushrooms', []) if m.get('name')]
    return [str(n) for n in names]


_GROUND_CACHE = {}


def _ground_model(world):
    """Height samples for a world, and the correction that calibrates them.

    The meta does not carry the heightmap, but it carries the z every object
    was placed at. Trees are the useful set: there are sixty-odd of them,
    spread evenly over the whole world. They are also planted slightly into
    the ground, by between 5 and 20 cm, so a tree's z is a little under the
    ground height - and that offset is what the mushrooms in the meta let us
    measure, because a mushroom sits exactly on the surface. One number per
    world, worked out once.
    """
    if world in _GROUND_CACHE:
        return _GROUND_CACHE[world]
    data = _meta(world) or {}
    trees = []
    for obj in data.get('objects', []):
        if obj.get('type') != 'tree':
            continue
        try:
            trees.append((float(obj['x']), float(obj['y']), float(obj.get('z', 0.0))))
        except (KeyError, TypeError, ValueError):
            continue
    floor = float((data.get('spawn_clearing') or {}).get('ground_z', 0.0))
    bias, residuals = 0.0, []
    for m in data.get('mushrooms', []):
        try:
            mx, my, mz = float(m['x']), float(m['y']), float(m.get('z', 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        residuals.append(mz - _interpolate(trees, mx, my, floor))
    if residuals:
        bias = sum(residuals) / len(residuals)
    model = (trees, floor, bias)
    _GROUND_CACHE[world] = model
    return model


def _interpolate(samples, x, y, floor):
    """Inverse-distance average of the three nearest samples."""
    if not samples:
        return floor
    near = sorted(samples, key=lambda s: math.hypot(x - s[0], y - s[1]))[:3]
    first = math.hypot(x - near[0][0], y - near[0][1])
    if first < 1e-3:
        return near[0][2]
    weights = [1.0 / max(1e-3, math.hypot(x - sx, y - sy)) for sx, sy, _z in near]
    return sum(w * z for w, (_sx, _sy, z) in zip(weights, near)) / sum(weights)


LFS_POINTER = b'version https://git-lfs.github.com/spec/v1'


def mesh_problem(model_sdf):
    """What is wrong with the mesh a model file draws itself with, or None.

    Worth its own check because of how this fails. Gazebo Fortress ships
    ColladaLoader, OBJLoader and STLLoader and nothing else, and a model whose
    mesh will not load still spawns perfectly well: it is a real entity,
    `ign model --list` lists it, `create` reports success, and it has no
    geometry on it. Nothing is printed. It is invisible to the operator and
    invisible to YOLO, which is a very long way to go before anyone suspects
    the mesh file.

    Two things go wrong in this repo. The file can be absent -
    `mushroom_native` has no mesh at all. Or, far more insidiously, it can be
    a **Git LFS pointer**: `41068_ignition_bringup/.gitattributes` puts
    `*.png`, `*.obj` and `*.mtl` in LFS, so on a machine without `git-lfs`
    installed a clone or pull writes a 132-byte text file that merely names
    the real asset. It has the right name and the right extension and no mesh
    loader on earth can read it.
    """
    try:
        with open(model_sdf) as fh:
            text = fh.read()
    except OSError:
        return None
    found = re.search(r'<uri>\s*(?:model://[^/]+/)?([^<]+?)\s*</uri>', text)
    if not found:
        return None
    rel = found.group(1).strip()
    path = os.path.join(os.path.dirname(model_sdf), rel)
    if not os.path.exists(path):
        return f'{rel} is not in the package'
    try:
        with open(path, 'rb') as fh:
            head = fh.read(len(LFS_POINTER))
    except OSError:
        return None
    if head == LFS_POINTER:
        return (f'{rel} is a Git LFS pointer, not a mesh - run '
                f'"git lfs install && git lfs pull" in the checkout')
    return None


def ground_z_at(world, x, y):
    """A guess at the ground height at (x, y), for spawning a mushroom on it.

    A mushroom spawned at z = 0 on a heightmap world is buried - `forest_random`
    rises over three metres - and a buried mushroom is one the quadcopter never
    finds. Accurate to a few centimetres on terrain this gentle, and exactly 0
    on a flat world, which is what the team's own launch assumes.
    """
    trees, floor, bias = _ground_model(world)
    if not trees:
        return floor
    return _interpolate(trees, x, y, floor) + bias


def trunk_pairs(flat):
    return list(zip(flat[0::2], flat[1::2]))


def _river_distance(meta, x, y):
    """Metres from the river centre line, or None on a world without one."""
    river = (meta or {}).get('river') or {}
    line = river.get('line') or []
    if not line:
        return None
    best = None
    for sample in line:
        try:
            dist = math.hypot(x - float(sample[0]), y - float(sample[1])) - float(sample[2])
        except (TypeError, ValueError, IndexError):
            continue
        if best is None or dist < best:
            best = dist
    return best


def _keep_out(world):
    """[(x, y, radius)] a mushroom must stay outside, from the world file.

    One list for every solid thing plus the ponds, so the shuffle makes a
    single pass instead of one test per object type - and so a world whose
    meta only lists trees (the river builder) still gets sensible answers.
    """
    data = _meta(world) or {}
    out = []
    for obj in data.get('objects', []):
        kind = obj.get('type')
        if kind == 'pond':
            continue                       # handled below, with its shore
        if kind not in SOLID_TYPES:
            continue
        try:
            x, y = float(obj['x']), float(obj['y'])
        except (KeyError, TypeError, ValueError):
            continue
        radius = obj.get('radius')
        radius = float(radius) if radius else (SHUFFLE_TRUNK_M if kind == 'tree' else 0.3)
        out.append((x, y, radius + SOLID_TYPES[kind] + SHUFFLE_CLEAR_M))
    for pond in list(data.get('ponds', [])) + [o for o in data.get('objects', [])
                                               if o.get('type') == 'pond']:
        try:
            x, y = float(pond['x']), float(pond['y'])
        except (KeyError, TypeError, ValueError):
            continue
        # shore_radius is where the bank ends; the waterline (radius) is well
        # inside it, and the few metres between the two are mud.
        edge = float(pond.get('shore_radius') or pond.get('radius') or 3.0)
        out.append((x, y, edge + SHUFFLE_POND_M))
    return out


def shuffle_mushrooms(world, count, seed=None):
    """Random positions a mushroom could plausibly be found and collected at.

    Clear of every solid object the world file knows about, outside the pond
    shores and the river, apart from each other and away from the spawn
    clearing, inside the same playable area the quad searches.

    Returns fewer than asked rather than looping forever - a dense world
    genuinely may not have room for eight well-spaced mushrooms, and the
    caller says so rather than pretending.
    """
    meta = world_meta_full(world)
    avoid = _keep_out(world)
    if not avoid:
        # a meta with no objects at all: fall back to the trunk list
        avoid = [(tx, ty, SHUFFLE_TRUNK_M)
                 for tx, ty in trunk_pairs(world_meta(world)[0])]
    half = max(4.0, world_meta(world)[2])
    rng = random.Random(seed)
    picked = []
    for _ in range(SHUFFLE_TRIES):
        if len(picked) >= count:
            break
        x = rng.uniform(-half, half)
        y = rng.uniform(-half, half)
        if math.hypot(x, y) < SHUFFLE_HOME_M:
            continue
        if any(math.hypot(x - ax, y - ay) < r for ax, ay, r in avoid):
            continue
        if any(math.hypot(x - px, y - py) < SHUFFLE_APART_M for px, py in picked):
            continue
        river = _river_distance(meta, x, y)
        if river is not None and river < SHUFFLE_RIVER_M:
            continue
        picked.append((round(x, 2), round(y, 2)))
    return picked


def _person_ok(x, y, avoid, picked, meta, half, mushrooms=()):
    """Is (x, y) somewhere a person could plausibly be standing?"""
    if abs(x) > half or abs(y) > half:
        return False
    if math.hypot(x, y) < PERSON_HOME_M:
        return False
    # Not on top of a mushroom. The deliberately-placed first person is kept
    # clear of the one they are standing in front of; without this the
    # scattered ones had no rule about mushrooms at all, and a person standing
    # on one makes it uncollectable - which reads as the gripper being broken
    # rather than as the hazard stop working.
    if any(math.hypot(x - float(m[0]), y - float(m[1])) < PERSON_PATH_KEEP_M
           for m in (mushrooms or ()) if m is not None and len(m) >= 2):
        return False
    for ax, ay, r in avoid:
        # `avoid` carries the mushroom clearance baked in; a person only needs
        # to be out of the object itself, so the mushroom margin comes back off.
        if math.hypot(x - ax, y - ay) < max(0.6, r - SHUFFLE_CLEAR_M + PERSON_CLEAR_M):
            return False
    if any(math.hypot(x - px, y - py) < PERSON_APART_M for px, py in picked):
        return False
    river = _river_distance(meta, x, y)
    return not (river is not None and river < PERSON_CLEAR_M)


def path_person(mushrooms, avoid, meta, half, rover=None):
    """A spot on the rover's way to its nearest mushroom, or None.

    Walks back along the line from the mushroom toward the rover rather than
    giving up at the first unusable point: on a crowded world the midpoint is
    often inside a trunk's clearance, and somewhere else on the same line is
    still on the rover's way.
    """
    # Rover 1's spawn, from the launch file's own list rather than a literal,
    # so moving the spawn moves this with it.
    rover = tuple(rover if rover is not None else ROVER_SPAWNS[0])
    best = None
    for point in (mushrooms or []):
        try:
            mx, my = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        d = math.hypot(mx - rover[0], my - rover[1])
        if best is None or d < best[0]:
            best = (d, mx, my)
    if best is None or best[0] < PERSON_HOME_M + PERSON_PATH_KEEP_M:
        return None       # nothing far enough away to stand in front of
    span, mx, my = best
    # The usable window along the line, as a fraction of its length.
    low = PERSON_HOME_M / span
    high = (span - PERSON_PATH_KEEP_M) / span
    if high <= low:
        return None
    # Sweep outward from the preferred fraction, alternating back and forth,
    # so the spot chosen is the closest usable one to the middle of the leg
    # rather than whichever end happened to be tried first.
    for step in range(26):
        for f in ({PERSON_PATH_FRAC} if step == 0 else
                  (PERSON_PATH_FRAC - step * 0.02, PERSON_PATH_FRAC + step * 0.02)):
            if not low <= f <= high:
                continue
            x = rover[0] + (mx - rover[0]) * f
            y = rover[1] + (my - rover[1]) * f
            # Every mushroom, not just the one at the end of this leg: the
            # line to the nearest one can still pass over another.
            if _person_ok(x, y, avoid, [], meta, half, mushrooms):
                return (round(x, 2), round(y, 2))
    return None


def place_people(world, count, mushrooms=None, seed=None):
    """Where to stand `count` people in this world.

    The first goes in the rover's way, on the line to its nearest mushroom,
    so the hazard stop is part of the run rather than a lucky accident. The
    rest are scattered under the same rules the mushroom shuffle uses, minus
    the collectability margins a person does not need.

    Returns fewer than asked rather than looping, exactly like
    `shuffle_mushrooms`, and the caller says so.
    """
    count = max(0, int(count or 0))
    if count <= 0:
        return []
    meta = world_meta_full(world)
    avoid = _keep_out(world)
    if not avoid:
        avoid = [(tx, ty, SHUFFLE_TRUNK_M)
                 for tx, ty in trunk_pairs(world_meta(world)[0])]
    half = max(4.0, world_meta(world)[2])
    picked = []
    first = path_person(mushrooms, avoid, meta, half)
    if first is not None:
        picked.append(first)
    rng = random.Random(seed)
    for _ in range(SHUFFLE_TRIES):
        if len(picked) >= count:
            break
        x = rng.uniform(-half, half)
        y = rng.uniform(-half, half)
        if _person_ok(x, y, avoid, picked, meta, half, mushrooms):
            picked.append((round(x, 2), round(y, 2)))
    return picked[:count]


def sim_already_running():
    out = subprocess.run(['pgrep', '-f', r'^(ruby [^ ]+/)?(ign gazebo|gz sim) '],
                         capture_output=True, text=True)
    return bool(out.stdout.strip())


def _as_launch_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


# `ros2 run ros_ign_gazebo create` builds the spawn pose from its own
# -x/-y/-z/-R/-P/-Y flags and IGNORES the <pose> in the model file. Leave the
# flags off and the model lands at the world origin with no rotation - which
# is why the first version of this camera sat on the ground in the spawn
# clearing looking sideways into the trees, and produced a picture that
# looked exactly like the skidsteer's own camera.
TOPDOWN_PITCH = 1.5707963      # a quarter turn about Y: +x forward -> -z down


def topdown_sdf(px=TOPDOWN_PX, hfov=TOPDOWN_HFOV, far=200.0):
    """A camera looking straight down, as a standalone model.

    The pose is deliberately NOT in here - see the note above `TOPDOWN_PITCH`.
    It is passed on the create command line by `start_topdown`.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{TOPDOWN_NAME}">
    <static>true</static>
    <link name="link">
      <sensor name="topdown" type="camera">
        <always_on>1</always_on>
        <update_rate>2</update_rate>
        <topic>{TOPDOWN_TOPIC}</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{px}</width><height>{px}</height><format>R8G8B8</format></image>
          <clip><near>0.5</near><far>{far:.1f}</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


class MissionLauncher:
    """Owns the simulation and the two mission node processes."""

    def __init__(self, logger=None):
        self.proc = None
        self.log = logger or (lambda _m: None)
        self.log_path = os.path.join(os.path.expanduser('~'), '.ros', 'ammo_gui_mission.log')
        self.state = IDLE
        self.world = None
        self.show_sim = False
        self.options = {}
        self.started_at = 0.0
        self.spawned = []          # [(name, x, y)] mushrooms we put there
        self.people = []           # [(name, x, y)] people we put there
        self.cleared_world = False  # the world's own mushrooms have been removed
        self.held = False          # the two mission nodes are frozen right now
        self.quads, self.rovers = 1, 1
        self._reported_death = False
        self._env = None           # the launch's environment, for short commands
        self.last_error = ''       # stderr of the last short command
        self._topdown_proc = None

    # -- stage one: prepare -------------------------------------------------
    def prepare(self, world, show_sim=True, options=None, tool='gripper',
                quads=1, rovers=1, people=0):
        """Bring the simulation up, or reuse the one already up.

        With one of each robot this runs the team's `cage_mission.launch.py`
        exactly as before. With more it runs `quad_zcams/fleet_mission.launch.py`
        instead, which is their own multi-robot path: up to four quads
        (`parrot1..4`) and five rovers (rover 1 as `husky1`, rover k under
        `rk`), with `fleet_coordinator` handing mushrooms out between the
        rovers. The GUI does not spawn a second robot itself - that needs a
        namespace, a rewritten bridge config, its own TF tree and a split
        search area, all of which their launch file already does.

        Returns 'started', 'reused', or None if it could not begin.
        """
        quads = max(1, min(int(quads), C.MAX_QUADS))
        rovers = max(1, min(int(rovers), C.MAX_ROVERS))
        if (self.state in (READY, RUNNING) and self.world == world
                and (self.quads, self.rovers) == (quads, rovers)):
            return 'reused'
        if self.state in (PREPARING, READY, RUNNING):
            self.shutdown()        # a different world needs a different Gazebo
        if sim_already_running():
            self.log('BLOCKED: a simulation is already running. The GUI starts its '
                     'own, so it will not start a second. Use "Stop what is '
                     'running" below, or close the other Gazebo.')
            return 'blocked'
        path = world_path(world)
        if not world_available(world):
            self.log(f'world {world} not found - for river worlds run forest_worlds/setup.sh')
            return None

        env = dict(os.environ)
        # Kept so `create`, `remove` and `ign model --list` run against the
        # same resource path as the server they are talking to.
        self._env = env
        # NOT SET ANY MORE. This used to point the three mission nodes at
        # `config/fastdds_udp_only.xml`, because the GUI held them still by
        # SIGSTOPping their processes and a process frozen past Fast DDS's
        # 20 s lease was dropped by everything else and not reliably
        # re-matched. The profile bought a one-hour lease at the price of
        # loopback UDP only, with shared memory and data-sharing both off.
        #
        # Nothing is SIGSTOPped any more - robots are held with the
        # `teleop_active` heartbeat instead - so the price was being paid for
        # nothing, and it is not a small price. `collision_guard` subscribes
        # to husky1/camera/depth/image: 720 x 480 float32 is 1.4 MB a frame,
        # which over shared memory is a pointer and over UDP is about a
        # thousand datagrams that the default kernel socket buffers drop. A
        # guard that is not reliably receiving depth is a guard whose
        # forward-motion checks come and go. It also left the mission nodes on
        # a different transport from Nav2, the Gazebo bridge and the
        # detectors, which is a strange place to be for no benefit.
        #
        # To put it back - if the SIGSTOP hold is ever revived - this one line
        # is all it takes; `cage_mission.launch.py` and `zcam_mission.launch.py`
        # both already read the variable and do nothing when it is absent:
        #   env['AMMO_FREEZABLE_PROFILE'] = os.path.join(
        #       _share('ammo_gui_node') or '', 'config', 'fastdds_udp_only.xml')
        fleet = quads > 1 or rovers > 1
        if fleet and int(people or 0) > 0:
            # fleet_mission.launch.py has no `hazard` argument, so there is
            # nowhere to turn the person detector on for its rovers. The
            # people still go into the world - they are just scenery on a
            # fleet run, and saying so beats a hazard stop that never comes.
            self.log('people will be placed, but the person detector only runs '
                     'on a one-of-each run - the fleet launch file has no '
                     'hazard option, so nothing will stop for them')
        if fleet and show_sim:
            # Two quads means two YOLO instances and two depth cameras on top
            # of everything else; adding Gazebo's own renderer to that is what
            # left a world that never finished loading.
            show_sim = False
            self.log('a fleet runs headless - the Gazebo window on top of two '
                     'detectors is more than this machine will carry. Every '
                     'camera is in the GUI anyway.')
        if fleet:
            cmd = ['ros2', 'launch', 'quad_zcams', 'fleet_mission.launch.py',
                   f'world:={path}', f'quads:={quads}', f'rovers:={rovers}',
                   'meta_goals:=false', 'rviz:=false',
                   f'headless:={"false" if show_sim else "true"}']
        else:
            cmd = ['ros2', 'launch', 'ammo_mission_control', 'cage_mission.launch.py',
                   f'world:={path}', f'tool:={tool}', 'meta_goals:=false', 'rviz:=false',
                   # cage_mission defaults husky_yolo to false, which leaves the
                   # rover with no detector and therefore no boxes on its feed.
                   'husky_yolo:=true',
                   # The COCO person detector on the rover, and the hazard
                   # brake that acts on it. Off unless this run has people in
                   # it: it is a second inference pass per frame, and on a
                   # four-core machine that is not free.
                   f'hazard:={"true" if int(people or 0) > 0 else "false"}',
                   f'headless:={"false" if show_sim else "true"}']
            for key, default in SEARCH_DEFAULTS.items():
                cmd.append(f'{key}:={_as_launch_value((options or {}).get(key, default))}')
        # The river worlds need their models on the resource path before
        # Gazebo starts or nothing spawns at all and the sim looks frozen -
        # their own README is emphatic about this, and demo_river.sh does the
        # same export. The fleet loads the two-class detector itself; the
        # single-quad launch defaults to a one-class model that sees no
        # natives, so it is passed explicitly.
        # Every world needs the bringup models on the path, not just the river
        # ones. `mushroom_amanita` and `mushroom_native` live there, the river
        # world <include>s both by `model://` URI, and the model file the GUI
        # spawns refers to its own mesh the same way - so the server has to be
        # able to resolve them or they load as entities with no geometry.
        # Nothing else in the repo sets this; `models/mushroom_amanita/
        # README_MUSHROOM.md` says as much and tells you to export it by hand,
        # which is not something an operator should have to know.
        extra = []
        bringup = _share('41068_ignition_bringup')
        if bringup:
            extra.append(os.path.join(bringup, 'models'))
        if WORLDS.get(world) == 'forest_worlds':
            base = _forest_worlds_dir()
            extra += glob.glob(os.path.join(base, 'models', '*')) + [os.path.join(base, 'world_assets')]
        if extra:
            env['IGN_GAZEBO_RESOURCE_PATH'] = ':'.join(
                extra + [env.get('IGN_GAZEBO_RESOURCE_PATH', '')]).strip(':')
            # Garden and later read a different name; setting both costs
            # nothing and saves a confusing afternoon if anyone upgrades.
            env['GZ_SIM_RESOURCE_PATH'] = env['IGN_GAZEBO_RESOURCE_PATH']
        if WORLDS.get(world) == 'forest_worlds':
            if not fleet:
                model = os.path.join(_share('perception') or '', 'models', 'variants',
                                     'amanita_river_v6_yolo11n.pt')
                if os.path.exists(model):
                    cmd.append(f'model_path:={model}')
        # Mushrooms are NOT passed here any more. The GUI spawns them after the
        # mission nodes are frozen, so they can be shuffled and looked at
        # without the quadcopter ever having had a chance to see them.
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            with open(self.log_path, 'w') as log:
                self.proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                             stdin=subprocess.DEVNULL, start_new_session=True)
        except FileNotFoundError:
            self.log('could not start the mission: is the workspace sourced?')
            return None
        self.state = PREPARING
        self.world = world
        self.quads, self.rovers = quads, rovers
        self.show_sim = bool(show_sim)
        self.options = dict(options or {})
        self.started_at = time.monotonic()
        self._reported_death = False
        self.spawned = []
        self.people = []
        self.cleared_world = False
        self.held = False
        what = (f'{quads} quadcopter(s) and {rovers} skidsteer(s)' if fleet
                else 'one of each robot')
        self.log(f'preparing {world} with {what} - Gazebo and Nav2 take about '
                 f'30 s, longer for a fleet (log: {self.log_path})')
        return 'started'

    def hold_nodes(self):
        """Freeze the mission node processes outright. The last resort.

        Prepare, Pause, Abort and hand-driving all use the robots' own
        `teleop_active` stand-down instead, which leaves the nodes running
        and their clocks and Nav2 connections intact. This is what is left
        for the case where a node stops answering the heartbeat at all.

        The caller must also publish zeros. Gazebo's VelocityControl keeps
        the last command it was given forever, so a node frozen one second
        into a take-off leaves a quadcopter still climbing.

        Returns True only on the call that actually froze them.
        """
        if self.held or self.state not in (PREPARING, READY):
            return False
        self.suspend(MISSION_NODES)
        self.held = True
        return True

    def mark_ready(self):
        """Everything has arrived; hold here until the operator launches.

        This no longer stops any process. The robots are held still by their
        own `teleop_active` stand-down, published by the window - see
        `contract.PARROT_TELEOP`. A standing-down node keeps its timers, its
        clock and its Nav2 connection, re-plans from where it was left, and
        goes on reporting sightings; a stopped one comes back with all of
        those stale. `suspend`/`resume` are still here as a last resort.

        Also accepted from READY, which is the state a finished mission
        leaves behind, so preparing the same world again just re-holds.
        """
        if self.state not in (PREPARING, READY):
            return False
        self.state = READY
        self.log('ready - the simulation is up and both robots are held')
        return True

    # -- stage two: run -----------------------------------------------------
    def launch(self):
        """Start the mission. The window releases the stand-down itself."""
        if self.state != READY:
            return False
        self.resume(MISSION_NODES)   # no-op unless something was suspended
        self.held = False
        self.state = RUNNING
        return True

    def end_mission(self, keep_sim=False):
        """Stop the mission, and by default the simulation with it.

        Keeping Gazebo up between runs is faster, and it is what this did
        first - but a simulation that outlives its mission turned out to
        cause most of the trouble worth having: its camera topics keep
        feeding panels for robots that are gone, its latched occupancy grid
        reads as a live coverage number that never moves, and the next
        Prepare refuses to start because something is already running. So
        the default is now to stop everything, and `keep_sim=True` is the
        opt-in.
        """
        if self.state == IDLE:
            return
        if not keep_sim:
            self.shutdown()
            return
        # Not suspended: the window holds both robots with their own
        # stand-down, which leaves the nodes healthy for the next run.
        self.remove_mushrooms()
        self.remove_people()
        self.state = READY if self.proc and self.proc.poll() is None else IDLE
        self.log('mission ended - the simulation is still up')

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def prepared(self):
        return self.state in (READY, RUNNING) and self.running()

    # -- mushrooms ----------------------------------------------------------
    def spawn_mushrooms(self, points):
        """Put the chosen mushrooms into the running world.

        Called after the mission nodes are frozen, so the quadcopter has not
        had a chance to see any of them - which is what makes the top-down
        photo an honest picture of what the run starts from.
        """
        self.remove_mushrooms()
        share = _share('41068_ignition_bringup')
        if not share:
            self.log('cannot find 41068_ignition_bringup to spawn mushrooms')
            return []
        sdf = os.path.join(share, 'models', 'mushroom_amanita', 'model.sdf')
        if not os.path.exists(sdf):
            self.log(f'the mushroom model is missing at {sdf} - nothing to spawn')
            return []
        problem = mesh_problem(sdf)
        if problem:
            self.log(f'WARNING: {problem}. The mushrooms will spawn as entities '
                     f'with no geometry - invisible in Gazebo, invisible to '
                     f'YOLO, and no error anywhere. Fix that before blaming '
                     f'the detector.')
        errors = []
        for i, (x, y) in enumerate(points, 1):
            name = f'{MUSHROOM_PREFIX}{i}'
            z = ground_z_at(self.world, float(x), float(y)) + GROUND_LIFT_M
            self._run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', sdf,
                       '-name', name, '-x', str(float(x)), '-y', str(float(y)),
                       '-z', f'{z:.3f}'], 20)
            if self.last_error:
                errors.append(f'{name}: {self.last_error.splitlines()[-1][:120]}')
            self.spawned.append((name, float(x), float(y)))
        if self.spawned:
            there = self.confirm_mushrooms()
            if there is None:
                self.log(f'{len(self.spawned)} mushroom(s) asked for, but Gazebo '
                         f'would not say what it has - `ign model --list` gave '
                         f'nothing, so treat the count as unconfirmed')
            elif there == 0:
                self.log(f'NONE of the {len(self.spawned)} mushrooms are in the '
                         f'world. The spawn command ran and Gazebo has no '
                         f'model named {MUSHROOM_PREFIX}*. Model file: {sdf}')
            elif there < len(self.spawned):
                self.log(f'only {there} of {len(self.spawned)} mushrooms are '
                         f'actually in the world - the rest did not spawn')
            else:
                self.log(f'{there} mushroom(s) placed and confirmed in Gazebo')
            for line in errors[:3]:
                self.log(f'spawn error - {line}')
        return [(x, y) for _n, x, y in self.spawned]

    def confirm_mushrooms(self):
        """How many of ours Gazebo says it actually has, or None if unknown.

        `create` reports success whether or not the model file resolved, so
        "we ran the command five times" is not the same as "there are five
        mushrooms in the world" - which is exactly the doubt worth removing.
        """
        out = self._run(['ign', 'model', '--list'], 8)
        if not out:
            return None
        return sum(1 for line in out.splitlines()
                   if MUSHROOM_PREFIX in line)

    def remove_mushrooms(self):
        """Take out the mushrooms the GUI put there. Leaves the world's own."""
        for name, _x, _y in self.spawned:
            self._remove_model(name)
        self.spawned = []

    # -- people -------------------------------------------------------------
    # People are hazards: the rover's COCO detector identifies them and the
    # hazard brake stops it driving into one. They are spawned exactly like
    # the mushrooms - after the mission nodes are held, so nothing has seen
    # them before the run starts - and removed when the mission ends.
    #
    # `walking_person` is static in its own model file, so nobody walks
    # anywhere. That is on purpose: a person who moves makes "did the rover
    # stop for them, or did they step aside" unanswerable from a recording.
    def spawn_people(self, points):
        """Put people into the running world. Returns the (x, y) that took."""
        self.remove_people()
        if not points:
            return []
        share = _share('41068_ignition_bringup')
        if not share:
            self.log('cannot find 41068_ignition_bringup to spawn people')
            return []
        sdf = os.path.join(share, 'models', PERSON_MODEL, 'model.sdf')
        if not os.path.exists(sdf):
            self.log(f'the person model is missing at {sdf} - nothing to spawn. '
                     f'It is part of 41068_ignition_bringup; a colcon build of '
                     f'that package installs it.')
            return []
        problem = mesh_problem(sdf)
        if problem:
            self.log(f'WARNING: {problem}. The people will spawn with no '
                     f'geometry - invisible in Gazebo and invisible to the '
                     f'detector, with no error anywhere. `git lfs pull` and '
                     f'rebuild before concluding the hazard stop is broken.')
        errors = []
        for i, (x, y) in enumerate(points, 1):
            name = f'{PERSON_PREFIX}{i}'
            z = ground_z_at(self.world, float(x), float(y)) + PERSON_LIFT_M
            self._run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', sdf,
                       '-name', name, '-x', str(float(x)), '-y', str(float(y)),
                       '-z', f'{z:.3f}',
                       # A person is recognisable from the front and oddly hard
                       # to call from directly behind, so they are turned to
                       # face the middle of the world - which is where the
                       # rover is coming from.
                       '-Y', f'{math.atan2(-float(y), -float(x)):.3f}'], 20)
            if self.last_error:
                errors.append(f'{name}: {self.last_error.splitlines()[-1][:120]}')
            self.people.append((name, float(x), float(y)))
        there = self.confirm_people()
        if there is None:
            self.log(f'{len(self.people)} person(s) asked for, but Gazebo would '
                     f'not say what it has - treat the count as unconfirmed')
        elif there == 0:
            self.log(f'NONE of the {len(self.people)} people are in the world. '
                     f'The spawn ran and Gazebo has no model named '
                     f'{PERSON_PREFIX}*. Model file: {sdf}')
        elif there < len(self.people):
            self.log(f'only {there} of {len(self.people)} people are actually '
                     f'in the world - the rest did not spawn')
        else:
            self.log(f'{there} person(s) placed and confirmed in Gazebo')
        for line in errors[:3]:
            self.log(f'person spawn error - {line}')
        return [(x, y) for _n, x, y in self.people]

    def confirm_people(self):
        """How many of ours Gazebo says it has, or None if it would not say."""
        out = self._run(['ign', 'model', '--list'], 8)
        if not out:
            return None
        return sum(1 for line in out.splitlines() if PERSON_PREFIX in line)

    def remove_people(self):
        for name, _x, _y in self.people:
            self._remove_model(name)
        self.people = []

    def clear_world_mushrooms(self):
        """Take out the mushrooms the world file came with.

        Only used when the operator asks for a different arrangement on a
        world that ships its own: without this, a shuffle would add a second
        set on top of the first. Done once per simulation - the models do not
        come back until Gazebo is restarted.
        """
        if self.cleared_world or not self.world:
            return True
        names = world_mushroom_names(self.world)
        if not names:
            return False
        for name in names:
            self._remove_model(name)
        self.cleared_world = True
        self.log(f"removed the world's own {len(names)} mushroom(s)")
        return True

    def _remove_model(self, name):
        if not self.world:
            return
        self._run(['ign', 'service', '-s', f'/world/{os.path.basename(self.world)}/remove',
                   '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                   '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'], 6)

    def _run(self, args, timeout):
        """Run one short command and give back its stdout.

        It inherits the launch's own environment when there is one, so a
        `create` call resolves models against the same resource path Gazebo
        was started with rather than the GUI's bare environment. stderr is
        kept in `self.last_error` - `create` reports success on stdout
        whether or not the model file resolved, so the only account of what
        went wrong is on the other stream.
        """
        self.last_error = ''
        try:
            done = subprocess.run(args, capture_output=True, text=True,
                                  timeout=timeout, env=self._env or None)
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as exc:
            self.last_error = str(exc)
            return ''
        self.last_error = (done.stderr or '').strip()
        return done.stdout

    # -- the top-down camera --------------------------------------------------
    def topdown_height(self):
        return topdown_height(self.world)

    def start_topdown(self):
        """Spawn a downward camera over the world and bridge its image topic.

        Stays up for the whole run now, not just long enough for one frame:
        the operator page and the Debug UI both show it as a live feed. It is
        a third rendered camera, so `stop_topdown` is worth calling when the
        feed is switched off.

        Returns False if it could not be started at all. On a machine without
        hardware OpenGL frames can take several seconds or never arrive,
        which is why the caller times out and falls back to the drawn map.
        """
        if self.state not in (READY, RUNNING) or not self.world:
            return False
        if self._topdown_proc is not None and self._topdown_proc.poll() is None:
            return True                      # already up
        height = self.topdown_height()
        path = os.path.join(os.path.dirname(self.log_path), 'ammo_topdown.sdf')
        try:
            with open(path, 'w') as fh:
                fh.write(topdown_sdf(far=max(50.0, height * 3.0)))
        except OSError:
            return False
        # The pose goes HERE, not in the SDF. `create` ignores the model's own
        # <pose> and uses these flags, defaulting every one it is not given
        # to zero - which put the camera on the ground looking sideways.
        self._run(['ros2', 'run', 'ros_ign_gazebo', 'create',
                   '-file', path, '-name', TOPDOWN_NAME,
                   '-x', '0', '-y', '0', '-z', f'{height:.2f}',
                   '-R', '0', '-P', f'{TOPDOWN_PITCH:.7f}', '-Y', '0'], 25)
        try:
            self._topdown_proc = subprocess.Popen(
                ['ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                 f'{TOPDOWN_TOPIC}@sensor_msgs/msg/Image[ignition.msgs.Image'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL, start_new_session=True)
        except (FileNotFoundError, OSError):
            self._remove_model(TOPDOWN_NAME)
            return False
        return True

    def kill_strays(self):
        """Kill a simulation this GUI does not own.

        A launch that was interrupted leaves Gazebo, the bridges and the
        mission nodes running, and they keep publishing: stale camera topics
        feed panels for robots that are not there any more, and `prepare`
        refuses to start a second simulation. Nothing in the GUI can reach
        them - they are not in our process group - so this is the blunt
        instrument, and it is why it asks first.

        Returns the number of processes it signalled.
        """
        patterns = [r'ign gazebo', r'gz sim', r'ruby .*ign gazebo',
                    'ign-gazebo-server', 'ign-gazebo-gui',
                    'cage_mission.launch.py', 'fleet_mission.launch.py',
                    'zcam_mission.launch.py', 'parameter_bridge', 'yolo_node',
                    'fleet_yolo_pool', 'fleet_coordinator']
        killed = 0
        for sig in ('-TERM', '-KILL'):
            for pattern in patterns:
                out = subprocess.run(['pkill', sig, '-f', pattern],
                                     capture_output=True)
                if out.returncode == 0:
                    killed += 1
            killed += self._kill_ign_ruby()
        return killed

    def _kill_ign_ruby(self):
        """Kill Ignition's own server and GUI, found the way they really look.

        `ign gazebo` is a ruby script, and on Fortress the server and the GUI
        end up as separate ruby processes whose command lines do not reliably
        contain the string "ign gazebo" - which is why pattern matching alone
        left a Gazebo window that would not close and sometimes needed a
        reboot. `forest_worlds/view.sh` has always done it this way instead,
        and that one works: look at every ruby process and read its command
        line. Matching on the ruby name as well as "ign" is what keeps this
        from ever touching an unrelated ruby program.
        """
        killed = 0
        try:
            pids = subprocess.run(['pgrep', '-x', 'ruby'], capture_output=True,
                                  text=True, timeout=5).stdout.split()
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
            return 0
        for pid in pids:
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                    cmdline = fh.read().decode('utf-8', 'replace')
            except (OSError, ValueError):
                continue
            if 'ign' not in cmdline and 'gz' not in cmdline:
                continue
            try:
                os.kill(int(pid), signal.SIGKILL)
                killed += 1
            except (ProcessLookupError, PermissionError, ValueError, OSError):
                continue
        return killed

    def topdown_running(self):
        return self._topdown_proc is not None and self._topdown_proc.poll() is None

    def stop_topdown(self):
        if self._topdown_proc is not None:
            try:
                os.killpg(self._topdown_proc.pid, signal.SIGINT)
            except (ProcessLookupError, PermissionError, OSError):
                pass
            self._topdown_proc = None
        self._remove_model(TOPDOWN_NAME)

    # -- health -------------------------------------------------------------
    def health_warning(self):
        """A sentence to show the operator if the launch died on its own."""
        if self.proc is None or self._reported_death:
            return None
        if self.proc.poll() is None:
            return None
        self._reported_death = True
        self.state = IDLE
        if time.monotonic() - self.started_at > SIM_WARMUP_S:
            return 'The mission launch exited. See ~/.ros/ammo_gui_mission.log.'
        tail = self._log_tail()
        if 'Ogre' in tail or 'GL3Plus' in tail or 'UnimplementedException' in tail:
            if self.show_sim:
                return ('Gazebo could not open its window (an Ogre/OpenGL error). '
                        'Untick "Show the Gazebo window" and prepare again, or run '
                        'with LIBGL_ALWAYS_SOFTWARE=1.')
            return ('Gazebo failed to start with an Ogre/OpenGL error - try '
                    'LIBGL_ALWAYS_SOFTWARE=1 in this terminal.')
        if 'not found' in tail or 'Package' in tail:
            return ('The launch could not find a package - is the workspace built '
                    'and sourced in this terminal?')
        return ('The mission launch exited after a few seconds. The last lines of '
                f'{self.log_path} will say why.')

    def _log_tail(self, limit=4000):
        try:
            with open(self.log_path, 'rb') as fh:
                try:
                    fh.seek(-limit, os.SEEK_END)
                except OSError:
                    fh.seek(0)
                return fh.read().decode('utf-8', 'replace')
        except OSError:
            return ''

    # -- suspend / resume ---------------------------------------------------
    def _pids(self, names):
        """PIDs in our launch's session whose command runs one of `names`."""
        if not self.proc or self.proc.poll() is not None:
            return []
        out = subprocess.run(['ps', '-o', 'pid=,args=', '-s', str(self.proc.pid)],
                             capture_output=True, text=True).stdout
        pids = []
        for line in out.splitlines():
            pid, _, args = line.strip().partition(' ')
            # python nodes run as `python3 .../lib/<pkg>/<exe> --ros-args ...`
            if any(os.path.basename(tok) in names for tok in args.split()[:2]):
                pids.append(int(pid))
        return pids

    def _signal(self, names, sig):
        for pid in self._pids(names):
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass

    def suspend(self, names=MISSION_NODES):
        self._signal(names, signal.SIGSTOP)

    def resume(self, names=MISSION_NODES):
        self._signal(names, signal.SIGCONT)

    def mission_nodes_up(self):
        """Both mission node processes exist - they have at least started."""
        running = {os.path.basename(n) for n in self._node_names()}
        return (any(n in running for n in ROVER_NODES)
                and any(n in running for n in QUAD_NODES))

    def _node_names(self):
        if not self.proc or self.proc.poll() is not None:
            return []
        out = subprocess.run(['ps', '-o', 'args=', '-s', str(self.proc.pid)],
                             capture_output=True, text=True).stdout
        names = []
        for line in out.splitlines():
            for tok in line.strip().split()[:2]:
                base = os.path.basename(tok)
                if base in MISSION_NODES + GUARD_NODES:
                    names.append(base)
        return names

    # -- teardown -----------------------------------------------------------
    def shutdown(self):
        """End everything: mission nodes, Nav2 and Gazebo."""
        self.stop_topdown()
        if not self.proc:
            # Nothing of ours is running, but something of somebody's might
            # be - an interrupted launch, or a Gazebo that outlived its
            # group. The setup page is supposed to mean nothing is running,
            # so sweep anyway rather than returning on a technicality.
            self.kill_strays()
            self.state = IDLE
            return
        self.resume()
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            pass
        except (ProcessLookupError, PermissionError, OSError):
            pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        # Killing the process group is not always enough. Ignition's own
        # launcher re-parents the server and the GUI, so they survive their
        # group and go on publishing - which is how a closed Gazebo window
        # still fed camera feeds, and why it sometimes took a reboot. These
        # are our own run's processes; `kill_strays` is the same thing for
        # a run nobody owns.
        self.kill_strays()
        self.proc = None
        self.spawned = []
        self.people = []
        self.cleared_world = False
        self.held = False
        self.state = IDLE
        self.log('mission and simulation stopped')

    # kept so older wiring keeps working
    def stop_mission(self):
        self.shutdown()

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
import random
import signal
import subprocess
import time

from ammo_gui_node import contract as C

# executables inside the launch, by role
ROVER_NODES = ('rover_capture_node', 'cage_capture_node')
GUARD_NODES = ('collision_guard_node',)
QUAD_NODES = ('zcam_quad_node', 'quad_search_node')
MISSION_NODES = ROVER_NODES + QUAD_NODES          # the guard stays awake
STOP_TIMEOUT_S = 20.0
SIM_WARMUP_S = 45.0       # below this, an exit means it failed rather than finished

# states
IDLE = 'idle'             # nothing running
PREPARING = 'preparing'   # the launch is up but not everything has arrived
READY = 'ready'           # everything is up and the mission nodes are frozen
RUNNING = 'running'       # the mission nodes are thawed and the clock is going

# name -> folder holding <name>.sdf (None = 41068_ignition_bringup/worlds)
WORLDS = {
    'simple_trees': None,
    'forest_flat': None,
    'forest_random': None,
    'large_demo': None,
    'river_moderate_aus': 'forest_worlds',
    'river_moderate_aus_green': 'forest_worlds',
    'river_moderate_green': 'forest_worlds',
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

# Shuffle: a mushroom has to be reachable and findable, so keep it clear of
# trunks and water and away from the robots' spawn point.
SHUFFLE_TRUNK_M = 1.8
SHUFFLE_RIVER_M = 2.5
SHUFFLE_APART_M = 4.0
SHUFFLE_HOME_M = 6.0
SHUFFLE_TRIES = 400
# Spawned a few centimetres high on purpose. The height estimate is good to
# about that, and the two errors are not equal: a mushroom a little above the
# ground looks fine and the detector still sees it, while one a little below
# it is invisible and the run quietly has one fewer mushroom in it than the
# counter says.
GROUND_LIFT_M = 0.06

TOPDOWN_NAME = 'ammo_topdown_cam'
TOPDOWN_TOPIC = C.TOPDOWN_TOPIC
TOPDOWN_PX = 640
TOPDOWN_HFOV = 1.5


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


def shuffle_mushrooms(world, count, seed=None):
    """Random positions a mushroom could plausibly be found and collected at.

    Clear of trunks, out of the river, apart from each other and away from
    the spawn clearing, inside the same playable area the quad searches.
    Falls back to fewer than asked rather than looping forever.
    """
    meta = world_meta_full(world)
    trunks = trunk_pairs(world_meta(world)[0])
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
        if any(math.hypot(x - tx, y - ty) < SHUFFLE_TRUNK_M for tx, ty in trunks):
            continue
        if any(math.hypot(x - px, y - py) < SHUFFLE_APART_M for px, py in picked):
            continue
        river = _river_distance(meta, x, y)
        if river is not None and river < SHUFFLE_RIVER_M:
            continue
        picked.append((round(x, 2), round(y, 2)))
    return picked


def sim_already_running():
    out = subprocess.run(['pgrep', '-f', r'^(ruby [^ ]+/)?(ign gazebo|gz sim) '],
                         capture_output=True, text=True)
    return bool(out.stdout.strip())


def _as_launch_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def topdown_sdf(height, px=TOPDOWN_PX, hfov=TOPDOWN_HFOV):
    """A camera looking straight down, as a standalone model.

    Pitched a quarter turn about Y so the lens, which looks along +x by
    default, looks along -z instead.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{TOPDOWN_NAME}">
    <static>true</static>
    <pose>0 0 {height:.2f} 0 1.5707963 0</pose>
    <link name="link">
      <sensor name="topdown" type="camera">
        <always_on>1</always_on>
        <update_rate>1</update_rate>
        <topic>{TOPDOWN_TOPIC}</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{px}</width><height>{px}</height><format>R8G8B8</format></image>
          <clip><near>0.5</near><far>{height * 3.0:.1f}</far></clip>
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
        self.cleared_world = False  # the world's own mushrooms have been removed
        self.held = False          # the two mission nodes are frozen right now
        self._reported_death = False
        self._topdown_proc = None

    # -- stage one: prepare -------------------------------------------------
    def prepare(self, world, show_sim=True, options=None, tool='gripper'):
        """Bring the simulation up, or reuse the one already up.

        Returns 'started', 'reused', or None if it could not begin.
        """
        if self.state in (READY, RUNNING) and self.world == world:
            return 'reused'
        if self.state in (PREPARING, READY, RUNNING):
            self.shutdown()        # a different world needs a different Gazebo
        if sim_already_running():
            self.log('Gazebo is already running - close it first, the GUI starts its own.')
            return None
        path = world_path(world)
        if not world_available(world):
            self.log(f'world {world} not found - for river worlds run forest_worlds/setup.sh')
            return None

        env = dict(os.environ)
        # the nodes we freeze talk over UDP: over shared memory a long freeze cut them off from Nav2 for good
        env['AMMO_FREEZABLE_PROFILE'] = os.path.join(_share('ammo_gui_node') or '', 'config', 'fastdds_udp_only.xml')
        cmd = ['ros2', 'launch', 'ammo_mission_control', 'cage_mission.launch.py',
               f'world:={path}', f'tool:={tool}', 'meta_goals:=false', 'rviz:=false',
               # cage_mission defaults husky_yolo to false, which leaves the
               # rover with no detector and therefore no boxes on its feed.
               'husky_yolo:=true',
               f'headless:={"false" if show_sim else "true"}']
        for key, default in SEARCH_DEFAULTS.items():
            cmd.append(f'{key}:={_as_launch_value((options or {}).get(key, default))}')
        if WORLDS.get(world) == 'forest_worlds':
            # as forest_worlds/demo_river.sh: its models first, and the two-class river detector
            base = _forest_worlds_dir()
            extra = glob.glob(os.path.join(base, 'models', '*')) + [os.path.join(base, 'world_assets')]
            env['IGN_GAZEBO_RESOURCE_PATH'] = ':'.join(
                extra + [env.get('IGN_GAZEBO_RESOURCE_PATH', '')]).strip(':')
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
        self.show_sim = bool(show_sim)
        self.options = dict(options or {})
        self.started_at = time.monotonic()
        self._reported_death = False
        self.spawned = []
        self.cleared_world = False
        self.held = False
        self.log(f'preparing {world} - Gazebo and Nav2 take about 30 s '
                 f'(log: {self.log_path})')
        return 'started'

    def hold_nodes(self):
        """Freeze the two mission nodes, as soon as they exist.

        This is called the first moment the processes appear, long before the
        rest of the checklist has filled in - which is the whole difference
        between "prepared" and "already half way through a mission". The
        quadcopter takes off a few seconds after its node starts, so waiting
        until everything else was up meant watching it fly a search pattern
        over a world with no mushrooms in it while the page still said
        PREPARING.

        Nothing else on the checklist depends on these two: the cameras,
        odometry and TF come from Gazebo's bridges, the detectors are their
        own processes, and Nav2, truth_localizer and collision_guard are all
        separate. So freezing early costs nothing and is honest.

        The caller must also start a zero-velocity hold. Gazebo's
        VelocityControl plugin keeps the last command it was given forever,
        so a node frozen mid-takeoff leaves a quadcopter still climbing.

        Returns True only on the call that actually froze them.
        """
        if self.held or self.state not in (PREPARING, READY):
            return False
        self.suspend(MISSION_NODES)
        self.held = True
        return True

    def mark_ready(self):
        """Everything has arrived; hold here until the operator launches.

        Also accepted from READY, which is the state a finished mission
        leaves behind: preparing the same world again re-freezes rather than
        restarting anything.
        """
        if self.state not in (PREPARING, READY):
            return False
        self.hold_nodes()          # usually already done, early, by the caller
        self.suspend(MISSION_NODES)
        self.held = True
        self.state = READY
        self.log('ready - the simulation is up and the mission nodes are held')
        return True

    # -- stage two: run -----------------------------------------------------
    def launch(self):
        if self.state != READY:
            return False
        self.resume(MISSION_NODES)
        self.held = False
        self.state = RUNNING
        return True

    def end_mission(self, keep_sim=True):
        """Stop the mission. By default Gazebo stays up for the next run."""
        if self.state == IDLE:
            return
        if not keep_sim:
            self.shutdown()
            return
        self.suspend(MISSION_NODES)
        self.held = True
        self.remove_mushrooms()
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
        for i, (x, y) in enumerate(points, 1):
            name = f'{MUSHROOM_PREFIX}{i}'
            z = ground_z_at(self.world, float(x), float(y)) + GROUND_LIFT_M
            self._run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', sdf,
                       '-name', name, '-x', str(float(x)), '-y', str(float(y)),
                       '-z', f'{z:.3f}'], 20)
            self.spawned.append((name, float(x), float(y)))
        if self.spawned:
            self.log(f'{len(self.spawned)} mushroom(s) placed')
        return [(x, y) for _n, x, y in self.spawned]

    def remove_mushrooms(self):
        """Take out the mushrooms the GUI put there. Leaves the world's own."""
        for name, _x, _y in self.spawned:
            self._remove_model(name)
        self.spawned = []

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
        try:
            return subprocess.run(args, capture_output=True, text=True, timeout=timeout).stdout
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
            return ''

    # -- the top-down photo -------------------------------------------------
    def start_topdown(self):
        """Spawn a downward camera and bridge its image topic.

        Returns False if it could not be started. The GUI subscribes to
        TOPDOWN_TOPIC, takes the first frame it gets, and calls
        `stop_topdown()`. On a machine without hardware OpenGL this can take
        several seconds or never arrive, which is why the caller times out
        and falls back to the drawn map.
        """
        if self.state not in (READY, RUNNING) or not self.world:
            return False
        size = 2.0 * max(8.0, world_meta(self.world)[2] + 4.5)
        height = max(12.0, size / (2.0 * math.tan(TOPDOWN_HFOV / 2.0)) + 2.0)
        path = os.path.join(os.path.dirname(self.log_path), 'ammo_topdown.sdf')
        try:
            with open(path, 'w') as fh:
                fh.write(topdown_sdf(height))
        except OSError:
            return False
        # `create` prints nothing useful on success on some builds, so a
        # blank result is not proof of failure - but a missing executable is,
        # and that is what the bridge below would fail on anyway.
        self._run(['ros2', 'run', 'ros_ign_gazebo', 'create',
                   '-file', path, '-name', TOPDOWN_NAME], 25)
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
        self.proc = None
        self.spawned = []
        self.cleared_world = False
        self.held = False
        self.state = IDLE
        self.log('mission and simulation stopped')

    # kept so older wiring keeps working
    def stop_mission(self):
        self.shutdown()

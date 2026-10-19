"""Starts, stops, suspends and resumes the mission.

Launch runs the team's own command, `cage_mission.launch.py` (z-camera quad,
rover_capture_node, collision guard, Nav2 and Gazebo), as one process group,
so the GUI always runs the current mission code. Pause and operator goals
freeze individual mission nodes inside that group; the simulation keeps
running.

Search-pattern settings from the setup page are passed as launch arguments.
They are declared in `zcam_mission.launch.py` rather than in
`cage_mission.launch.py`, and reach it because an argument given on the
`ros2 launch` command line becomes a launch configuration that an inner
`DeclareLaunchArgument` does not overwrite - the same route the existing
`model_path` and `imgsz` arguments already take.
"""
import glob
import json
import os
import signal
import subprocess
import time

# executables inside the launch, by role
ROVER_NODES = ('rover_capture_node', 'cage_capture_node')
GUARD_NODES = ('collision_guard_node',)
QUAD_NODES = ('zcam_quad_node', 'quad_search_node')
MISSION_NODES = ROVER_NODES + GUARD_NODES + QUAD_NODES
STOP_TIMEOUT_S = 20.0
SIM_WARMUP_S = 45.0       # below this, an exit means it failed rather than finished

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
    """The whole .meta.json for a world, or {} if it has none.

    Cached: both windows load this per world, and the setup page redraws its
    preview on every keystroke in a coordinate field.
    """
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


def world_natives(world):
    """Native mushrooms the world file placed, as [[x, y], ...]."""
    data = _meta(world) or {}
    return [[round(m['x'], 2), round(m['y'], 2)] for m in data.get('natives', [])]


def trunk_pairs(flat):
    return list(zip(flat[0::2], flat[1::2]))


def sim_already_running():
    out = subprocess.run(['pgrep', '-f', r'^(ruby [^ ]+/)?(ign gazebo|gz sim) '],
                         capture_output=True, text=True)
    return bool(out.stdout.strip())


def _as_launch_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


class MissionLauncher:

    def __init__(self, logger=None):
        self.proc = None
        self.log = logger or (lambda _m: None)
        self.log_path = os.path.join(os.path.expanduser('~'), '.ros', 'ammo_gui_mission.log')
        self.started_at = 0.0
        self.show_sim = False
        self._reported_death = False

    def start_mission(self, world, tool='gripper', spawn=(), show_sim=False, options=None):
        """Launch the sim and mission in one go. Returns trunk pairs, or None."""
        if self.proc and self.proc.poll() is None:
            self.log('a mission is already running')
            return None
        if sim_already_running():
            self.log('Gazebo is already running - close it first, the GUI starts its own.')
            return None
        path = world_path(world)
        if not world_available(world):
            self.log(f'world {world} not found - for river worlds run forest_worlds/setup.sh')
            return None

        env = dict(os.environ)
        cmd = ['ros2', 'launch', 'ammo_mission_control', 'cage_mission.launch.py',
               f'world:={path}', f'tool:={tool}', 'meta_goals:=false', 'rviz:=false',
               # cage_mission defaults husky_yolo to false, which leaves the
               # rover with no detector and therefore no boxes on its feed.
               'husky_yolo:=true',
               f'headless:={"false" if show_sim else "true"}']
        for key, default in SEARCH_DEFAULTS.items():
            value = (options or {}).get(key, default)
            cmd.append(f'{key}:={_as_launch_value(value)}')
        if WORLDS.get(world) == 'forest_worlds':
            # as forest_worlds/demo_river.sh: its models first, and the two-class river detector
            base = _forest_worlds_dir()
            extra = glob.glob(os.path.join(base, 'models', '*')) + [os.path.join(base, 'world_assets')]
            env['IGN_GAZEBO_RESOURCE_PATH'] = ':'.join(
                extra + [env.get('IGN_GAZEBO_RESOURCE_PATH', '')]).strip(':')
            model = os.path.join(_share('perception') or '', 'models', 'variants',
                                 'amanita_river_v4_yolo11n.pt')
            if os.path.exists(model):
                cmd.append(f'model_path:={model}')
        if spawn:
            cmd.append('mushrooms:=' + ','.join(f'{float(v)}' for p in spawn for v in p))
        # straight to a file: piping every node's log through the GUI costs CPU
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            with open(self.log_path, 'w') as log:
                self.proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                             stdin=subprocess.DEVNULL, start_new_session=True)
        except FileNotFoundError:
            self.log('could not start the mission: is the workspace sourced?')
            return None
        self.started_at = time.monotonic()
        self.show_sim = bool(show_sim)
        self._reported_death = False
        self.log(f'launching {world} - Gazebo and Nav2 take about 30 s (log: {self.log_path})')
        return trunk_pairs(world_meta(world)[0])

    # -- health -------------------------------------------------------------
    def health_warning(self):
        """A sentence to show the operator if the launch died on its own.

        Mostly this catches Gazebo's GUI failing to start, which on WSL comes
        out of Ogre and takes the whole launch with it. Reported once.
        """
        if self.proc is None or self._reported_death:
            return None
        if self.proc.poll() is None:
            return None
        self._reported_death = True
        if time.monotonic() - self.started_at > SIM_WARMUP_S:
            return 'The mission launch exited. See ~/.ros/ammo_gui_mission.log.'
        tail = self._log_tail()
        if 'Ogre' in tail or 'GL3Plus' in tail or 'UnimplementedException' in tail:
            if self.show_sim:
                return ('Gazebo could not open its window (an Ogre/OpenGL error). '
                        'Untick "Show the Gazebo window" and launch again, or run '
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

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    # -- teardown -----------------------------------------------------------
    def stop_mission(self):
        """End the launch: mission nodes, Nav2 and Gazebo."""
        if not self.proc:
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
        self.log('mission and simulation stopped')

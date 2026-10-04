"""Starts, stops, suspends and resumes the mission nodes, and spawns mushrooms.

Each mission node is launched as its own `ros2 run` process rather than one
`ros2 launch`, which costs a little more code here and buys two things the
GUI genuinely needs:

  * pause can suspend the mission nodes individually, and
  * an operator goal can suspend only the rover's node - leaving the
    quadcopter flying and still reporting mushrooms - while Nav2 drives the
    rover to where you clicked.

The parameters mirror `ammo_mission_control/launch/cage_mission.launch.py`
so the nodes behave exactly as they do when the team runs that launch file.
What is deliberately *not* here is the simulation: Gazebo, Nav2 and the
cameras come up once in terminal 1 and stay up, so starting and ending
missions never costs a 30-second world reload.
"""
import json
import os
import signal
import subprocess

ROVER_NODES = ('cage_capture', 'collision_guard')
QUAD_NODES = ('quad_search',)
STOP_TIMEOUT_S = 6.0


def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        return None


def world_meta(world):
    """Trunk positions, ground height and playable size, if the world has them.

    Only the generated forest worlds ship a .meta.json; `simple_trees` does
    not, so fall back to the same defaults cage_mission.launch.py uses.
    """
    share = _share('41068_ignition_bringup')
    meta = os.path.join(share, 'worlds', f'{world}.meta.json') if share else None
    trunks, ground_z, half = [0.0, 3.0, 5.0, 0.0], 0.0, 6.5
    if meta and os.path.exists(meta):
        try:
            with open(meta) as fh:
                data = json.load(fh)
            trunks = [v for o in data.get('objects', [])
                      if o.get('type') == 'tree' for v in (o['x'], o['y'])]
            half = data['world']['playable_size_m'] / 2.0 - 4.5
            ground_z = data.get('spawn_clearing', {}).get('ground_z', 0.0)
        except Exception:
            pass
    return trunks, float(ground_z), float(half)


def trunk_pairs(flat):
    return list(zip(flat[0::2], flat[1::2]))


def mushroom_sdf():
    share = _share('41068_ignition_bringup')
    if not share:
        return None
    path = os.path.join(share, 'models', 'mushroom_amanita', 'model.sdf')
    return path if os.path.exists(path) else None


def _arr(values):
    """A float list as ROS 2 expects it on the command line."""
    return '[' + ', '.join(f'{float(v)}' for v in values) + ']'


class MissionLauncher:

    def __init__(self, logger=None):
        self.procs = {}          # name -> Popen
        self.spawned = []        # names of mushrooms we put in the world
        self.log = logger or (lambda _m: None)

    # -- mushrooms ----------------------------------------------------------
    def spawn_mushrooms(self, points):
        """Put the chosen layout into the running world, one process each.

        `simple_trees.sdf` contains no mushrooms at all - every demo
        mushroom in this project is spawned at runtime like this, which is
        exactly why the layout can be chosen in the GUI instead of being
        baked into the world file.
        """
        sdf = mushroom_sdf()
        if not sdf:
            self.log('mushroom model not found - is 41068_ignition_bringup built?')
            return 0
        placed = 0
        for i, (x, y) in enumerate(points, 1):
            name = f'gui_mushroom_{i}'
            cmd = ['ros2', 'run', 'ros_ign_gazebo', 'create',
                   '-file', sdf, '-name', name,
                   '-x', str(float(x)), '-y', str(float(y)), '-z', '0.0']
            try:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
                self.spawned.append(name)
                placed += 1
            except Exception as exc:
                self.log(f'could not spawn {name}: {exc}')
        self.log(f'spawning {placed} mushroom(s)')
        return placed

    # -- mission nodes ------------------------------------------------------
    def start_mission(self, world, tool='gripper', with_quad=True):
        trunks, ground_z, half = world_meta(world)
        sim = ['-p', 'use_sim_time:=true']

        self._start('truth_localizer', [
            'ros2', 'run', 'ammo_mission_control', 'truth_localizer',
            '--ros-args', *sim,
            '-r', '/tf:=/husky1/tf', '-r', '/tf_static:=/husky1/tf_static'])

        self._start('collision_guard', [
            'ros2', 'run', 'skidsteer_controller', 'collision_guard_node',
            '--ros-args', *sim,
            '-r', '/tf:=/husky1/tf', '-r', '/tf_static:=/husky1/tf_static',
            # Nav2's velocity smoother publishes husky1/cmd_vel_smoothed.
            # cage_mission.launch.py remaps that onto cmd_vel_nav_guarded
            # from inside its own launch scope; we bring the simulation up
            # separately, so the guard has to listen where Nav2 actually
            # publishes or Nav2's output never reaches the robot.
            '-r', '/husky1/cmd_vel_nav_guarded:=/husky1/cmd_vel_smoothed'])

        if with_quad:
            alt = 3.0
            self._start('quad_search', [
                'ros2', 'run', 'ammo_mission_control', 'quad_search_node',
                '--ros-args', *sim,
                '-p', f'trunks:={_arr(trunks)}',
                '-p', 'waypoints:=[0.0]',
                '-p', f'ground_z:={ground_z}',
                '-p', f'search_area:={_arr([-half, half, -half, half, alt])}'])

        self._start('cage_capture', [
            'ros2', 'run', 'ammo_mission_control', 'cage_capture_node',
            '--ros-args', *sim,
            '-p', f'world:={world}',
            '-p', f'trunks:={_arr(trunks)}',
            '-p', 'count:=0',
            '-p', f'tool:={tool}'])

        return trunk_pairs(trunks)

    def _start(self, name, cmd):
        if name in self.procs:
            return
        try:
            # Its own session, so a signal reaches the node and anything it
            # spawns rather than just the `ros2 run` wrapper.
            self.procs[name] = subprocess.Popen(cmd, start_new_session=True)
            self.log(f'started {name}')
        except FileNotFoundError:
            self.log(f'could not start {name}: is the workspace sourced?')
        except Exception as exc:
            self.log(f'could not start {name}: {exc}')

    # -- suspend / resume ---------------------------------------------------
    def _signal(self, names, sig):
        for name in names:
            proc = self.procs.get(name)
            if proc is None or proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), sig)
            except (ProcessLookupError, PermissionError, OSError):
                pass

    def suspend(self, names=None):
        """Freeze nodes so they stop issuing commands.

        Used both for pause and for an operator goal, where only the rover's
        nodes are frozen so the quadcopter keeps searching.
        """
        self._signal(names or list(self.procs), signal.SIGSTOP)

    def resume(self, names=None):
        self._signal(names or list(self.procs), signal.SIGCONT)

    def running(self):
        return any(p.poll() is None for p in self.procs.values())

    # -- teardown -----------------------------------------------------------
    def stop_mission(self):
        # Anything suspended has to be woken first or it will never see the
        # interrupt and we would end up killing it the hard way.
        self.resume()
        self._signal(list(self.procs), signal.SIGINT)
        for name, proc in list(self.procs.items()):
            try:
                proc.wait(timeout=STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self._signal([name], signal.SIGKILL)
            except Exception:
                pass
        self.procs.clear()
        self.log('mission nodes stopped')

    def remove_mushrooms(self, world):
        """Take our spawned mushrooms back out so the next run starts clean.

        The rover deletes the ones it collects, but anything it never
        reached would otherwise still be lying there when you set up the
        next run and would be found all over again. Same Ignition service
        call `cage_capture_node` uses for its own removals.
        """
        removed = 0
        for name in list(self.spawned):
            cmd = ['ign', 'service', '-s', f'/world/{world}/remove',
                   '--reqtype', 'ignition.msgs.Entity',
                   '--reptype', 'ignition.msgs.Boolean',
                   '--timeout', '2000',
                   '--req', f'name: "{name}" type: MODEL']
            try:
                subprocess.run(cmd, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=4)
                removed += 1
            except Exception:
                pass
        self.spawned.clear()
        if removed:
            self.log(f'removed {removed} leftover mushroom(s)')
        return removed

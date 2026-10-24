"""fleet_mission.launch.py for benchmarks: up to 6 quads, and an optional lighter sim.

    ros2 launch quad_zcams fleet_bench.launch.py world:=<abs path>/river_moderate_aus headless:=true rviz:=false \
        meta_goals:=false quads:=6 rovers:=2 lite:=true

Arguments on top of fleet_mission's:
  queue_cap:=N    the coordinator gives a rover at most N open mushrooms; the rest go to whichever rover frees up first
                  (default 1; 0 = fleet_mission's static queues)
  lite:=true      shorthand for drop_points:=true cam_hz:=3 (and yolo_pool:=true, now the default)
  husky_yolo:=    the rover's detector feeds the debug window (default true, 1 Hz in the pool); false frees those frames
  drop_points:=true  the quads' 720x480 camera point clouds (unused, ~10 MB a frame) are not bridged to ROS
  cam_hz:=N       the quads' RGB-D camera rate (stock 6 Hz; YOLO samples it at 2 Hz)

lite, drop_points and cam_hz change the robot description and bridge files, which are built before arguments are
resolved, so they are read from the command line.
"""
import importlib.util
import os
import re
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

BRINGUP = get_package_share_directory('41068_ignition_bringup')
HERE = get_package_share_directory('quad_zcams')
MAX_QUADS = 6
# the 4 m spawn clearing: the fleet's 4 quad spots, then two diagonals clear of the rovers at (+-2.5, +-2.5)
SPAWNS = [(2.0, 0.0), (-2.0, 0.0), (0.0, 2.0), (0.0, -2.0), (1.3, 1.3), (-1.3, 1.3)]


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _argv(name, default):
    for a in sys.argv:
        if a.startswith(name + ':='):
            return a.split(':=', 1)[1]
    return default


LITE = _argv('lite', 'false').lower() == 'true'
DROP_POINTS = _argv('drop_points', str(LITE)).lower() == 'true'
CAM_HZ = _argv('cam_hz', '3' if LITE else '')


def _light_parrot():
    """A copy of the z-camera Parrot's xacros with the RGB-D camera at CAM_HZ; returns the pkg_path to use."""
    out = tempfile.mkdtemp(prefix='fleet_bench_urdf_')
    src = os.path.join(BRINGUP, 'urdf_parrot')
    for f in ('parrot.urdf.xacro', 'parrot.gazebo.xacro', 'common_properties.urdf.xacro'):
        s = open(os.path.join(src, f)).read()
        s = re.sub(r'\$\(find 41068_ignition_bringup\)/urdf_parrot/(\w+\.(?:urdf|gazebo)\.xacro)', out + r'/\1', s)
        if f == 'parrot.gazebo.xacro' and CAM_HZ:
            s = re.sub(r'(type="rgbd_camera">.*?<update_rate>)[^<]*(</update_rate>)', rf'\g<1>{CAM_HZ}\g<2>', s,
                       flags=re.S)
        open(os.path.join(out, f), 'w').write(s)
    os.makedirs(os.path.join(out, 'urdf'))
    open(os.path.join(out, 'urdf', 'parrot_zcams.urdf.xacro'), 'w').write(
        open(os.path.join(HERE, 'urdf', 'parrot_zcams.urdf.xacro')).read().replace(
            '$(find 41068_ignition_bringup)/urdf_parrot/parrot.urdf.xacro', os.path.join(out, 'parrot.urdf.xacro')))
    return out


def _light_configs(config_path, bridge, loc):
    """bridge and localization files copied, minus the camera point cloud."""
    out = tempfile.mkdtemp(prefix='fleet_bench_cfg_')
    shutil.copy(os.path.join(config_path, loc), out)
    entries = open(os.path.join(config_path, bridge)).read().split('\n- ')
    keep = [e for e in entries if 'camera/depth/points' not in e]
    open(os.path.join(out, bridge), 'w').write('\n- '.join(keep))
    return out


def generate_launch_description():
    fm = _load(os.path.join(HERE, 'launch', 'fleet_mission.launch.py'), 'fleet_mission_launch')
    fm.MAX_QUADS, fm.SPAWNS = MAX_QUADS, SPAWNS
    zm_load = fm._load

    def load(path, name):  # fleet_mission loads zcam_mission through this: hook its robot builder
        mod = zm_load(path, name)
        if name == 'zcam_mission_launch' and (DROP_POINTS or CAM_HZ):
            helpers = mod._bringup_helpers
            urdf = _light_parrot() if CAM_HZ else None

            def bringup():
                base = helpers()
                add_robot = base.add_robot

                def light(ld, **kw):
                    if 'parrot_zcams.urdf.xacro' in kw.get('xacro_parts', []):
                        if urdf:
                            kw['pkg_path'] = urdf
                        if DROP_POINTS:
                            kw['config_path'] = _light_configs(kw['config_path'], kw['bridge_config'],
                                                               kw['localization_config'])
                    return add_robot(ld, **kw)
                base.add_robot = light
                return base
            mod._bringup_helpers = bringup
        return mod
    fm._load = load
    fleet_node = fm.Node

    def node(**kw):
        if kw.get('executable') == 'fleet_coordinator':
            kw['parameters'] = [dict(kw['parameters'][0], queue_cap=LaunchConfiguration('queue_cap'),
                                     river_meta=[LaunchConfiguration('world'), '.meta.json'])]
        return fleet_node(**kw)
    fm.Node = node
    inner = fm.generate_launch_description()

    lite = [SetLaunchConfiguration('yolo_pool', 'true')] if LITE else []
    return LaunchDescription([
        DeclareLaunchArgument('lite', default_value='false'),
        DeclareLaunchArgument('queue_cap', default_value='1'),
        DeclareLaunchArgument('drop_points', default_value='false'),
        DeclareLaunchArgument('cam_hz', default_value=''),
        *lite,
        *inner.entities])

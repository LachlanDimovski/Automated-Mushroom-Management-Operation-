"""AMMO main launch: the full mission (sim, Nav2, quad, rover). Same arguments as cage_mission.launch.py.

    ros2 launch ammo_mission_control ammo_main.launch.py
    ros2 launch ammo_mission_control ammo_main.launch.py world:=forest_random headless:=true
"""
import importlib.util
import os

_spec = importlib.util.spec_from_file_location(
    'cage_mission_launch', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cage_mission.launch.py'))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


def generate_launch_description():
    return _mod.generate_launch_description()

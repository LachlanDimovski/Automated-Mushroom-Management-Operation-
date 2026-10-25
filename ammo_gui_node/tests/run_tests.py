"""Run every check, headless.

    python3 src/RS1_Team5_/ammo_gui_node/tests/run_tests.py

No ROS, no simulation, no display and no built workspace: the stubs in
`stubs/` stand in for rclpy and the message types. Takes a few seconds.

Add `--render` to also write pictures of both windows and the map into
`tests/out/`, which is the quickest way to see what a change did to a layout
without starting Gazebo.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402

# (title, file, needs PySide6)
SUITES = [
    ('log lines still in the mission nodes', 'test_log_lines.py', False),
    ('the status bridge', 'test_bridge.py', False),
    ('both windows and the launcher', 'test_windows.py', True),
]
RENDERS = [
    ('the map, on both world shapes', 'render_map.py', True),
    ('the Debug UI', 'render_windows.py', True),
]


def have_qt():
    import importlib.util
    return importlib.util.find_spec('PySide6') is not None


def run(path):
    # Deliberately not -I: that would isolate away the user's site-packages,
    # which is where `pip3 install PySide6` puts Qt. The stubs still win over
    # a sourced ROS workspace because _paths puts them at the front of
    # sys.path. -B keeps .pyc files out of the checkout.
    return subprocess.call([sys.executable, '-B', os.path.join(_paths.HERE, path)],
                           cwd=_paths.HERE)


def main():
    failures, skipped = [], []
    qt = have_qt()
    plan = list(SUITES) + (list(RENDERS) if '--render' in sys.argv else [])
    for title, path, needs_qt in plan:
        if needs_qt and not qt:
            skipped.append(title)
            continue
        print(f'\n=== {title} ' + '=' * max(0, 58 - len(title)))
        if run(path) != 0:
            failures.append(title)
    if '--render' in sys.argv and qt:
        print(f'\npictures in {_paths.OUT}')

    if skipped:
        print('\nSkipped, because PySide6 is not installed for %s:' % sys.executable)
        for title in skipped:
            print('  -', title)
        print('  Install it with:  pip3 install PySide6')

    print('\n' + '=' * 64)
    if failures:
        print('FAILED: ' + ', '.join(failures))
        return 1
    print('everything passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())

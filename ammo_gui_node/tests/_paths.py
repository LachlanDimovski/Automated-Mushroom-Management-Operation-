"""Put the stub ROS packages and this package on the path.

The tests run without ROS, without a display and without a built workspace:
`stubs/` holds just enough of rclpy and the message types for the GUI's nodes
and widgets to be constructed and driven by hand.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)                  # .../ammo_gui_node
REPO = os.path.dirname(PKG)                  # the workspace src checkout

for path in (os.path.join(HERE, 'stubs'), PKG, HERE):
    if path not in sys.path:
        sys.path.insert(0, path)

# get_package_share_directory() in the stub resolves against the checkout, so
# layouts/ and worlds/ are found exactly where a built workspace would put them
os.environ.setdefault('AMMO_STUB_SHARE', REPO)
# Forced, not setdefault: the terminals used to launch the GUI export
# QT_QPA_PLATFORM=xcb, and the tests must never try to open a real window.
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

BRINGUP_WORLDS = os.path.join(REPO, '41068_ignition_bringup', 'worlds')
FOREST_META = os.path.join(BRINGUP_WORLDS, 'forest_random.meta.json')
OUT = os.path.join(HERE, 'out')


def out_dir():
    os.makedirs(OUT, exist_ok=True)
    return OUT

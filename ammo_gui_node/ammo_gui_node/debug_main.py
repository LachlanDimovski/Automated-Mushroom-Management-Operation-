"""AMMO Debug UI - entry point.

    ros2 launch ammo_gui_node debug.launch.py
    ros2 run ammo_gui_node debug

Read-only: it subscribes and never publishes, so opening or closing it
cannot disturb a running mission. Start it before, during or after a run.

The timeline and the state cards need `mission_bridge` to be running, which
`gui.launch.py` starts alongside the operator window. The cameras, minimap
and velocities come straight off the simulation and work on their own.
"""
import sys

import rclpy
from PySide6.QtWidgets import QApplication

from ammo_gui_node import theme
from ammo_gui_node.debug_window import DebugWindow
from ammo_gui_node.ros_bridge import RosLink


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    link = RosLink(role='debug')
    window = DebugWindow(link)
    link.start()

    app.aboutToQuit.connect(link.shutdown)
    window.show()

    code = app.exec()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

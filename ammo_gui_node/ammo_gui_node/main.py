"""AMMO operator GUI - entry point.

    ros2 launch ammo_gui_node gui.launch.py
    (or: ros2 run ammo_gui_node gui)

Needs the core sim already up in another terminal (see the cheat sheet) -
this only starts the mission-control layer, not Gazebo/SLAM/Nav2/YOLO. See
mission_launcher.py for why that split, and each module's own docstring for
the rest of the design notes.
"""
import sys

import rclpy
from PySide6.QtWidgets import QApplication, QStackedWidget

from ammo_gui_node import theme
from ammo_gui_node.home_page import HomePage
from ammo_gui_node.mission_launcher import MissionLauncher
from ammo_gui_node.operator_page import OperatorPage
from ammo_gui_node.ros_bridge import RosBridge


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    bridge = RosBridge()
    bridge.start()

    launcher = MissionLauncher()

    stack = QStackedWidget()
    stack.setWindowTitle('AMMO Operator GUI')
    stack.resize(1400, 900)

    home = HomePage()
    operator = OperatorPage(bridge)
    stack.addWidget(home)
    stack.addWidget(operator)

    def on_begin_mission(mode, area_ha):
        bridge.set_target_area_ha(area_ha)
        launcher.start(mode)
        operator.start_mission(mode, area_ha)
        stack.setCurrentWidget(operator)

    def on_end_mission():
        launcher.stop()
        operator.stop_mission()
        stack.setCurrentWidget(home)

    home.begin_mission.connect(on_begin_mission)
    operator.end_mission.connect(on_end_mission)

    def on_quit():
        launcher.stop()
        bridge.shutdown()

    app.aboutToQuit.connect(on_quit)

    stack.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()

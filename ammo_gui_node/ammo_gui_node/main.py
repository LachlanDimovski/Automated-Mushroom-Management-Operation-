"""AMMO operator GUI - entry point.

    ros2 launch ammo_gui_node gui.launch.py       (also starts the bridge)
    ros2 run ammo_gui_node gui                    (bridge separately)

Launch starts the whole run (Gazebo, Nav2, both robots and the mission
nodes) through cage_mission.launch.py; ending the mission closes it again.
This window is the only thing that pauses the mission nodes. The Debug UI is
read-only and can be opened or closed at any point without affecting a run.
"""
import sys

import rclpy
from PySide6.QtWidgets import QApplication, QStackedWidget

from ammo_gui_node import theme
from ammo_gui_node.home_page import HomePage
from ammo_gui_node.mission_launcher import ROVER_NODES, MissionLauncher, world_mushrooms
from ammo_gui_node.operator_page import OperatorPage
from ammo_gui_node.ros_bridge import RosLink


class OperatorApp:
    """Wiring between the two pages, the ROS link and the child processes."""

    def __init__(self):
        self.link = RosLink(role='gui')
        self.launcher = MissionLauncher(logger=self._log)

        self.stack = QStackedWidget()
        self.stack.setWindowTitle('AMMO Operator')
        self.stack.resize(1400, 900)

        self.home = HomePage()
        self.operator = OperatorPage(self.link)
        self.stack.addWidget(self.home)
        self.stack.addWidget(self.operator)

        self.config = {}
        self.active_goal = None
        self.mission_running = False

        self.home.begin_mission.connect(self.begin_mission)
        self.operator.end_mission.connect(self.finish_mission)
        self.operator.pause_toggled.connect(self.set_paused)
        self.operator.goal_clicked.connect(self.send_goal)

        self.link.status_updated.connect(self.operator.apply_status)
        self.link.event_received.connect(self.operator.apply_event)
        self.link.rover_pose.connect(self.operator.on_rover_pose)
        self.link.quad_pose.connect(self.operator.on_quad_pose)
        self.link.rover_frame.connect(self.operator.on_rover_frame)
        self.link.quad_yolo_frame.connect(self.operator.on_quad_yolo_frame)
        self.link.nav_goal_finished.connect(self.on_goal_finished)

        self.link.start()

    def _log(self, message):
        self.home.set_note(message)

    # -- mission ------------------------------------------------------------
    def begin_mission(self, config):
        self.config = config
        world = config.get('world', 'simple_trees')
        spawn = config.get('points', []) if config.get('spawn', True) else []
        trunks = self.launcher.start_mission(world, tool=config.get('tool', 'gripper'),
                                             spawn=spawn, show_sim=config.get('show_sim', False))
        if trunks is None:
            return
        config['trunks'] = trunks
        total = len(spawn) + len(world_mushrooms(world))

        self.link.publish_command({
            'cmd': 'start',
            'mode': config.get('mode', 'autonomous'),
            'world': config.get('world', ''),
            'layout': config.get('layout', ''),
            'total': total,
        })
        self.mission_running = True
        self.operator.start_mission(config)
        self.stack.setCurrentWidget(self.operator)

    def finish_mission(self):
        self.link.publish_command({'cmd': 'end'})
        self.link.set_holding(False)
        self.link.publish_pause(False)
        self.active_goal = None
        self.launcher.stop_mission()
        self.mission_running = False
        self.operator.stop_mission()
        self.stack.setCurrentWidget(self.home)

    # -- pause --------------------------------------------------------------
    def set_paused(self, paused):
        if paused:
            # Order matters. Cancel first so Nav2 stops driving, start the
            # zero-velocity hold so nothing coasts on its last command, and
            # only then freeze the mission nodes - a frozen node cannot be
            # asked to stop itself.
            self.link.cancel_all_nav()
            self.link.set_holding(True)
            self.launcher.suspend()
        else:
            self.launcher.resume()
            self.link.set_holding(False)
            if self.active_goal:
                # Our own Nav2 goal was cancelled by the pause; put it back.
                self.link.send_nav_goal(*self.active_goal)
        self.link.publish_pause(paused)

    # -- operator goals -----------------------------------------------------
    def send_goal(self, x, y):
        if not self.mission_running:
            return
        if x != x or y != y:      # NaN from the Clear button
            self.clear_goal()
            return

        # Freeze only the rover's own mission node. The quadcopter keeps
        # flying and keeps reporting mushrooms while the rover is away,
        # which is the whole point of not just pausing everything.
        self.launcher.suspend(ROVER_NODES)
        self.link.cancel_all_nav()
        self.active_goal = (x, y)
        if not self.link.send_nav_goal(x, y):
            self._log('Nav2 did not accept the goal - is the sim running with nav2:=true?')
            self.launcher.resume(ROVER_NODES)
            self.active_goal = None
            return
        self.link.publish_command({'cmd': 'operator_goal', 'x': x, 'y': y})

    def on_goal_finished(self, _ok):
        if not self.active_goal:
            return
        self.active_goal = None
        self.launcher.resume(ROVER_NODES)
        self.link.publish_command({'cmd': 'operator_goal_done'})

    def clear_goal(self):
        if not self.active_goal:
            return
        self.active_goal = None
        self.link.cancel_all_nav()
        self.launcher.resume(ROVER_NODES)
        self.link.publish_command({'cmd': 'operator_goal_done'})

    # -- teardown -----------------------------------------------------------
    def quit(self):
        try:
            if self.mission_running:
                self.link.publish_command({'cmd': 'end'})
            self.launcher.stop_mission()
        finally:
            self.link.shutdown()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    operator = OperatorApp()
    app.aboutToQuit.connect(operator.quit)
    operator.stack.show()

    code = app.exec()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

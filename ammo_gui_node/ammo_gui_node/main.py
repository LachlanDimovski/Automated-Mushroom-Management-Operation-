"""AMMO operator GUI - entry point.

    ros2 launch ammo_gui_node gui.launch.py       (also starts the bridge)
    ros2 run ammo_gui_node gui                    (bridge separately)

Launch starts the whole run (Gazebo, Nav2, both robots and the mission
nodes) through cage_mission.launch.py; ending the mission closes it again.
This window is the only thing that pauses the mission nodes, and the only
thing that hand-drives either robot. The Debug UI is read-only and can be
opened or closed at any point without affecting a run.
"""
import math
import sys

import rclpy
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QStackedWidget

from ammo_gui_node import contract as C
from ammo_gui_node import theme
from ammo_gui_node.home_page import HomePage
from ammo_gui_node.mission_launcher import (QUAD_NODES, ROVER_NODES,
                                            MissionLauncher, world_mushrooms)
from ammo_gui_node.operator_page import OperatorPage
from ammo_gui_node.ros_bridge import RosLink

HEALTH_MS = 2000


class OperatorApp:
    """Wiring between the two pages, the ROS link and the child processes."""

    def __init__(self):
        self.link = RosLink(role='gui')
        self.launcher = MissionLauncher(logger=self._log)

        self.stack = QStackedWidget()
        self.stack.setWindowTitle('AMMO Operator')
        self.stack.resize(1500, 950)

        self.home = HomePage()
        self.operator = OperatorPage(self.link)
        self.stack.addWidget(self.home)
        self.stack.addWidget(self.operator)

        self.config = {}
        self.active_goal = None
        self.mission_running = False
        self.flown_by_hand = set()
        self.rover_at = None          # last rover pose, for aiming operator goals

        self.home.begin_mission.connect(self.begin_mission)
        self.operator.end_mission.connect(self.finish_mission)
        self.operator.pause_toggled.connect(self.set_paused)
        self.operator.goal_clicked.connect(self.send_goal)
        self.operator.teleop_armed.connect(self.set_hand_flown)

        self.link.status_updated.connect(self.operator.apply_status)
        self.link.event_received.connect(self.operator.apply_event)
        self.link.rover_pose.connect(self.operator.on_rover_pose)
        self.link.rover_pose.connect(self._note_rover_pose)
        self.link.quad_pose.connect(self.operator.on_quad_pose)
        self.link.rover_frame.connect(self.operator.on_rover_frame)
        self.link.quad_frame.connect(self.operator.on_quad_frame)
        self.link.rover_yolo_frame.connect(self.operator.on_rover_yolo_frame)
        self.link.quad_yolo_frame.connect(self.operator.on_quad_yolo_frame)
        self.link.rover_detections.connect(self.operator.on_rover_detections)
        self.link.quad_detections.connect(self.operator.on_quad_detections)
        self.link.nav_goal_finished.connect(self.on_goal_finished)

        # The launch runs detached with its output in a file, so the only way
        # to notice it died - Gazebo failing to open a window, most often - is
        # to look.
        self.health = QTimer(self.stack)
        self.health.timeout.connect(self._check_health)
        self.health.start(HEALTH_MS)

        self.link.start()

    def _log(self, message):
        self.home.set_note(message)

    def _note_rover_pose(self, x, y, _yaw):
        self.rover_at = (x, y)

    def _check_health(self):
        warning = self.launcher.health_warning()
        if not warning:
            return
        self._log(warning)
        if self.mission_running:
            self.link.publish_command({'cmd': 'note', 'text': warning})
            self.finish_mission()

    # -- mission ------------------------------------------------------------
    def begin_mission(self, config):
        self.config = config
        world = config.get('world', 'simple_trees')
        spawn = config.get('points', []) if config.get('spawn', True) else []
        trunks = self.launcher.start_mission(world, tool=config.get('tool', 'gripper'),
                                             spawn=spawn,
                                             show_sim=config.get('show_sim', True),
                                             options=config.get('options'))
        if trunks is None:
            return
        config['trunks'] = trunks
        total = len(spawn) + len(world_mushrooms(world))
        config['total'] = total

        self.link.publish_command({
            'cmd': 'start',
            'mode': config.get('mode', 'autonomous'),
            'world': world,
            'layout': config.get('layout', ''),
            'total': total,
        })
        self.mission_running = True
        self.flown_by_hand.clear()
        self.operator.start_mission(config)
        self.stack.setCurrentWidget(self.operator)

    def finish_mission(self):
        self.link.publish_command({'cmd': 'end'})
        self.link.set_holding(False)
        self.link.publish_pause(False)
        self.active_goal = None
        self.flown_by_hand.clear()
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
            # a robot being hand-flown goes back to being hand-flown, not
            # back to its mission node
            for robot in self.flown_by_hand:
                self.launcher.suspend(self._nodes_for(robot))
            self.link.set_holding(False)
            if self.active_goal:
                # Our own Nav2 goal was cancelled by the pause; put it back.
                self.link.send_nav_goal(*self.active_goal)
        self.link.publish_pause(paused)

    # -- hand flying --------------------------------------------------------
    @staticmethod
    def _nodes_for(robot):
        return ROVER_NODES if robot == 'rover' else QUAD_NODES

    def set_hand_flown(self, robot, armed):
        """Arming a teleop panel takes that robot off its mission node.

        Without this both the node and the operator publish velocity for the
        same robot and it obeys whichever message landed last, which feels
        like the controls are fighting you - because they are.
        """
        if not self.mission_running:
            return
        if armed:
            self.flown_by_hand.add(robot)
            if robot == 'rover':
                self.link.cancel_all_nav()
            self.launcher.suspend(self._nodes_for(robot))
            self.link.publish_command(
                {'cmd': 'note', 'text': f'{robot} taken over by the operator'})
        else:
            self.flown_by_hand.discard(robot)
            self.launcher.resume(self._nodes_for(robot))
            self.link.publish_command(
                {'cmd': 'note', 'text': f'{robot} handed back to its mission node'})

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
        # The guard replays the mission node's last command for half a second
        # after we freeze it. Send a zero so what it replays is a stop, then
        # let that window pass before handing the robot to Nav2 - otherwise
        # the two take turns driving and the rover just twitches.
        self.link.publish_stop()
        self.active_goal = (x, y)
        QTimer.singleShot(int(C.GUARD_SETTLE_S * 1000), lambda: self._dispatch_goal(x, y))

    def _dispatch_goal(self, x, y):
        if self.active_goal != (x, y):
            return                      # cleared or replaced while we waited
        yaw = 0.0
        if self.rover_at is not None:
            yaw = math.atan2(y - self.rover_at[1], x - self.rover_at[0])
        if not self.link.send_nav_goal(x, y, yaw):
            self._goal_failed('Nav2 would not take the goal - is it up yet?')
            return
        self.link.publish_command({'cmd': 'operator_goal', 'x': x, 'y': y})

    def _goal_failed(self, why):
        self.active_goal = None
        self._log(why)
        if 'rover' not in self.flown_by_hand:
            self.launcher.resume(ROVER_NODES)
        self.link.publish_command({'cmd': 'operator_goal_done', 'ok': False, 'why': why})

    def on_goal_finished(self, ok):
        if not self.active_goal:
            return
        self.active_goal = None
        if 'rover' not in self.flown_by_hand:
            self.launcher.resume(ROVER_NODES)
        # Distinguish the two: a rejected or aborted goal used to be reported
        # as "reached the goal", which is how a rover that never moved could
        # still claim to have arrived.
        self.link.publish_command({'cmd': 'operator_goal_done', 'ok': bool(ok)})

    def clear_goal(self):
        if not self.active_goal:
            return
        self.active_goal = None
        self.link.cancel_all_nav()
        if 'rover' not in self.flown_by_hand:
            self.launcher.resume(ROVER_NODES)
        self.link.publish_command({'cmd': 'operator_goal_done', 'ok': None})

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

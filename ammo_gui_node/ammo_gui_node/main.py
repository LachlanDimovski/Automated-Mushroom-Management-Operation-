"""AMMO operator GUI - entry point.

    ros2 launch ammo_gui_node gui.launch.py       (also starts the bridge)
    ros2 run ammo_gui_node gui                    (bridge separately)

A run starts in two stages, and the split is the point of this file.

**Prepare** runs the team's own `cage_mission.launch.py` - Gazebo, Nav2, both
robots, both detectors - and then watches until every one of those is
genuinely up, which takes about thirty seconds and sometimes much longer.
Only then does it freeze the two mission node processes, spawn the mushrooms
and photograph the result from above. Nothing has searched, driven or been
timed at that point.

**Launch** thaws those two processes and starts the clock, so the first
second of the mission is a second in which something actually happens.

Ending a mission freezes them again and takes the mushrooms out, but leaves
the simulation up: preparing the same world a second time is instant.

This window is the only thing that pauses the mission nodes and the only
thing that hand-drives either robot. The Debug UI is read-only and can be
opened or closed at any point without affecting a run.
"""
import math
import sys
import time

import rclpy
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QStackedWidget

from ammo_gui_node import contract as C
from ammo_gui_node import theme
from ammo_gui_node.home_page import HomePage
from ammo_gui_node.mission_launcher import (QUAD_NODES, ROVER_NODES,
                                            MissionLauncher, world_meta,
                                            world_mushrooms)
# aliased: this module has a slot of the same name, and a bare call that
# resolved to the module function by luck is not worth reading twice
from ammo_gui_node.mission_launcher import shuffle_mushrooms as random_placement
from ammo_gui_node.operator_page import OperatorPage
from ammo_gui_node.ready_page import ReadyPage
from ammo_gui_node.ros_bridge import RosLink

HEALTH_MS = 2000
# Twice a second, not once: the gap between the mission nodes appearing and
# the quadcopter leaving the ground is only a few seconds, and that gap is
# what the prepare stage has to get inside.
PROBE_MS = 500
PHOTO_WAIT_S = 20.0   # software rendering is slow; the drawing is the fallback


class OperatorApp:
    """Wiring between the three pages, the ROS link and the child processes."""

    def __init__(self):
        self.link = RosLink(role='gui')
        self.launcher = MissionLauncher(logger=self._log)

        self.stack = QStackedWidget()
        self.stack.setWindowTitle('AMMO Operator')
        self.stack.resize(1500, 950)

        self.home = HomePage()
        self.ready = ReadyPage()
        self.operator = OperatorPage(self.link)
        for page in (self.home, self.ready, self.operator):
            self.stack.addWidget(page)

        self.config = {}
        self.placed = []              # where the mushrooms actually are
        self.phase = 'setup'          # setup, preparing, ready, running
        self.active_goal = None
        self.mission_running = False
        self.flown_by_hand = set()
        self.rover_at = None          # last rover pose, for aiming operator goals
        self._prepare_t0 = 0.0
        self._photo_wait = False
        self._held_said = False       # the hold has been announced on the page

        self.home.prepare_requested.connect(self.begin_prepare)
        self.ready.launch_requested.connect(self.launch_mission)
        self.ready.shuffle_requested.connect(self.shuffle_mushrooms)
        self.ready.photo_requested.connect(self.take_photo)
        self.ready.back_requested.connect(self.back_to_setup)
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
        self.link.topdown_frame.connect(self.on_topdown)

        # The launch runs detached with its output in a file, so the only way
        # to notice it died - Gazebo failing to open a window, most often - is
        # to look.
        self.health = QTimer(self.stack)
        self.health.timeout.connect(self._check_health)
        self.health.start(HEALTH_MS)

        # Only runs during the prepare stage.
        self.probe = QTimer(self.stack)
        self.probe.timeout.connect(self._probe_tick)

        self.link.start()

    def _log(self, message):
        self.home.set_note(message)
        if self.phase in ('preparing', 'ready'):
            self.ready.note.setText(message)

    def _note_rover_pose(self, x, y, _yaw):
        self.rover_at = (x, y)

    def _check_health(self):
        warning = self.launcher.health_warning()
        if not warning:
            return
        self._log(warning)
        if self.phase == 'preparing':
            self.probe.stop()
            self.phase = 'setup'
            self.home.set_note(warning)
            self.stack.setCurrentWidget(self.home)
            return
        if self.mission_running:
            self.link.publish_command({'cmd': 'note', 'text': warning})
            self.finish_mission()

    # -- stage one: prepare -------------------------------------------------
    def begin_prepare(self, config):
        self.config = config
        world = config.get('world', 'simple_trees')
        started = self.launcher.prepare(world,
                                        show_sim=config.get('show_sim', True),
                                        options=config.get('options'),
                                        tool=config.get('tool', 'gripper'))
        if started is None:
            return
        self.phase = 'preparing'
        self.placed = []
        self._prepare_t0 = time.monotonic()
        self._held_said = False
        self.ready.begin(config)
        self.stack.setCurrentWidget(self.ready)
        self.link.publish_command({'cmd': 'prepare', 'stage': 'preparing'})
        self.probe.start(PROBE_MS)

    def _probe_tick(self):
        """One look at the simulation, folded into the checklist."""
        if self.phase != 'preparing':
            self.probe.stop()
            return
        probe = self.link.probe_readiness()
        # The one check the ROS graph cannot answer: our own child processes.
        probe['nodes'] = self.launcher.mission_nodes_up()
        if probe['nodes']:
            self._hold_robots()
        if self.ready.apply_probe(probe, time.monotonic() - self._prepare_t0):
            self.probe.stop()
            self._hold_and_place()

    def _hold_robots(self):
        """Freeze both mission nodes the moment they exist, and hold zero.

        Both halves matter. Freezing stops the nodes deciding anything;
        the zero-velocity hold stops the robots coasting, because Gazebo's
        VelocityControl plugin keeps its last command forever and a node
        frozen one second into a take-off leaves a quadcopter climbing with
        nobody flying it.
        """
        self.launcher.hold_nodes()
        # Once per prepare, not once per tick: the page's note is also where
        # "launch anyway" explains itself, and that must not be overwritten
        # every half second.
        if not self.launcher.held or self._held_said:
            return
        self._held_said = True
        self.link.set_holding(True)
        self.ready.set_held()

    def _hold_and_place(self, photo=True):
        """Hold here: mission nodes frozen, mushrooms in, photo taken."""
        self._hold_robots()
        self.launcher.mark_ready()
        self.phase = 'ready'
        self.link.publish_command({'cmd': 'prepare', 'stage': 'ready'})
        self.placed = self._place_mushrooms(self.config.get('points') or [])
        self.ready.set_ready(self.placed)
        if photo:
            self.take_photo()

    def _place_mushrooms(self, points):
        """Put `points` in the world, clearing whatever is there first.

        A world that ships its own mushrooms is the awkward case: its models
        are in the .sdf, so using them means spawning nothing, and using
        anything else means taking them out first or the quadcopter finds two
        sets and the counter only knows about one.
        """
        world = self.config.get('world', '')
        if self.config.get('clear_world') and not self.launcher.clear_world_mushrooms():
            self._log("could not identify the world's own mushrooms to remove them - "
                      "using them as they are")
            self.config['spawn'] = False
        if not self.config.get('spawn', True):
            return world_mushrooms(world)
        return self.launcher.spawn_mushrooms(points)

    def shuffle_mushrooms(self):
        """Move the mushrooms and look again. Only valid before launch."""
        if self.phase != 'ready':
            return
        world = self.config.get('world', '')
        count = len(self.placed) or int(self.config.get('count', 4) or 4)
        points = random_placement(world, count)
        if not points:
            self._log('could not find anywhere clear to put them in this world')
            return
        if not self.config.get('spawn', True):
            # Switching off the world's own set for the first time.
            if not self.launcher.clear_world_mushrooms():
                self._log("this world's mushrooms are part of its world file and "
                          "cannot be moved - pick another layout on the setup page")
                return
            self.config['spawn'] = True
        self.config['points'] = points
        self.placed = self.launcher.spawn_mushrooms(points)
        self.ready.set_points(self.placed)
        self.take_photo()

    # -- the top-down photo -------------------------------------------------
    def take_photo(self):
        if self.phase != 'ready':
            return
        self._photo_wait = True
        self.ready.photo.set_note('adding a camera above the world...')
        self.ready.photo.clear()
        if not self.launcher.start_topdown():
            self._photo_wait = False
            self.ready.photo_failed('Gazebo would not add the camera. The drawing '
                                    'below is the same information.')
            return
        QTimer.singleShot(int(PHOTO_WAIT_S * 1000), self._photo_timeout)

    def on_topdown(self, image):
        if not self._photo_wait:
            return
        self._photo_wait = False
        self.ready.set_photo(image)
        # One frame is all we wanted; leaving the camera there costs frames
        # the two real cameras need.
        self.launcher.stop_topdown()

    def _photo_timeout(self):
        if not self._photo_wait:
            return
        self._photo_wait = False
        self.launcher.stop_topdown()
        self.ready.photo_failed(
            f'No frame from Gazebo in {PHOTO_WAIT_S:.0f} s - headless or software '
            'rendering usually means no camera at all. The drawing below is the '
            'same information.')

    def back_to_setup(self):
        """Leave the simulation up and go back to the setup page."""
        if self._photo_wait:
            self._photo_wait = False
            self.launcher.stop_topdown()
        self.probe.stop()
        if self.phase == 'preparing':
            # A half-started simulation is not worth keeping. The hold goes
            # with it; while the simulation stays up the hold stays on, which
            # is what keeps the robots still on the setup page.
            self.link.set_holding(False)
            self.launcher.shutdown()
        self.phase = 'setup' if not self.launcher.prepared() else 'ready'
        self.link.publish_command({'cmd': 'prepare', 'stage': 'idle'})
        self.home.set_note('The simulation is still up - Prepare again is instant '
                           'unless you change world.'
                           if self.launcher.prepared() else '')
        self.stack.setCurrentWidget(self.home)

    # -- stage two: launch --------------------------------------------------
    def launch_mission(self):
        # Launching from the middle of the checklist is allowed once the page
        # has offered it - something that was never going to arrive should not
        # cost you the run. It still goes through the hold, so the mushrooms
        # are spawned and the photo attempted exactly as they would have been.
        if self.phase == 'preparing':
            self.probe.stop()
            self._hold_and_place(photo=False)   # about to launch; no time to look
        # A camera still hanging over the world costs frames the two real
        # cameras need, and nobody is looking at the photo now.
        self._photo_wait = False
        self.launcher.stop_topdown()
        # The hold has to stop before the nodes do anything, or our zeros and
        # their commands take turns and the robots twitch.
        self.link.set_holding(False)
        if not self.launcher.launch():
            self._log('nothing to launch - the simulation is not held and ready')
            return
        config = self.config
        world = config.get('world', 'simple_trees')
        config['trunks'] = world_meta(world)[0]
        config['total'] = len(self.placed)

        self.link.publish_command({
            'cmd': 'start',
            'mode': config.get('mode', 'autonomous'),
            'world': world,
            'layout': config.get('layout', ''),
            'total': config['total'],
        })
        self.phase = 'running'
        self.mission_running = True
        self.flown_by_hand.clear()
        self.operator.start_mission(config)
        self.stack.setCurrentWidget(self.operator)

    def finish_mission(self):
        self.link.cancel_all_nav()
        self.link.publish_command({'cmd': 'end'})
        self.link.publish_pause(False)
        self.active_goal = None
        self.flown_by_hand.clear()
        # Zeros first, then freeze. Both robots were moving a moment ago and
        # Gazebo's VelocityControl holds its last command forever, so freezing
        # the nodes on their own would leave two robots coasting across a
        # simulation that is meant to be sitting still waiting for the next
        # run. The hold stays on for exactly that reason.
        self.link.set_holding(True)
        # Keeps Gazebo: the next run on the same world needs no warm-up.
        self.launcher.end_mission(keep_sim=True)
        if not self.launcher.prepared():
            self.link.set_holding(False)
        self._held_said = False
        self.mission_running = False
        self.placed = []
        self.phase = 'ready' if self.launcher.prepared() else 'setup'
        self.operator.stop_mission()
        self.home.set_note('Mission ended. The simulation is still up - Prepare '
                           'again to run the same world straight away.'
                           if self.launcher.prepared() else 'Mission ended.')
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
            self.launcher.shutdown()
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

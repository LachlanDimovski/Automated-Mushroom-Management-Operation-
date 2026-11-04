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

**Launch** releases them and starts the clock, so the first second of the
mission is a second in which something actually happens.

Holding a robot still is done with the robots' own `teleop_active`
stand-down, the same heartbeat the PS4 controller uses: while it keeps
arriving, `rover_capture_node` and `zcam_quad_node` stop commanding their
robot and, when it stops, re-plan from where they were left. That is how
Prepare, Pause, Abort and hand-driving all hold a robot, and it is much
better than stopping the process: a standing-down node keeps its timers, its
clock and its Nav2 connection, and the quad goes on reporting sightings to
the rover the whole time.

It does not stop a robot on its own, though. Gazebo's VelocityControl keeps
the last command it was given forever, so whoever holds the heartbeat also
publishes the zeros.

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
from ammo_gui_node.mission_launcher import (MissionLauncher, place_people,
                                            world_meta, world_mushrooms)
# aliased: this module has a slot of the same name, and a bare call that
# resolved to the module function by luck is not worth reading twice
from ammo_gui_node.mission_launcher import shuffle_mushrooms as random_placement
from ammo_gui_node.operator_page import OperatorPage
from ammo_gui_node.ready_page import ReadyPage
from ammo_gui_node.ros_bridge import RosLink

HEALTH_MS = 2000
ABORT_WATCH_MS = 250   # how often an abort checks whether everything stopped
# Twice a second, not once: the gap between the mission nodes appearing and
# the quadcopter leaving the ground is only a few seconds, and that gap is
# what the prepare stage has to get inside.
PROBE_MS = 500
PHOTO_WAIT_S = 15.0   # per attempt; software rendering is slow

# Above this the quadcopter counts as off the ground. Their own landing test
# is tighter (`LANDED_AGL_M`); this is the other direction - high enough that
# ground noise in the odometry altitude cannot read as a take-off.
AIRBORNE_M = 0.6
PHOTO_TRIES = 4       # ...so keep waiting a full minute before calling it


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
        self.standing = []            # ...and where the people are
        self.phase = 'setup'          # setup, preparing, ready, running
        self.active_goal = None
        self.mission_running = False
        self.flown_by_hand = set()
        self._airborne_said = False   # said the quad left the ground, once per run
        self._touchdown_said = False  # said it came back down, once per abort
        self.rover_at = None          # last rover pose, for aiming operator goals
        self._prepare_t0 = 0.0
        self._photo_wait = False
        self._photo_tries = 0
        self._held_said = False       # the hold has been announced on the page
        self.aborting = False
        self.abort_done = False
        self._abort_t0 = 0.0
        self._still_since = None
        self.status = {}

        self.home.prepare_requested.connect(self.begin_prepare)
        self.home.kill_strays_requested.connect(self.kill_strays)
        self.ready.launch_requested.connect(self.launch_mission)
        self.ready.shuffle_requested.connect(self.shuffle_mushrooms)
        self.ready.photo_requested.connect(lambda: self.take_photo(retake=True))
        self.ready.back_requested.connect(self.back_to_setup)
        self.operator.end_mission.connect(self.finish_mission)
        self.operator.pause_toggled.connect(self.set_paused)
        self.operator.goal_clicked.connect(self.send_goal)
        self.operator.teleop_armed.connect(self.set_hand_flown)
        self.operator.abort_requested.connect(self.abort)
        self.operator.resume_requested.connect(self.resume_from_abort)
        self.operator.mark_requested.connect(self.mark_for_capture)
        self.operator.queue_clicked.connect(self.queue_waypoint)

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
        self.link.fleet_frame.connect(self.operator.on_fleet_frame)
        self.link.fleet_pose.connect(self.operator.on_fleet_pose)
        self.link.robot_state.connect(self.operator.on_robot_state)
        self.link.status_updated.connect(self._note_status)

        # The launch runs detached with its output in a file, so the only way
        # to notice it died - Gazebo failing to open a window, most often - is
        # to look.
        self.health = QTimer(self.stack)
        self.health.timeout.connect(self._check_health)
        self.health.start(HEALTH_MS)

        # Only runs during the prepare stage.
        self.probe = QTimer(self.stack)
        self.probe.timeout.connect(self._probe_tick)

        # Only runs between an abort and everything coming to rest.
        self.abort_watch = QTimer(self.stack)
        self.abort_watch.timeout.connect(self._abort_tick)

        self.link.start()

    def _log(self, message):
        self.home.set_note(message)
        if self.phase in ('preparing', 'ready'):
            self.ready.note.setText(message)

    def _note_rover_pose(self, x, y, _yaw):
        self.rover_at = (x, y)

    def _note_status(self, status):
        self.status = status
        self._watch_takeoff()

    def _watch_takeoff(self):
        """Say, once, when the quadcopter has actually left the ground.

        "Launch pressed" and "the mission is under way" are not the same
        moment and the gap between them is confusing: the quad is released
        and then spends the better part of a minute climbing to cruise
        height before it flies its first leg, which from the mission page
        looks exactly like nothing happening. So this calls it out, and the
        same line doubles as proof the stand-down was released - a quad that
        never leaves the ground is a quad still being held.
        """
        if not self.mission_running or self.aborting or self._airborne_said:
            return
        agl = self.link.quad_height()
        if agl is None:
            agl = (self.status.get('quad') or {}).get('alt')
        try:
            agl = float(agl)
        except (TypeError, ValueError):
            return
        if agl < AIRBORNE_M:
            return
        self._airborne_said = True
        self.link.publish_command(
            {'cmd': 'note',
             'text': f'Quadcopter airborne at {agl:.1f} m - climbing to cruise '
                     f'height before its first leg'})

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
        quads = int(config.get('quads', 1) or 1)
        rovers = int(config.get('rovers', 1) or 1)
        self.link.set_fleet(quads, rovers)
        # Hold every robot from this moment, before a single node exists. The
        # heartbeat is harmless until something subscribes to it, and holding
        # from the start is what stops the quadcopter taking off during the
        # thirty seconds the checklist takes - there is no window to lose.
        self.link.set_standdown('all', True)
        self.link.set_holding(True)
        # Everything the checklist knows is from the last run. Forget it, or a
        # second Prepare ticks the camera and telemetry boxes from messages
        # the previous simulation sent and offers a Launch that cannot work.
        self.link.reset_readiness()
        started = self.launcher.prepare(world,
                                        show_sim=config.get('show_sim', True),
                                        options=config.get('options'),
                                        tool=config.get('tool', 'gripper'),
                                        quads=quads, rovers=rovers,
                                        # > 0 turns the rover's person
                                        # detector and the hazard brake on
                                        people=len(config.get('people') or []))
        if started is None or started == 'blocked':
            self.link.set_standdown('all', False)
            self.link.set_holding(False)
            # 'blocked' means something else is already running. That used to
            # be a quiet note at the bottom of the setup page, which reads
            # exactly like the button not working.
            self.home.set_blocked(started == 'blocked')
            return
        self.home.set_blocked(False)
        self.phase = 'preparing'
        self.placed = []
        self.standing = []
        self._prepare_t0 = time.monotonic()
        self._held_said = False
        self.ready.begin(config)
        self.ready.set_held()
        self._held_said = True
        self.stack.setCurrentWidget(self.ready)
        self.link.publish_command({'cmd': 'prepare', 'stage': 'preparing'})
        self.probe.start(PROBE_MS)

    def kill_strays(self):
        """Stop a simulation the GUI does not own, after the operator asks."""
        killed = self.launcher.kill_strays()
        self.home.set_blocked(False)
        self._log('stopped what was running - give it a few seconds, then '
                  'press Prepare again' if killed else
                  'nothing left to stop - press Prepare again')

    def _probe_tick(self):
        """One look at the simulation, folded into the checklist."""
        if self.phase != 'preparing':
            self.probe.stop()
            return
        probe = self.link.probe_readiness()
        # The one check the ROS graph cannot answer: our own child processes.
        probe['nodes'] = self.launcher.mission_nodes_up()
        if self.ready.apply_probe(probe, time.monotonic() - self._prepare_t0):
            self.probe.stop()
            self._hold_and_place()

    def _hold_robots(self):
        """Hold every robot of this run still, and keep it held."""
        self.link.set_standdown('all', True)
        self.link.set_holding(True)
        if not self._held_said:
            self._held_said = True
            self.ready.set_held()

    def _hold_and_place(self, photo=True):
        """Hold here: mission nodes frozen, mushrooms in, photo taken."""
        self._hold_robots()
        self.launcher.mark_ready()
        self.phase = 'ready'
        self.link.publish_command({'cmd': 'prepare', 'stage': 'ready'})
        self.placed = self._place_mushrooms(self.config.get('points') or [])
        # People go in with the mushrooms, under the same hold, so nothing has
        # driven at one before the run starts.
        self.standing = self.launcher.spawn_people(self.config.get('people') or [])
        self.ready.set_ready(self.placed, people=self.standing)
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
        self.ready.shuffling()
        points = random_placement(world, count)
        if not points:
            self.ready.photo_failed('Could not find anywhere clear to put them in '
                                    'this world - try fewer mushrooms.')
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
        # The first person stands on the way to the nearest mushroom, so
        # moving the mushrooms has to move them too - otherwise a shuffle
        # leaves somebody standing in the middle of nowhere.
        if self.config.get('people'):
            self.config['people'] = place_people(
                world, len(self.config['people']), points)
            self.standing = self.launcher.spawn_people(self.config['people'])
        self.ready.set_points(self.placed, people=self.standing)
        if len(self.placed) < count:
            self._log(f'only found room for {len(self.placed)} of {count} '
                      f'- {world} is crowded')
        self.take_photo(retake=True)

    # -- the top-down photo -------------------------------------------------
    def take_photo(self, retake=False):
        """Spawn the overhead camera if it is not up, and wait for a frame.

        The first frame is genuinely slow: the model has to spawn, the bridge
        process has to start, and under software rendering the first render
        of a 640 px shot from thirty metres up can take a long while. The
        first version gave up after one timeout and said so, which is why
        pressing Retake appeared to be the only thing that worked - by then
        the camera was already there and only the waiting had to start again.
        """
        if self.phase != 'ready':
            return
        self._photo_wait = True
        self._photo_tries = 0
        fresh = not self.launcher.topdown_running()
        self.ready.photo_busy('Retaking the picture' if retake else
                              ('Adding a camera above the world' if fresh else
                               'Waiting for the first frame'))
        if not self.launcher.start_topdown():
            self._photo_wait = False
            self.ready.photo_failed('Gazebo would not add the camera. The drawing '
                                    'below is the same information.')
            return
        QTimer.singleShot(int(PHOTO_WAIT_S * 1000), self._photo_timeout)

    def on_topdown(self, image):
        # The camera stays up for the whole run now, so every frame goes to
        # the mission page's overhead panel. The ready page only wants the
        # first one, to show what the spawn looks like.
        self.operator.on_topdown_frame(image)
        if not self._photo_wait:
            return
        self._photo_wait = False
        self.ready.set_photo(image)

    def _photo_timeout(self):
        if not self._photo_wait:
            return
        self._photo_tries += 1
        if self._photo_tries < PHOTO_TRIES:
            # Still waiting, not failed. Say how long it has been so a slow
            # machine looks slow rather than broken.
            waited = PHOTO_WAIT_S * self._photo_tries
            self.ready.photo_busy(
                f'Still waiting for the first frame ({waited:.0f} s) - software '
                'rendering is slow')
            QTimer.singleShot(int(PHOTO_WAIT_S * 1000), self._photo_timeout)
            return
        self._photo_wait = False
        # The camera itself is left running: if a frame turns up late it
        # still has somewhere useful to go, and it is the mission page's
        # overhead feed either way.
        self.ready.photo_failed(
            f'No frame from Gazebo in {PHOTO_WAIT_S * PHOTO_TRIES:.0f} s. Headless '
            'or software rendering often means no camera at all - the drawing '
            'below is the same information. Retake tries again.')

    def back_to_setup(self):
        """Stop everything and go back to the setup page.

        Being on the setup page means nothing is running. Full stop. It used
        to leave the simulation up so that preparing again on the same world
        was instant, and that one optimisation is behind most of the
        confusing behaviour of the last three runs: stale latched topics
        feeding panels for robots that were gone, a leftover `ign gazebo`
        making the next Prepare refuse quietly so the button looked dead, and
        a Gazebo window that would not close. A slow second Prepare is a far
        smaller cost than a simulation nobody can account for.
        """
        if self._photo_wait:
            self._photo_wait = False
            self.launcher.stop_topdown()
        self.probe.stop()
        self.link.set_holding(False)
        self.link.set_standdown('all', False)
        self.launcher.shutdown()
        self.placed = []
        self.standing = []
        self.phase = 'setup'
        self.link.publish_command({'cmd': 'prepare', 'stage': 'idle'})
        self.home.set_blocked(False)
        self.home.set_note('Simulation stopped. Preparing again starts a fresh '
                           'one, so give it the usual half minute.')
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
        # The photo is taken; the camera stays up as the fourth live feed.
        self._photo_wait = False
        # The hold has to stop before the nodes do anything, or our zeros and
        # their commands take turns and the robots twitch. Zeros first, then
        # the heartbeat, so nothing is driven in the gap between.
        self.link.set_holding(False)
        self.link.set_standdown('all', False)
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
            # Where they actually are. Only the GUI knows - it spawned them -
            # and the Debug UI needs it to reveal them on the overhead view
            # and to tag them in the end-of-mission report.
            'placed': [[round(float(x), 2), round(float(y), 2)]
                       for x, y in self.placed],
            # Same reasoning for the people: the GUI stood them there, so it
            # is the only thing that knows where they are. The detector finds
            # them; this is what the map draws before it does.
            'people': [[round(float(x), 2), round(float(y), 2)]
                       for x, y in self.standing],
            'quads': self.launcher.quads,
            'rovers': self.launcher.rovers,
        })
        self.phase = 'running'
        self.mission_running = True
        self._airborne_said = False
        self.flown_by_hand.clear()
        # The robots were released a few lines above. Saying so in the feed
        # separates "Launch did nothing" from "Launch worked and the quad is
        # still climbing", which from the mission page look identical.
        self.link.publish_command(
            {'cmd': 'note',
             'text': 'Mission started - every robot released from the hold'})
        config['quads'] = self.launcher.quads
        config['rovers'] = self.launcher.rovers
        self.operator.set_topdown_visible(self.launcher.topdown_running())
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
        self.link.set_standdown('all', True)
        self.launcher.stop_topdown()
        # Everything goes: the simulation outliving its mission is what left
        # phantom robots on the cameras, a coverage figure frozen at a number
        # from a dead run, and a Prepare that refused to start.
        self.launcher.end_mission(keep_sim=False)
        self.link.set_holding(False)
        self.link.set_standdown('all', False)
        self._held_said = False
        self.aborting = self.abort_done = False
        self.abort_watch.stop()
        self.mission_running = False
        self.placed = []
        self.standing = []
        self.phase = 'ready' if self.launcher.prepared() else 'setup'
        self.operator.stop_mission()
        self.home.set_note('Mission ended and the simulation is closed. The '
                           'Debug UI is holding the report until you prepare '
                           'the next run.')
        self.stack.setCurrentWidget(self.home)

    # -- abort --------------------------------------------------------------
    def abort(self):
        """Emergency stop: everything halts, the quadcopters land where they are.

        Built out of the robots' own stand-down rather than around it. Holding
        `teleop_active` is exactly the state in which a quad expects somebody
        else to be flying it, so descending it on `parrot1/cmd_vel` while the
        heartbeat is held is their sanctioned manual path, not a fight with
        their controller.

        Their `ammo/mission_complete` would also land a quad - properly, with
        avoidance - but it flies home first, which is not what an emergency
        stop should do. The Return Home button on the mission page is that
        one, for when there is no emergency.
        """
        if not self.mission_running or self.aborting:
            return
        self.aborting, self.abort_done = True, False
        self._abort_t0 = time.monotonic()
        self._still_since = None
        self._touchdown_said = False
        self.link.cancel_all_nav()
        self.link.set_holding(True)        # zeros to everything
        self.link.set_standdown('all', True)
        self.link.set_descending(True)     # ...except the quad, which flies down
        self.active_goal = None
        self.link.publish_command({'cmd': 'abort'})
        self.operator.set_aborting(True)
        self.abort_watch.start(ABORT_WATCH_MS)

    def _abort_tick(self):
        """Watch until everything has actually come to rest, then say so."""
        if not self.aborting or self.abort_done:
            self.abort_watch.stop()
            return
        elapsed = time.monotonic() - self._abort_t0
        rover = (self.status.get('rover') or {})
        rover_still = float(rover.get('speed', 0.0) or 0.0) <= C.ABORT_STILL_MS
        agl = self.link.quad_height()
        # No agl at all means the quad never published one; the odometry
        # altitude in the status is the fallback rather than waiting forever.
        if agl is None:
            agl = float((self.status.get('quad') or {}).get('alt', 0.0) or 0.0)
        quad_down = agl <= C.LANDED_AGL_M
        if quad_down and not self._touchdown_said:
            # The settle confirmation below waits for the rover too and for
            # ABORT_SETTLE_S on top, so without this the moment the quad
            # actually reaches the ground is never reported - which is the
            # one thing you want confirmed after an emergency stop.
            self._touchdown_said = True
            self.link.publish_command(
                {'cmd': 'note',
                 'text': f'Quadcopter has touched down at {agl:.2f} m above '
                         f'ground after {elapsed:.0f} s'})
        if rover_still and quad_down:
            if self._still_since is None:
                self._still_since = time.monotonic()
            if time.monotonic() - self._still_since >= C.ABORT_SETTLE_S:
                self._finish_abort('All robots stopped - quadcopter down')
                return
        else:
            self._still_since = None
        if elapsed >= C.ABORT_TIMEOUT_S:
            left = []
            if not rover_still:
                left.append('the skidsteer is still rolling')
            if not quad_down:
                left.append(f'the quadcopter is {agl:.1f} m up')
            self._finish_abort('Stopped waiting after %.0f s - %s'
                               % (elapsed, ' and '.join(left) or 'unclear'))

    def _finish_abort(self, text):
        self.abort_done = True
        self.abort_watch.stop()
        self.link.set_descending(False)
        self.link.publish_command({'cmd': 'abort_done', 'text': text})
        self.operator.set_abort_done(text)

    def resume_from_abort(self):
        """Hand the robots back and start the clock again."""
        if not self.aborting:
            return
        self.aborting = self.abort_done = False
        self.abort_watch.stop()
        self.link.set_descending(False)
        self.link.set_holding(False)
        self.link.set_standdown('all', False)
        for robot in self.flown_by_hand:
            self.link.set_standdown(robot, True)
        self.link.publish_command({'cmd': 'abort_clear'})
        self.operator.set_aborting(False)

    # -- marking a mushroom for the rover -------------------------------------
    def mark_for_capture(self):
        """Ask the quadcopter to send what it can see to the rover's queue.

        The quad answers with its own most recent gated sighting and its own
        freshness check, so the projection is theirs. Nothing here guesses a
        position - which matters, because their projection is the part of
        this system with the most hard-won detail in it.
        """
        if not self.mission_running:
            return
        if not self.link.publish_mark():
            self._log('could not reach the quadcopter to mark a mushroom')
            return
        # Their quad only keeps a gated sighting once it has taken off
        # (`if msg.detections and self.taken_off` in `_det_cb`), so a mark
        # sent from the ground is answered with nothing at all. A box drawn
        # on the camera feed is not the same thing as a sighting the quad is
        # willing to hand over, and that difference is invisible from here.
        agl = self.link.quad_height()
        if agl is None:
            agl = (self.status.get('quad') or {}).get('alt')
        try:
            grounded = float(agl) < AIRBORNE_M
        except (TypeError, ValueError):
            grounded = False
        if grounded:
            self.link.publish_command(
                {'cmd': 'note',
                 'text': 'Marked, but the quadcopter is on the ground - it only '
                         'keeps a sighting it can hand over once it has taken '
                         'off, so nothing will reach the skidsteer. Fly up first.'})
            return
        self.link.publish_command(
            {'cmd': 'marked',
             'text': 'Operator marked the mushroom in the quadcopter\'s view'})

    def queue_waypoint(self, x, y):
        """Queue a point for the rover to collect, straight from the map."""
        if not self.mission_running:
            return
        self.link.publish_waypoint(x, y)
        self.link.publish_command(
            {'cmd': 'marked', 'x': x, 'y': y,
             'text': f'Operator queued ({x:.1f}, {y:.1f}) for the rover'})

    # -- pause --------------------------------------------------------------
    def set_paused(self, paused):
        if paused:
            # Order matters. Cancel first so Nav2 stops driving, start the
            # zero-velocity hold so nothing coasts on its last command, and
            # hold the nodes down last.
            self.link.cancel_all_nav()
            self.link.set_holding(True)
            self.link.set_standdown('all', True)
        else:
            self.link.set_holding(False)
            self.link.set_standdown('all', False)
            # a robot being hand-flown goes back to being hand-flown, not
            # back to its mission node
            for robot in self.flown_by_hand:
                self.link.set_standdown(robot, True)
            if self.active_goal:
                # Our own Nav2 goal was cancelled by the pause; put it back.
                self.link.send_nav_goal(*self.active_goal)
        self.link.publish_pause(paused)

    # -- hand flying --------------------------------------------------------
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
            self.link.set_standdown(robot, True)
            self.link.publish_command(
                {'cmd': 'note', 'text': f'{robot} taken over by the operator'})
        else:
            self.flown_by_hand.discard(robot)
            self.link.set_standdown(robot, False)
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
        self.link.set_standdown('rover', True)
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
            self._goal_failed(f'Nav2 would not take the goal at ({x:.1f}, {y:.1f}) '
                              '- is it up yet?')
            return
        self.link.publish_command({'cmd': 'operator_goal', 'x': x, 'y': y})

    def _goal_failed(self, why):
        self.active_goal = None
        self._log(why)
        if 'rover' not in self.flown_by_hand:
            self.link.set_standdown('rover', False)
        self.link.publish_command({'cmd': 'operator_goal_done', 'ok': False, 'why': why})

    def on_goal_finished(self, ok, why=''):
        if not self.active_goal:
            return
        self.active_goal = None
        if 'rover' not in self.flown_by_hand:
            self.link.set_standdown('rover', False)
        # Distinguish the two: a rejected or aborted goal used to be reported
        # as "reached the goal", which is how a rover that never moved could
        # still claim to have arrived.
        self.link.publish_command({'cmd': 'operator_goal_done', 'ok': bool(ok),
                                   'why': why or ''})

    def clear_goal(self):
        if not self.active_goal:
            return
        self.active_goal = None
        self.link.cancel_all_nav()
        if 'rover' not in self.flown_by_hand:
            self.link.set_standdown('rover', False)
        self.link.publish_command({'cmd': 'operator_goal_done', 'ok': None})

    # -- teardown -----------------------------------------------------------
    def quit(self):
        try:
            if self.mission_running:
                self.link.publish_command({'cmd': 'end'})
            self.link.set_holding(False)
            self.link.set_standdown('all', False)
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

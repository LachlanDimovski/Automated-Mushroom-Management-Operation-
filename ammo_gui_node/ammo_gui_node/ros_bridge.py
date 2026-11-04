"""The ROS side of both windows, wrapped so Qt can use it safely.

rclpy spins on a background daemon thread and Qt keeps the main thread -
the same split `ammo_dashboard` uses, and the only arrangement that stops a
slow repaint from backing up the subscriptions. The handoff is Qt signals:
because this QObject is constructed on the GUI thread, emitting from the
ROS thread queues the call onto the GUI thread automatically, so no slot in
either window ever touches a ROS callback's thread.

`role` decides how much of this is wired up. The Debug UI takes 'debug' and
gets subscriptions only, so opening or closing it can never disturb a
running mission.
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from PySide6.QtCore import QObject, Signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from ammo_gui_node import contract as C
from ammo_gui_node.image_utils import qimage_from_ros

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
EVENT_QOS = QoSProfile(depth=1000,
                       history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)

LATCHED_TF_QOS = QoSProfile(depth=100,
                            history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

HOLD_HZ = 20.0   # rate we publish zero velocity at while paused
IMAGE_HZ = 5.0   # camera frames converted per feed; the rest are dropped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# How recently a camera frame or an odometry message must have arrived for
# that check to count as live. The slowest of them runs at a few hertz, so
# six seconds is far longer than a healthy feed ever goes quiet, and short
# enough that a dead one is noticed within a poll or two.
FRESH_S = 6.0

class _LinkNode(Node):
    """The actual ROS node. Everything here runs on the spin thread."""

    def __init__(self, link, role):
        super().__init__(f'ammo_{role}_link')
        self.link = link
        self.role = role
        self._last_image = {}
        # {id(signal): {source: ([box], at)}} - see `_merged`
        self._boxes = {}
        # When each thing on the readiness checklist last arrived. A frame or
        # a message arriving is a much better answer to "is the camera
        # working" than a publisher existing on the topic.
        #
        # It is a time and not a flag on purpose. "Arrived at least once" was
        # the original test and it was wrong twice over: the set was built
        # when this node started and never cleared, so every Prepare after
        # the first ticked these boxes before the new simulation had produced
        # anything at all - the checklist said a world was ready that had not
        # even finished loading - and a camera that died mid-run went on
        # reading as live for the rest of the session.
        self.seen = {}
        self._tf_ok = False

        self.create_subscription(String, C.STATUS_TOPIC, self._on_status, LIVE_QOS)
        self.create_subscription(String, C.EVENTS_TOPIC, self._on_event, EVENT_QOS)
        self.create_subscription(Bool, C.PAUSE_TOPIC, self._on_pause, 10)
        self.create_subscription(Odometry, C.HUSKY_ODOM, self._on_rover_odom, LIVE_QOS)
        self.create_subscription(Odometry, C.PARROT_ODOM, self._on_quad_odom, LIVE_QOS)
        self.create_subscription(String, C.TELEOP_STATUS, lambda m: link.teleop_status.emit(m.data), QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Raw feeds drive the live panels; the annotated ones are only
        # subscribed so a frame is available the moment a sighting lands.
        # yolo_node publishes its annotated image lazily - only while
        # something is subscribed - so these subscriptions are what turn it on.
        self.create_subscription(Image, C.HUSKY_CAM,
                                 lambda m: self._image(m, self.link.rover_frame,
                                                       'rover_cam'), LIVE_QOS)
        self.create_subscription(Image, C.PARROT_CAM,
                                 lambda m: self._image(m, self.link.quad_frame,
                                                       'quad_cam'), LIVE_QOS)
        self.create_subscription(Image, C.HUSKY_CAM_YOLO,
                                 lambda m: self._image(m, self.link.rover_yolo_frame), LIVE_QOS)
        self.create_subscription(Image, C.PARROT_CAM_YOLO,
                                 lambda m: self._image(m, self.link.quad_yolo_frame), LIVE_QOS)

        # The boxes themselves, so the windows can draw them over the raw feed
        # at the camera's rate instead of showing the detector's 2 Hz image.
        try:
            from vision_msgs.msg import Detection2DArray
            self.create_subscription(Detection2DArray, C.HUSKY_DETECTIONS,
                                     lambda m: self._detections(m, self.link.rover_detections),
                                     LIVE_QOS)
            self.create_subscription(Detection2DArray, C.PARROT_DETECTIONS,
                                     lambda m: self._detections(m, self.link.quad_detections),
                                     LIVE_QOS)
            # The person detector's boxes, on the same camera tile as the
            # mushroom detector's. Two models on one picture, so they are
            # merged rather than one replacing the other - see `_merged`.
            self.create_subscription(Detection2DArray, C.HUSKY_HAZARDS,
                                     lambda m: self._detections(m, self.link.rover_detections,
                                                                source='hazard'),
                                     LIVE_QOS)
        except ImportError:
            self.get_logger().warn('vision_msgs not available - no boxes on the camera feeds')

        # One frame from the camera mission_launcher spawns over the world
        # while a run is being prepared. Subscribed always: the topic simply
        # has no publisher until that camera exists.
        self.create_subscription(Image, C.TOPDOWN_TOPIC, self._on_topdown, LIVE_QOS)

        # Every other robot a fleet could hold. Subscribed up front rather
        # than created once the counts are known: a subscription to a topic
        # nobody publishes carries no traffic, and creating subscriptions
        # while the executor is already spinning is not worth the risk.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for i in range(2, C.MAX_QUADS + 1):
            key = f'quad{i}'
            self.create_subscription(
                Image, C.quad_topic(i, 'camera/image'),
                lambda m, k=key: self._fleet_image(m, k), LIVE_QOS)
            self.create_subscription(
                Odometry, C.quad_topic(i, 'odometry'),
                lambda m, k=key: self._fleet_odom(m, k), LIVE_QOS)
            self.create_subscription(
                String, C.quad_state_topic(i),
                lambda m, k=key: self._fleet_state(m, k), latched)
        for i in range(2, C.MAX_ROVERS + 1):
            key = f'rover{i}'
            self.create_subscription(
                Image, C.rover_topic(i, 'camera/image'),
                lambda m, k=key: self._fleet_image(m, k), LIVE_QOS)
            self.create_subscription(
                Odometry, C.rover_topic(i, 'odometry'),
                lambda m, k=key: self._fleet_odom(m, k), LIVE_QOS)

        # Height above ground, published by the quad itself. Odometry z is
        # height above the world origin, which is not the same thing on a
        # heightmap world.
        from std_msgs.msg import Float32
        self.create_subscription(Float32, C.PARROT_AGL, self._on_agl, LIVE_QOS)
        self.create_subscription(Bool, C.PARROT_LANDED, self._on_landed,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # The rover's map frame is the one detail that quietly breaks operator
        # goals, so the checklist waits to actually see it rather than
        # assuming Nav2 being up implies it.
        try:
            from tf2_msgs.msg import TFMessage
            for topic in (C.ROVER_TF_TOPIC, C.GLOBAL_TF_TOPIC):
                self.create_subscription(TFMessage, topic, self._on_tf, LIVE_QOS)
            for topic in (C.ROVER_TF_STATIC, C.GLOBAL_TF_STATIC):
                self.create_subscription(TFMessage, topic, self._on_tf, LATCHED_TF_QOS)
        except ImportError:
            self._tf_ok = None       # unknown rather than missing

        self.pause_pub = self.command_pub = self.waypoint_pub = None
        self.hold_timer = None
        self.cmd_pubs = []
        self.teleop_pub = None
        self.quad_teleop_pub = self.mark_pub = None
        self.fleet_teleop = {}
        self.fleet_cmd = {}
        self.fleet_mark = {}
        self.standdown_timer = None
        # How many of each robot this run has. Set from the setup page's
        # counts at Prepare, so nothing has to be discovered.
        self.quads, self.rovers = 1, 1
        # Which robots the GUI is currently holding still. A set rather than a
        # flag because Prepare holds both, hand-driving holds one, and an
        # abort holds both again for a different reason.
        self._standdown = set()
        self._descending = False
        self.agl = None
        self.agl_at = 0.0          # when that reading arrived
        self.quad_z = None         # odometry height, the landing fallback
        self._descend_from = 0.0   # when the abort descent started
        self.nav_client = None
        self.cancel_client = None
        self._nav_handle = None
        self._holding = False

        if role == 'gui':
            self.pause_pub = self.create_publisher(Bool, C.PAUSE_TOPIC, 10)
            self.command_pub = self.create_publisher(String, C.COMMAND_TOPIC, 10)
            self.waypoint_pub = self.create_publisher(PointStamped, C.CAGE_WAYPOINTS, 10)
            self.cmd_pubs = [self.create_publisher(Twist, t, 10)
                             for t in (C.HUSKY_CMD, C.HUSKY_CMD_RAW, C.PARROT_CMD)]
            # Their stand-down heartbeats, one per robot, and the quad's
            # "send what you can see to the rover" button.
            self.teleop_pub = self.create_publisher(Bool, C.HUSKY_TELEOP, 10)
            self.quad_teleop_pub = self.create_publisher(Bool, C.PARROT_TELEOP, 10)
            self.mark_pub = self.create_publisher(Bool, C.PARROT_MARK, 10)
            # Robots 2.. of a fleet. Publishers are cheap and silent until
            # something is published, so they are all made up front rather
            # than created while the executor is already spinning.
            self.fleet_teleop = {}
            self.fleet_cmd = {}
            self.fleet_mark = {}
            for i in range(2, C.MAX_QUADS + 1):
                self.fleet_teleop[f'quad{i}'] = self.create_publisher(
                    Bool, C.quad_topic(i, 'teleop_active'), 10)
                self.fleet_cmd[f'quad{i}'] = self.create_publisher(
                    Twist, C.quad_topic(i, 'cmd_vel'), 10)
                self.fleet_mark[f'quad{i}'] = self.create_publisher(
                    Bool, C.quad_topic(i, 'teleop_mark'), 10)
            for i in range(2, C.MAX_ROVERS + 1):
                self.fleet_teleop[f'rover{i}'] = self.create_publisher(
                    Bool, C.rover_topic(i, 'teleop_active'), 10)
                self.fleet_cmd[f'rover{i}'] = self.create_publisher(
                    Twist, C.rover_topic(i, 'cmd_vel_raw'), 10)
            self.fleet_teleop['quad1'] = self.quad_teleop_pub
            self.fleet_teleop['rover1'] = self.teleop_pub
            self.fleet_mark['quad1'] = self.mark_pub
            self.standdown_timer = self.create_timer(1.0 / C.TELEOP_HZ, self._standdown_tick)
            self.hold_timer = self.create_timer(1.0 / HOLD_HZ, self._hold_tick)
            self._setup_nav()

    # -- Nav2, used for operator goals and for stopping on pause ------------
    def _setup_nav(self):
        try:
            from action_msgs.srv import CancelGoal
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self.nav_client = ActionClient(self, NavigateToPose, 'husky1/navigate_to_pose')
            self.cancel_client = self.create_client(CancelGoal, C.NAV_CANCEL_SERVICE)
        except ImportError:
            self.get_logger().warn(
                'nav2_msgs/action_msgs not found - operator goals and pause-stop '
                'will not be able to talk to Nav2')

    def cancel_all_nav(self):
        """Stop whatever Nav2 is driving, whoever asked for it.

        A zero goal_id and a zero stamp in a CancelGoal request means "every
        goal you have", which is the only way to cancel a goal that another
        node started - and cage_capture_node's goals are exactly that.
        """
        if self.cancel_client is None or not self.cancel_client.service_is_ready():
            return False
        try:
            from action_msgs.srv import CancelGoal
            self.cancel_client.call_async(CancelGoal.Request())
            return True
        except Exception:
            return False

    def send_nav_goal(self, x, y, yaw=0.0):
        """An operator goal, built exactly the way the rover's own node builds one.

        Two details matter and both are copied from `rover_capture_node._nav_to`:
        the frame is `husky1_map`, not `map` - with truth_localizer running
        there is no bare `map` in the rover's TF tree - and the stamp is left
        empty, which tells Nav2 to use the latest transform rather than asking
        for one at an instant the buffer may not have. Getting either wrong
        gets the goal rejected, which looks from the outside like the rover
        turning on the spot and then giving up.
        """
        if self.nav_client is None or not self.nav_client.wait_for_server(timeout_sec=2.0):
            return False
        try:
            from nav2_msgs.action import NavigateToPose
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = C.ROVER_MAP_FRAME
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(self._nav_accepted)
            return True
        except Exception as exc:
            self.get_logger().warn(f'could not send the operator goal: {exc}')
            return False

    def _nav_accepted(self, future):
        try:
            handle = future.result()
        except Exception:
            return
        if handle is None or not handle.accepted:
            # Rejected before it started. Nav2 does this when the goal is
            # outside the global costmap or in a frame it cannot transform -
            # a different fault from giving up part-way, and the two used to
            # be reported with the same sentence.
            self.link.nav_goal_finished.emit(
                False, 'Nav2 would not accept the goal - it is outside the '
                       'costmap, or the rover has no transform for it yet')
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result)

    # action_msgs/GoalStatus, so a failure can say which kind it was rather
    # than offering the operator a list of things it might have been.
    NAV_STATUS = {
        2: 'the goal was cancelled while it was running - something else '
           'cancelled it, which on a run with people in it is usually the '
           'hazard brake',
        5: 'Nav2 gave up: no route it could follow, or it stopped making '
           'progress along the one it had',
        6: 'the goal was cancelled before it finished',
    }

    def _nav_result(self, future):
        ok, why = False, 'Nav2 ended the goal without saying how'
        try:
            result = future.result()
            status = getattr(result, 'status', 0)
            ok = status == 4  # STATUS_SUCCEEDED
            if not ok:
                why = self.NAV_STATUS.get(status, f'Nav2 ended with status {status}')
        except Exception as exc:
            why = f'the goal result could not be read: {str(exc)[:80]}'
        self._nav_handle = None
        self.link.nav_goal_finished.emit(ok, '' if ok else why)

    # -- the zero-velocity hold used while paused ---------------------------
    def set_holding(self, holding):
        self._holding = bool(holding)

    def _hold_tick(self):
        """While paused, we are the only thing publishing velocity.

        Both robots use Gazebo's VelocityControl plugin, which holds the
        last command it was given forever - there is no watchdog that zeroes
        it. So stopping the mission nodes is not enough on its own: somebody
        has to actively send zero, or the robots keep going at whatever they
        were last told. That is this timer.
        """
        if not self._holding:
            return
        stop = Twist()
        for i, pub in enumerate(self.cmd_pubs):
            if i == 2 and self._descending:
                continue      # the descent owns parrot1/cmd_vel while it runs
            pub.publish(stop)
        # Robots 2.. of a fleet. VelocityControl latches for each of them
        # separately, so each needs its own zero.
        for key in self.robot_keys():
            pub = self.fleet_cmd.get(key)
            if pub is not None:
                pub.publish(stop)

    # -- inbound ------------------------------------------------------------
    def _on_status(self, msg):
        try:
            self.link.status_updated.emit(json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def _on_event(self, msg):
        try:
            self.link.event_received.emit(json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def _on_pause(self, msg):
        self.link.pause_changed.emit(bool(msg.data))

    def _on_rover_odom(self, msg):
        self.seen['rover_odom'] = time.monotonic()
        p = msg.pose.pose.position
        self.link.rover_pose.emit(p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_quad_odom(self, msg):
        self.seen['quad_odom'] = time.monotonic()
        p = msg.pose.pose.position
        self.quad_z = float(p.z)
        self.link.quad_pose.emit(p.x, p.y, yaw_of(msg.pose.pose.orientation), p.z)

    def _merged(self, signal, source, boxes):
        """Every detector's current boxes for one camera, as one list.

        The rover has two models looking at the same picture - mushrooms from
        the mission weights, people from stock COCO - and they publish at
        different rates on different topics. Emitting whichever arrived last
        would make each set blink out whenever the other ticked. So each
        source's boxes are kept under its own key and the union is emitted,
        with a source that has gone quiet for longer than the panel would
        hold its boxes anyway dropped.
        """
        cache = self._boxes.setdefault(id(signal), {})
        cache[source] = (boxes, time.monotonic())
        now = time.monotonic()
        out = []
        for key, (saved, at) in list(cache.items()):
            if now - at > C.DET_HOLD_S:
                if key != source:
                    del cache[key]
                continue
            out += saved
        return out

    def _detections(self, msg, signal, source='mission'):
        """Flatten a Detection2DArray into what a camera panel needs to draw.

        The box is already in the source image's pixels, and the hypothesis
        carries the class name and the score - the same three things the
        annotated image shows, just as data.
        """
        boxes = []
        for det in msg.detections:
            best = None
            for res in det.results:
                score = float(getattr(res.hypothesis, 'score', 0.0))
                if best is None or score > best[1]:
                    best = (str(getattr(res.hypothesis, 'class_id', '')), score)
            if best is None:
                continue
            bbox = getattr(det, 'bbox', None)
            if bbox is None:
                continue
            try:
                boxes.append({'cx': float(bbox.center.position.x),
                              'cy': float(bbox.center.position.y),
                              'w': float(bbox.size_x), 'h': float(bbox.size_y),
                              'label': best[0], 'score': best[1]})
            except AttributeError:
                continue
        signal.emit(self._merged(signal, source, boxes))

    def _image(self, msg, signal, seen=None):
        if seen:
            self.seen[seen] = time.monotonic()
        now = time.monotonic()
        if now - self._last_image.get(id(signal), 0.0) < 1.0 / IMAGE_HZ:
            return
        self._last_image[id(signal)] = now
        img = qimage_from_ros(msg)
        if img is not None:
            signal.emit(img)

    def _on_topdown(self, msg):
        img = qimage_from_ros(msg)
        if img is not None:
            self.link.topdown_frame.emit(img)

    def _fleet_image(self, msg, key):
        now = time.monotonic()
        if now - self._last_image.get(key, 0.0) < 1.0 / IMAGE_HZ:
            return
        self._last_image[key] = now
        img = qimage_from_ros(msg)
        if img is not None:
            self.link.fleet_frame.emit(key, img)

    def _fleet_state(self, msg, key):
        """A fleet quad's own phase, straight off its namespaced topic."""
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(payload, dict):
            phase = str(payload.get('phase') or '--')
            self.link.robot_state.emit(key, phase, C.QUAD_PHASE_DETAIL.get(phase, ''))

    def _fleet_odom(self, msg, key):
        p = msg.pose.pose.position
        self.link.fleet_pose.emit(key, p.x, p.y, yaw_of(msg.pose.pose.orientation), p.z)

    def _on_agl(self, msg):
        self.agl = float(msg.data)
        self.agl_at = time.monotonic()
        self.link.quad_agl.emit(self.agl)

    def _on_landed(self, msg):
        if msg.data:
            self.link.quad_landed.emit(True)

    # -- holding robots still ----------------------------------------------
    def _standdown_tick(self):
        """Beat the stand-down heartbeats, and fly the quad down if asked.

        Their nodes hold a 0.5 s timer off each heartbeat, so this has to keep
        arriving - one message is not a stand-down, it is a blip. The descent
        rides on the same timer because the quad only accepts someone else's
        velocity while its own node is standing down, which is exactly the
        state this heartbeat creates.
        """
        for key in list(self._standdown):
            pub = self.fleet_teleop.get(key)
            if pub is not None:
                pub.publish(Bool(data=True))
        if self._descending and self.cmd_pubs:
            down = Twist()
            if self._should_descend():
                down.linear.z = -float(C.ABORT_DESCENT_MS)
            self.cmd_pubs[2].publish(down)       # parrot1/cmd_vel

    # Longest an abort descent may run before it stops regardless. The quad
    # cruises at about a metre: from four metres up, at ABORT_DESCENT_MS, the
    # ground arrives in about eight seconds, so twelve is generous and still
    # finite. The abort watcher's own 40 s timeout is far too long to be the
    # only thing between the quadcopter and the centre of the earth.
    DESCENT_LIMIT_S = 12.0
    AGL_STALE_S = 2.0

    def _should_descend(self):
        """Is the quadcopter still above the ground?

        This used to read `self.agl is None or self.agl > LANDED_AGL_M` - so
        the case where `parrot1/agl` had not arrived, or had stopped
        arriving, flew the quadcopter *downward*. On a run where that topic
        was quiet, Abort drove it through the world and kept going until the
        abort watcher gave up forty seconds later. Twenty-two metres of it.

        Unknown now means stop. Height comes from `agl` while it is fresh,
        from odometry z when it is not, and when neither can be had the
        descent runs only long enough to have landed from any sane cruise
        height.
        """
        now = time.monotonic()
        if now - self._descend_from > self.DESCENT_LIMIT_S:
            return False
        if self.agl is not None and now - self.agl_at < self.AGL_STALE_S:
            return self.agl > C.LANDED_AGL_M
        # Odometry z is not height above ground - on a heightmap world the
        # ground itself moves - but it is bounded below by the world origin,
        # and stopping a little high beats not stopping at all.
        if self.quad_z is not None:
            return self.quad_z > C.LANDED_AGL_M
        return True      # nothing to go on; the time limit above is the floor

    def set_fleet(self, quads, rovers):
        """How many of each robot this run has."""
        self.quads = max(1, min(int(quads), C.MAX_QUADS))
        self.rovers = max(1, min(int(rovers), C.MAX_ROVERS))

    def robot_keys(self):
        return ([f'quad{i}' for i in range(1, self.quads + 1)]
                + [f'rover{i}' for i in range(1, self.rovers + 1)])

    def set_standdown(self, robot, on):
        """Hold a robot's mission node down, or hand the robot back.

        `robot` is 'quad'/'rover' for the first of each - the single-robot
        case, and what every caller used before there were fleets - or an
        explicit 'quad2'/'rover3'. 'quads'/'rovers'/'all' cover the run.
        """
        keys = self._expand(robot)
        for key in keys:
            if on:
                self._standdown.add(key)
            else:
                self._standdown.discard(key)
                pub = self.fleet_teleop.get(key)
                if pub is not None:
                    pub.publish(Bool(data=False))   # released, not merely unheard
        if not on and any(k.startswith('quad') for k in keys):
            self._descending = False

    def _expand(self, robot):
        if robot == 'all':
            return self.robot_keys()
        if robot in ('quads', 'quad'):
            return [f'quad{i}' for i in range(1, self.quads + 1)] if robot == 'quads' else ['quad1']
        if robot in ('rovers', 'rover'):
            return [f'rover{i}' for i in range(1, self.rovers + 1)] if robot == 'rovers' else ['rover1']
        return [robot]

    def holding_robot(self, key):
        return key in self._standdown

    def set_descending(self, on):
        if on and not self._descending:
            self._descend_from = time.monotonic()
        self._descending = bool(on)

    def publish_mark(self, quad='quad1'):
        """Ask a quad to hand the mushroom in view to the rover."""
        pub = self.fleet_mark.get(quad)
        if pub is None:
            return False
        pub.publish(Bool(data=True))
        return True

    def _on_tf(self, msg):
        if self._tf_ok:
            return
        for tr in getattr(msg, 'transforms', []):
            if (getattr(tr.header, 'frame_id', '') == C.ROVER_MAP_FRAME
                    or getattr(tr, 'child_frame_id', '') == C.ROVER_MAP_FRAME):
                self._tf_ok = True
                return

    # -- readiness ----------------------------------------------------------
    def reset_readiness(self):
        """Forget everything the checklist has seen. Called on every Prepare.

        A new simulation has to prove itself from scratch, however much the
        last one managed. Without this the second run of a session starts
        with half the checklist already ticked by the first run's messages.
        """
        self.seen.clear()
        if self._tf_ok is not None:      # None means tf2_msgs is missing
            self._tf_ok = False

    def probe_readiness(self):
        """One look at everything the prepare stage is waiting for.

        Cameras and odometry are judged by whether a message has arrived
        *recently* - see `FRESH_S`, and the note on `self.seen`. Detectors are
        judged by whether their topic has a publisher, because a YOLO node
        with its weights loaded has created the publisher even before anything
        walks into frame. Nav2 is asked directly.
        """
        now = time.monotonic()
        probe = {key: (now - self.seen.get(key, -1e9)) < FRESH_S
                 for key in ('rover_odom', 'quad_odom', 'rover_cam', 'quad_cam')}
        probe['sim'] = self._has_publisher(C.CLOCK_TOPIC)
        probe['rover_yolo'] = self._has_publisher(C.HUSKY_DETECTIONS)
        probe['quad_yolo'] = self._has_publisher(C.PARROT_DETECTIONS)
        probe['nav2'] = self._nav_ready()
        # None means tf2_msgs is not installed, in which case claiming the
        # frame is missing would block a launch that would work.
        probe['map_frame'] = True if self._tf_ok is None else bool(self._tf_ok)
        return probe

    def _nav_ready(self):
        if self.nav_client is None:
            return False
        try:
            return bool(self.nav_client.server_is_ready())
        except AttributeError:
            return bool(self.nav_client.wait_for_server(timeout_sec=0.0))

    def _has_publisher(self, topic):
        try:
            return self.count_publishers(topic) > 0
        except Exception:
            return False

    # -- outbound -----------------------------------------------------------
    def publish_pause(self, paused):
        if self.pause_pub:
            self.pause_pub.publish(Bool(data=bool(paused)))

    def publish_command(self, payload):
        if self.command_pub:
            self.command_pub.publish(String(data=json.dumps(payload)))

    def publish_waypoint(self, x, y):
        if not self.waypoint_pub:
            return
        pt = PointStamped()
        pt.header.frame_id = 'map'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(x)
        pt.point.y = float(y)
        self.waypoint_pub.publish(pt)

    def publish_drive(self, vx, wz, active=True):
        """Manual driving. Goes through cmd_vel_raw so collision_guard still
        gets to veto it - the operator should not be able to drive the rover
        into a trunk just because they are holding W. The teleop_active
        heartbeat makes the rover's mission node stand down meanwhile, the
        same as for the PS4 controller; active=False hands it back."""
        if not self.cmd_pubs or self._holding:
            return
        # An explicit hold from the window outranks the panel being disarmed:
        # during Prepare, a pause or an abort the rover is held for a reason
        # the panel knows nothing about.
        if active or 'rover1' not in self._standdown:
            self.teleop_pub.publish(Bool(data=bool(active)))
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pubs[1].publish(t)   # husky1/cmd_vel_raw

    def publish_quad_drive(self, vx, vy, vz, wz, active=True):
        """Hand-flying the quadcopter.

        There is no guard in front of the quad - nothing filters its velocity
        the way collision_guard filters the rover's - so this goes straight to
        `parrot1/cmd_vel`. Its mission node has to be suspended first or the
        two fight over the same topic, which the GUI does before arming.
        """
        if not self.cmd_pubs or self._holding:
            return
        # Same arrangement as the rover's: the heartbeat makes zcam_quad_node
        # stand down while the operator flies, and it keeps reporting
        # sightings to the rover the whole time.
        if self.quad_teleop_pub is not None and (active or 'quad1' not in self._standdown):
            self.quad_teleop_pub.publish(Bool(data=bool(active)))
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(wz)
        self.cmd_pubs[2].publish(t)   # parrot1/cmd_vel


class RosLink(QObject):
    """Qt-facing handle. Construct on the GUI thread, then call start()."""

    status_updated = Signal(dict)
    event_received = Signal(dict)
    pause_changed = Signal(bool)
    rover_pose = Signal(float, float, float)
    quad_pose = Signal(float, float, float, float)
    rover_frame = Signal(object)
    quad_frame = Signal(object)
    rover_yolo_frame = Signal(object)
    quad_yolo_frame = Signal(object)
    rover_detections = Signal(list)
    quad_detections = Signal(list)
    teleop_status = Signal(str)
    quad_agl = Signal(float)
    quad_landed = Signal(bool)
    fleet_frame = Signal(str, object)
    fleet_pose = Signal(str, float, float, float, float)
    robot_state = Signal(str, str, str)
    nav_goal_finished = Signal(bool, str)   # ok, and why not
    topdown_frame = Signal(object)

    def __init__(self, role='gui'):
        super().__init__()
        self.role = role
        self._node = _LinkNode(self, role)
        self._executor = None
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def shutdown(self):
        try:
            self._node.set_holding(False)
            self._node.destroy_node()
        except Exception:
            pass

    # thin pass-throughs so windows never reach into the node directly
    def set_holding(self, holding):
        self._node.set_holding(holding)

    def publish_pause(self, paused):
        self._node.publish_pause(paused)

    def publish_command(self, payload):
        self._node.publish_command(payload)

    def publish_waypoint(self, x, y):
        self._node.publish_waypoint(x, y)

    def publish_drive(self, vx, wz, active=True):
        self._node.publish_drive(vx, wz, active)

    def publish_quad_drive(self, vx, vy, vz, wz, active=True):
        self._node.publish_quad_drive(vx, vy, vz, wz, active)

    def set_standdown(self, robot, on):
        self._node.set_standdown(robot, on)

    def set_descending(self, on):
        self._node.set_descending(on)

    def publish_mark(self, quad='quad1'):
        return self._node.publish_mark(quad)

    def set_fleet(self, quads, rovers):
        self._node.set_fleet(quads, rovers)

    def robot_keys(self):
        return self._node.robot_keys()

    def quad_height(self):
        return self._node.agl

    def cancel_all_nav(self):
        return self._node.cancel_all_nav()

    def send_nav_goal(self, x, y, yaw=0.0):
        return self._node.send_nav_goal(x, y, yaw)

    def probe_readiness(self):
        return self._node.probe_readiness()

    def reset_readiness(self):
        """Make the next Prepare start from nothing. See the node's own."""
        self._node.reset_readiness()

    def publish_stop(self):
        """One zero on cmd_vel_raw, so the guard stops replaying a stale command."""
        self._node.publish_drive(0.0, 0.0)

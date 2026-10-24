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


class _LinkNode(Node):
    """The actual ROS node. Everything here runs on the spin thread."""

    def __init__(self, link, role):
        super().__init__(f'ammo_{role}_link')
        self.link = link
        self.role = role
        self._last_image = {}
        # What has been seen at least once, for the readiness checklist. A
        # frame or a message arriving is a much better answer to "is the
        # camera working" than a publisher existing on the topic.
        self.seen = set()
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
        except ImportError:
            self.get_logger().warn('vision_msgs not available - no boxes on the camera feeds')

        # One frame from the camera mission_launcher spawns over the world
        # while a run is being prepared. Subscribed always: the topic simply
        # has no publisher until that camera exists.
        self.create_subscription(Image, C.TOPDOWN_TOPIC, self._on_topdown, LIVE_QOS)

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
            self.teleop_pub = self.create_publisher(Bool, C.HUSKY_TELEOP, 10)
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
            self.link.nav_goal_finished.emit(False)
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        ok = False
        try:
            result = future.result()
            ok = getattr(result, 'status', 0) == 4  # STATUS_SUCCEEDED
        except Exception:
            ok = False
        self._nav_handle = None
        self.link.nav_goal_finished.emit(ok)

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
        for pub in self.cmd_pubs:
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
        self.seen.add('rover_odom')
        p = msg.pose.pose.position
        self.link.rover_pose.emit(p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_quad_odom(self, msg):
        self.seen.add('quad_odom')
        p = msg.pose.pose.position
        self.link.quad_pose.emit(p.x, p.y, yaw_of(msg.pose.pose.orientation), p.z)

    def _detections(self, msg, signal):
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
        signal.emit(boxes)

    def _image(self, msg, signal, seen=None):
        if seen:
            self.seen.add(seen)
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

    def _on_tf(self, msg):
        if self._tf_ok:
            return
        for tr in getattr(msg, 'transforms', []):
            if (getattr(tr.header, 'frame_id', '') == C.ROVER_MAP_FRAME
                    or getattr(tr, 'child_frame_id', '') == C.ROVER_MAP_FRAME):
                self._tf_ok = True
                return

    # -- readiness ----------------------------------------------------------
    def probe_readiness(self):
        """One look at everything the prepare stage is waiting for.

        Cameras and odometry are judged by whether a message has actually
        arrived; detectors by whether their topic has a publisher, because a
        YOLO node with its weights loaded has created the publisher even
        before anything walks into frame. Nav2 is asked directly.
        """
        probe = {key: (key in self.seen) for key in ('rover_odom', 'quad_odom',
                                                     'rover_cam', 'quad_cam')}
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
        self.teleop_pub.publish(Bool(data=bool(active)))
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pubs[1].publish(t)   # husky1/cmd_vel_raw

    def publish_quad_drive(self, vx, vy, vz, wz):
        """Hand-flying the quadcopter.

        There is no guard in front of the quad - nothing filters its velocity
        the way collision_guard filters the rover's - so this goes straight to
        `parrot1/cmd_vel`. Its mission node has to be suspended first or the
        two fight over the same topic, which the GUI does before arming.
        """
        if not self.cmd_pubs or self._holding:
            return
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
    nav_goal_finished = Signal(bool)
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

    def publish_quad_drive(self, vx, vy, vz, wz):
        self._node.publish_quad_drive(vx, vy, vz, wz)

    def cancel_all_nav(self):
        return self._node.cancel_all_nav()

    def send_nav_goal(self, x, y, yaw=0.0):
        return self._node.send_nav_goal(x, y, yaw)

    def probe_readiness(self):
        return self._node.probe_readiness()

    def publish_stop(self):
        """One zero on cmd_vel_raw, so the guard stops replaying a stale command."""
        self._node.publish_drive(0.0, 0.0)

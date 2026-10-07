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

        self.create_subscription(String, C.STATUS_TOPIC, self._on_status, LIVE_QOS)
        self.create_subscription(String, C.EVENTS_TOPIC, self._on_event, EVENT_QOS)
        self.create_subscription(Bool, C.PAUSE_TOPIC, self._on_pause, 10)
        self.create_subscription(Odometry, C.HUSKY_ODOM, self._on_rover_odom, LIVE_QOS)
        self.create_subscription(Odometry, C.PARROT_ODOM, self._on_quad_odom, LIVE_QOS)

        # Raw feeds drive the live panels; the annotated ones are only
        # subscribed so a frame is available the moment a sighting lands.
        # yolo_node publishes its annotated image lazily - only while
        # something is subscribed - so these subscriptions are what turn it on.
        self.create_subscription(Image, C.HUSKY_CAM,
                                 lambda m: self._image(m, self.link.rover_frame), LIVE_QOS)
        self.create_subscription(Image, C.PARROT_CAM,
                                 lambda m: self._image(m, self.link.quad_frame), LIVE_QOS)
        self.create_subscription(Image, C.HUSKY_CAM_YOLO,
                                 lambda m: self._image(m, self.link.rover_yolo_frame), LIVE_QOS)
        self.create_subscription(Image, C.PARROT_CAM_YOLO,
                                 lambda m: self._image(m, self.link.quad_yolo_frame), LIVE_QOS)

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

    def send_nav_goal(self, x, y):
        if self.nav_client is None or not self.nav_client.wait_for_server(timeout_sec=2.0):
            return False
        try:
            from geometry_msgs.msg import PoseStamped
            from nav2_msgs.action import NavigateToPose
            goal = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            goal.pose = pose
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
        p = msg.pose.pose.position
        self.link.rover_pose.emit(p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_quad_odom(self, msg):
        p = msg.pose.pose.position
        self.link.quad_pose.emit(p.x, p.y, yaw_of(msg.pose.pose.orientation), p.z)

    def _image(self, msg, signal):
        now = time.monotonic()
        if now - self._last_image.get(id(signal), 0.0) < 1.0 / IMAGE_HZ:
            return
        self._last_image[id(signal)] = now
        img = qimage_from_ros(msg)
        if img is not None:
            signal.emit(img)

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
    nav_goal_finished = Signal(bool)

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

    def cancel_all_nav(self):
        return self._node.cancel_all_nav()

    def send_nav_goal(self, x, y):
        return self._node.send_nav_goal(x, y)

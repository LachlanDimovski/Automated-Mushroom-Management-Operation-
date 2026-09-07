"""ROS2 <-> Qt bridge for the operator GUI.

Same "intake on its own thread, GUI owns the main thread" split
ammo_dashboard uses (see its main(), which spins rclpy on a daemon thread
so a slow render call can never back up subscriptions) - except here the
handoff to the GUI thread is explicit, via Qt signals, rather than shared
mutable attributes read straight off the node. A Qt signal emitted from a
non-GUI thread is queued onto the receiving QObject's own thread
automatically (Qt.AutoConnection, the default) as long as that QObject was
created on the GUI thread - which RosBridge is, since main.py constructs it
before starting the spin thread. That's what makes it safe for _BridgeNode's
callbacks (running on the spin thread) to just call bridge.<signal>.emit(...)
directly instead of needing a lock/queue of our own.

Topics this subscribes to, and why:
  - <map_topic> (nav_msgs/OccupancyGrid, default husky1/map): the ground
    SLAM grid - see map_view.py and the map-source discussion in the repo
    docs for why husky1 rather than parrot1.
  - husky1/odometry, parrot1/odometry (nav_msgs/Odometry): same raw
    world-absolute Gazebo bridge topics ammo_dashboard and
    mushroom_mission_node already read poses from directly (no EKF, no TF
    lookup) - husky_spawn/parrot_spawn below are the same spawn-offset
    convention those use.
  - <mushroom_found_topic> (geometry_msgs/PointStamped, default
    mushroom_found): NEW topic, not published by anything yet as of this
    commit - see mushroom_mission_node.py's diff, which adds one publish
    call each time it dispatches the husky to a newly-confirmed mushroom.
    That change needs Lachlan's sign-off before it's pushed; until then
    this subscription just never receives anything, which is harmless -
    mushroom count/markers stay at zero rather than erroring.

Publishes to:
  - <teleop_topic> (geometry_msgs/Twist, default /parrot1/cmd_vel): manual
    keyboard teleop, from teleop_widget.py. This is the exact topic
    ps4_drone_test.py also publishes to directly, with no arbitration node
    between them the way the husky has cbf_guard_node - so this and
    ps4_drone_test.py are an either/or, never both running at once (see
    teleop_widget.py's docstring).

Mission-progress estimate: on every map message, the fraction of cells
that are no longer "unknown" (-1) is used as a stand-in for "how much of
the working area has been explored", scaled against the working-area
figure entered on the home page. It's a coarse proxy - it counts explored
map area, not confirmed area within whatever the field boundary actually
is (there's no real geofencing yet) - but it's real SLAM data, not a
placeholder, and needs no changes to anyone else's node.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from PySide6.QtCore import QObject, Signal

# Newest sample only - a slow repaint must never work through a backlog of
# stale odometry/detections. Same LIVE_QOS shape used throughout this repo
# (mushroom_mission_node, ammo_dashboard).
LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# nav_msgs/OccupancyGrid map publishers (slam_toolbox, nav2 map_server)
# conventionally publish with this QoS - reliable + transient-local, so a
# late-joining subscriber still gets the latest map instead of waiting for
# the next update. Best-effort depth-1 (like LIVE_QOS) would very likely
# never match slam_toolbox's publisher at all. If this project's
# slam_toolbox launch overrides that QoS, this is the one line to change.
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# A dispatched-mushroom event closer than this to an already-counted one is
# treated as a re-trigger on the same mushroom (auto_trigger can re-fire on
# a later fly-over, or a later R1+Triangle can override a goal mid-drive -
# see mushroom_mission_node.py) rather than a new find. Sized a bit smaller
# than mushroom_mission_node's own APPROACH_RADIUS_M (4.0) since that's the
# radius it drives *to*, not the spacing between distinct mushrooms.
DEDUP_RADIUS_M = 2.5


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class _BridgeNode(Node):
    """The actual rclpy node. Kept separate from RosBridge (a QObject) since
    QObject and rclpy.Node both come with their own base-class machinery -
    simplest to keep them as two objects joined by plain callbacks rather
    than fight a multiple-inheritance MRO.
    """

    def __init__(self, bridge: 'RosBridge'):
        super().__init__('ammo_gui_node')

        self.declare_parameter('map_topic', 'husky1/map')
        self.declare_parameter('mushroom_found_topic', 'mushroom_found')
        self.declare_parameter('teleop_topic', '/parrot1/cmd_vel')
        self.declare_parameter('husky_spawn', [0.0, 0.0])
        self.declare_parameter('parrot_spawn', [2.0, 0.0])
        self.declare_parameter('parrot_ground_z', 0.8)

        self.husky_spawn = self.get_parameter('husky_spawn').value
        self.parrot_spawn = self.get_parameter('parrot_spawn').value
        self.parrot_ground_z = self.get_parameter('parrot_ground_z').value

        map_topic = self.get_parameter('map_topic').value
        mushroom_topic = self.get_parameter('mushroom_found_topic').value
        teleop_topic = self.get_parameter('teleop_topic').value

        self._bridge = bridge

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, MAP_QOS)
        self.create_subscription(
            Odometry, 'husky1/odometry', self._on_husky_odom, LIVE_QOS)
        self.create_subscription(
            Odometry, 'parrot1/odometry', self._on_parrot_odom, LIVE_QOS)
        self.create_subscription(
            PointStamped, mushroom_topic, self._on_mushroom_found, 10)

        self.teleop_pub = self.create_publisher(Twist, teleop_topic, 10)

        self._mushroom_points = []  # de-duplicated [(world_x, world_y), ...]

        self.get_logger().info(
            f'ammo_gui_node bridge up - map:{map_topic}  '
            f'mushroom:{mushroom_topic}  teleop:{teleop_topic}')

    def _on_map(self, msg: OccupancyGrid):
        cells = msg.data
        total = len(cells)
        if total == 0:
            return
        known = sum(1 for c in cells if c != -1)
        cell_area_m2 = msg.info.resolution * msg.info.resolution
        known_ha = (known * cell_area_m2) / 10000.0
        fraction = (known / total) if self._bridge.target_area_ha <= 0 else min(
            1.0, known_ha / self._bridge.target_area_ha)

        self._bridge.map_received.emit(msg)
        self._bridge.progress_updated.emit(known_ha, self._bridge.target_area_ha, fraction)

    def _on_husky_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        x = self.husky_spawn[0] + p.x
        y = self.husky_spawn[1] + p.y
        yaw = _yaw_of(msg.pose.pose.orientation)
        self._bridge.husky_pose.emit(x, y, yaw)

    def _on_parrot_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        x = self.parrot_spawn[0] + p.x
        y = self.parrot_spawn[1] + p.y
        yaw = _yaw_of(msg.pose.pose.orientation)
        alt = self.parrot_ground_z + p.z
        self._bridge.parrot_pose.emit(x, y, yaw, alt)

    def _on_mushroom_found(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        for ex, ey in self._mushroom_points:
            if math.hypot(x - ex, y - ey) < DEDUP_RADIUS_M:
                return  # same mushroom as one already counted - ignore
        self._mushroom_points.append((x, y))
        self._bridge.mushroom_found.emit(x, y, len(self._mushroom_points))


class RosBridge(QObject):
    """Owns the rclpy node + its spin thread, and re-emits everything it
    receives as Qt signals so the rest of the GUI never touches rclpy
    directly. Construct on the GUI thread, call start() once, shutdown()
    once at exit.
    """

    map_received = Signal(object)  # nav_msgs/OccupancyGrid
    husky_pose = Signal(float, float, float)  # world x, y, yaw (rad)
    parrot_pose = Signal(float, float, float, float)  # world x, y, yaw, altitude
    mushroom_found = Signal(float, float, int)  # world x, y, running total
    progress_updated = Signal(float, float, float)  # known_ha, target_ha, fraction

    def __init__(self):
        super().__init__()
        self._node = None
        self._spin_thread = None
        self.target_area_ha = 0.0

    def start(self):
        if self._node is not None:
            return
        self._node = _BridgeNode(self)
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    def set_target_area_ha(self, area_ha: float):
        self.target_area_ha = max(0.0, area_ha)

    def publish_teleop(self, twist: Twist):
        if self._node is not None:
            self._node.teleop_pub.publish(twist)

    def shutdown(self):
        # Same ordering ammo_dashboard's main() uses: signal shutdown first
        # so rclpy.spin() returns on its own, then join the thread, then
        # tear down the node - destroying the node while spin() is still
        # using it is the thing to avoid.
        if rclpy.ok():
            rclpy.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()

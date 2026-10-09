"""One list of mushrooms for the whole fleet, and which rover each one belongs to.

Every quad reports to fleet/reports. A report within MERGE_M of a known mushroom refines it and goes to that
mushroom's rover only; a new one goes to the rover with the least work (distance from it or its nearest queued
mushroom, plus QUEUE_COST_M per open mushroom). So two rovers never drive to the same mushroom.
With queue_cap:=N a rover holds at most N open mushrooms; the rest wait here as 'pending' and go to whichever rover
frees up first, nearest first (static queues left husky1 idle from 398 s while r2 had 5 queued - fleet4). Rover results come back on
<rover>/cage/done and <rover>/cage/missed and go out to the quads on fleet/done and fleet/missed.

Also: the quads' ground maps merged for the rovers (fleet/ground_obstacles), fleet/landed once every quad has landed,
and the whole picture as JSON on fleet/status (mushrooms, owners, states, robot positions, quad claims).

End of mission: the rovers' <rover>/collected counts add up to `total` -> ammo/mission_complete (every rover drives
home, every quad lands) -> once all are home and landed, or END_WAIT_S later, the sim is ended.
"""
import json
import math
import os
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String

MERGE_M = 1.5        # the quads' DISPATCHED_M
DONE_M = 2.0         # a rover's done/missed point this close to one of its mushrooms closes it
QUEUE_COST_M = 8.0   # an open mushroom in a rover's queue counts as this much extra driving
CROSS_COST_M = 15.0  # a river crossing (lane drive, banks) counts as this much extra driving
STUCK_S = 75.0       # a rover holding a mushroom that has moved < STUCK_M in this long gives it to an idle rover
STUCK_M = 1.0
END_WAIT_S = 150.0   # end the sim this long after the mission completes even if someone never reports home/landed
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LIVE = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class FleetCoordinator(Node):

    def __init__(self):
        super().__init__('fleet_coordinator')
        self.declare_parameter('rovers', ['husky1'])
        self.declare_parameter('quads', ['parrot1'])
        self.declare_parameter('total', 0)  # mushrooms in the world; 0 = never declare the mission complete
        self.declare_parameter('end_sim', True)
        self.declare_parameter('queue_cap', 0)  # 0 = every mushroom goes straight to a rover's queue
        self.cap = int(self.get_parameter('queue_cap').value)
        self.declare_parameter('river_meta', '')  # world .meta.json: distances go via a crossing across the river
        self.river, self.crossings = [], []
        meta = self.get_parameter('river_meta').value
        if meta and os.path.exists(meta):
            d = json.load(open(meta))
            self.river = sorted((p[0], p[1]) for p in d.get('river', {}).get('line', []))
            self.crossings = [tuple(c[:2]) for c in d.get('crossings', d.get('fords', []))]
        self.total = int(self.get_parameter('total').value)
        self.collected, self.home, self.complete_at = {}, set(), None
        self.rovers = list(self.get_parameter('rovers').value)
        self.quads = list(self.get_parameter('quads').value)
        self.mushrooms = []  # dicts: id, x, y, rover, state (pending | assigned | done | missed)
        self.pos = {}
        self.claims = {}
        self.maps = {}
        self.landed = set()
        self.moved = {}  # rover -> (x, y, sim time) of its last STUCK_M move
        self.cancel_pubs = {r: self.create_publisher(PointStamped, f'{r}/cage/cancel', 10) for r in self.rovers}
        self.wp_pubs = {r: self.create_publisher(PointStamped, f'{r}/cage/waypoints', 10) for r in self.rovers}
        self.done_pub = self.create_publisher(PointStamped, 'fleet/done', 10)
        self.missed_pub = self.create_publisher(PointStamped, 'fleet/missed', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'fleet/ground_obstacles', LATCHED)
        self.landed_pub = self.create_publisher(Bool, 'fleet/landed', LATCHED)
        self.status_pub = self.create_publisher(String, 'fleet/status', 10)
        self.complete_pub = self.create_publisher(Bool, 'ammo/mission_complete', LATCHED)
        self.create_subscription(PointStamped, 'fleet/reports', self._report, 10)
        self.create_subscription(PointStamped, 'fleet/claims',
                                 lambda m: self.claims.__setitem__(m.header.frame_id, (m.point.x, m.point.y)), 10)
        for r in self.rovers:
            self.create_subscription(PointStamped, f'{r}/cage/done', lambda m, r=r: self._result(r, m, 'done'), 10)
            self.create_subscription(PointStamped, f'{r}/cage/missed', lambda m, r=r: self._result(r, m, 'missed'), 10)
            self.create_subscription(Int32, f'{r}/collected', lambda m, r=r: self._collected(r, m), LATCHED)
            self.create_subscription(Bool, f'{r}/home', lambda m, r=r: m.data and self.home.add(r), LATCHED)
        for name in self.rovers + self.quads:
            self.create_subscription(Odometry, f'{name}/odometry', lambda m, n=name: self.pos.__setitem__(
                n, (m.pose.pose.position.x, m.pose.pose.position.y)), LIVE)
        for q in self.quads:
            self.create_subscription(OccupancyGrid, f'{q}/ground_obstacles', lambda m, q=q: self.maps.__setitem__(q, m),
                                     LATCHED)
            self.create_subscription(Bool, f'{q}/landed', lambda m, q=q: self._landed(q, m), LATCHED)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Fleet: rovers {self.rovers}, quads {self.quads}.')

    def _open(self, rover):
        return [m for m in self.mushrooms if m['rover'] == rover and m['state'] == 'assigned']

    def _report(self, msg):
        x, y = msg.point.x, msg.point.y
        near = [m for m in self.mushrooms if m['state'] != 'done' and math.hypot(m['x'] - x, m['y'] - y) < MERGE_M]
        if near:
            m = min(near, key=lambda m: math.hypot(m['x'] - x, m['y'] - y))
            m['x'], m['y'] = x, y
            if m['state'] == 'missed':  # found again after an empty grab: back to the same rover
                m['state'] = 'assigned'
        else:
            free = [r for r in self.rovers if not self.cap or len(self._open(r)) < self.cap]
            m = {'id': len(self.mushrooms) + 1, 'x': x, 'y': y, 'rover': self._pick_rover(x, y, free) if free else None,
                 'state': 'assigned' if free else 'pending'}
            self.mushrooms.append(m)
            self.get_logger().info(f"Mushroom M{m['id']} at ({x:.1f},{y:.1f}) -> " + (
                f"{m['rover']} ({len(self._open(m['rover']))} open for it)." if free else 'pending, every rover busy.'))
        if m['state'] == 'assigned':
            self._send(m)

    def _send(self, m):
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.point.x, out.point.y = m['x'], m['y']
        self.wp_pubs[m['rover']].publish(out)

    def _dispatch(self):
        """Pending mushrooms to rovers under the cap, each to the nearest such rover's next stop."""
        for r in self.rovers:
            while len(self._open(r)) < self.cap:
                pend = [m for m in self.mushrooms if m['state'] == 'pending']
                if not pend:
                    return
                q = self._open(r)
                px, py = (q[-1]['x'], q[-1]['y']) if q else self.pos.get(r, (0.0, 0.0))
                pend = [m for m in pend if m.get('from') != r] or pend  # not straight back to a rover stuck on it
                m = min(pend, key=lambda m: self._dist((m['x'], m['y']), (px, py)))
                m['rover'], m['state'] = r, 'assigned'
                self.get_logger().info(f"M{m['id']} -> {r} (was pending).")
                self._send(m)

    def _unstick(self):
        """A rover that has not moved for STUCK_S while it holds a mushroom (r2 stalled in Nav2 for the last 5 min
        of bench q6r2_lite_v2) loses that mushroom to a rover with nothing open; the stuck one is told to drop it."""
        now = self._now()
        for r in self.rovers:
            p = self.pos.get(r)
            if p is None:
                continue
            last = self.moved.get(r)
            if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) > STUCK_M or not self._open(r):
                self.moved[r] = (p[0], p[1], now)
                continue
            if now - last[2] < STUCK_S:
                continue
            idle = [o for o in self.rovers if o != r and not self._open(o)]
            if not idle:
                continue
            for m in self._open(r):
                o = min(idle, key=lambda o: self._dist((m['x'], m['y']), self.pos.get(o, (0.0, 0.0))))
                cancel = PointStamped()
                cancel.header.stamp = self.get_clock().now().to_msg()
                cancel.header.frame_id = 'map'
                cancel.point.x, cancel.point.y = m['x'], m['y']
                self.cancel_pubs[r].publish(cancel)
                m['rover'], m['from'] = o, r
                self.get_logger().warn(f"{r} has not moved in {now - last[2]:.0f} s - M{m['id']} handed to {o}.")
                self._send(m)
                idle.remove(o)
                if not idle:
                    break
            self.moved[r] = (p[0], p[1], now)

    def _bank(self, x, y):
        """+1 north of the river, -1 south (the river runs along x), 0 with no river."""
        if not self.river:
            return 0
        i = min(range(len(self.river)), key=lambda i: abs(self.river[i][0] - x))
        return 1 if y > self.river[i][1] else -1

    def _dist(self, a, b):
        """Driving distance estimate: straight, or via the best crossing when a and b are on opposite banks
        (straight-line costs sent r2 across the river after one mushroom and it never came back - bench q4r2)."""
        if not self.crossings or self._bank(*a) == self._bank(*b):
            return math.hypot(a[0] - b[0], a[1] - b[1])
        return CROSS_COST_M + min(math.hypot(a[0] - c[0], a[1] - c[1]) + math.hypot(b[0] - c[0], b[1] - c[1])
                                  for c in self.crossings)

    def _pick_rover(self, x, y, rovers):
        """Nearest rover, counting what it already has to do: the rover takes its queue nearest first, so a new
        mushroom costs the distance from the rover or from the nearest mushroom already queued for it, whichever is
        less (from the last-queued one sent rovers across the map - fleet3)."""
        def cost(r):
            q = self._open(r)
            here = [self.pos.get(r, (0.0, 0.0))] + [(m['x'], m['y']) for m in q]
            return min(self._dist((x, y), p) for p in here) + QUEUE_COST_M * len(q)
        return min(rovers, key=cost)

    def _result(self, rover, msg, state):
        x, y = msg.point.x, msg.point.y
        mine = [m for m in self._open(rover) if math.hypot(m['x'] - x, m['y'] - y) < DONE_M]
        if mine:
            m = min(mine, key=lambda m: math.hypot(m['x'] - x, m['y'] - y))
            m['state'] = state
            self.get_logger().info(f"M{m['id']} {state} by {rover}.")
            if self.cap:
                self._dispatch()
        (self.done_pub if state == 'done' else self.missed_pub).publish(msg)

    def _collected(self, rover, msg):
        self.collected[rover] = msg.data
        n = sum(self.collected.values())
        if self.total and n >= self.total and self.complete_at is None:
            self.complete_at = self._now()
            self.complete_pub.publish(Bool(data=True))
            self.get_logger().info(f'All {self.total} mushrooms collected ({self.collected}) - everyone home.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _end_check(self):
        if self.complete_at is None or not self.get_parameter('end_sim').value:
            return
        all_in = self.home >= set(self.rovers) and self.landed >= set(self.quads)
        if all_in or self._now() - self.complete_at > END_WAIT_S:
            n = sum(self.collected.values())
            self.get_logger().info(f'Mission complete: {n}/{self.total} collected, '
                                   + ('every robot home' if all_in else 'not every robot reported home')
                                   + ' - ending the simulation.')
            self.complete_at = float('inf')
            os.killpg(os.getpgid(0), signal.SIGINT)  # as Ctrl-C on the launch: every node and Gazebo

    def _landed(self, quad, msg):
        if msg.data and quad not in self.landed:
            self.landed.add(quad)
            if self.landed >= set(self.quads):
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info('Every quad has landed.')

    def _merged_map(self):
        msgs = [self.maps[q] for q in self.quads if q in self.maps]
        if not msgs:
            return None
        base = msgs[0]
        a = np.full(len(base.data), -1, np.int8)
        for m in msgs:
            if len(m.data) != len(base.data):
                continue
            d = np.array(m.data, np.int8)
            a = np.where(d == 100, 100, np.where((d == 0) & (a < 0), 0, a)).astype(np.int8)
        out = OccupancyGrid()
        out.header, out.info = base.header, base.info
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = a.tolist()
        return out

    def _tick(self):
        self._end_check()
        if self.cap and self.complete_at is None:
            self._dispatch()
            self._unstick()
        merged = self._merged_map()
        if merged is not None:
            self.map_pub.publish(merged)
        status = {'mushrooms': self.mushrooms, 'positions': self.pos, 'claims': self.claims,
                  'open': {r: [m['id'] for m in self._open(r)] for r in self.rovers},
                  'pending': [m['id'] for m in self.mushrooms if m['state'] == 'pending'], 'collected': self.collected,
                  'total': self.total, 'home': sorted(self.home), 'landed': sorted(self.landed)}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

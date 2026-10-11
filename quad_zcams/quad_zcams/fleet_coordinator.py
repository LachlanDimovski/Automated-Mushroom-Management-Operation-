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
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String

MERGE_M = 1.5        # the quads' DISPATCHED_M
DONE_M = 2.0         # a rover's done/missed point this close to one of its mushrooms closes it
QUEUE_COST_M = 8.0   # an open mushroom in a rover's queue counts as this much extra driving
CROSS_COST_M = 30.0  # a river crossing (lane drive, banks) counts as this much extra driving: 15 understated it,
                     # crossings took r2 3-4 min and it kept being sent to the other bank (bench v4/v5)
STUCK_M = 1.0
CROWD_R_M = 5.0      # two rovers working within this of each other get in each other's way...
CROWD_COST_M = 10.0  # ...so a mushroom this close to another rover's spot counts as this much extra driving
PICK_COST_M = 8.0    # align, lower, collect, raise: ~9 sim s, about 8 m of the rovers' real progress
NO_PROGRESS_S = 150.0  # a rover whose route distance to its mushroom has not shrunk STUCK_M in this long gives it up
GAVE_UP_HOLD_S = 120.0  # a mushroom every rover has given up on waits this long before it is handed out again
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
            self.create_subscription(PointStamped, f'{r}/cage/gave_up', lambda m, r=r: self._gave_up(r, m), 10)
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
        elif self.cap:  # a new sighting goes through the joint plan (_dispatch) like every other pending one
            m = {'id': len(self.mushrooms) + 1, 'x': x, 'y': y, 'rover': None, 'state': 'pending'}
            self.mushrooms.append(m)
            self.get_logger().info(f"Mushroom M{m['id']} at ({x:.1f},{y:.1f}).")
            self._dispatch()
            return
        else:
            m = {'id': len(self.mushrooms) + 1, 'x': x, 'y': y, 'rover': self._pick_rover(x, y, self.rovers),
                 'state': 'assigned'}
            self.mushrooms.append(m)
            self.get_logger().info(f"Mushroom M{m['id']} at ({x:.1f},{y:.1f}) -> {m['rover']} "
                                   f"({len(self._open(m['rover']))} open for it).")
        if m['state'] == 'assigned':
            self._send(m)

    def _send(self, m):
        if m.get('since_rover') != m['rover']:  # held since it was given to this rover
            m['since'], m['since_rover'] = self._now(), m['rover']
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.point.x, out.point.y = m['x'], m['y']
        self.wp_pubs[m['rover']].publish(out)

    def _dispatch(self):
        """Pending mushrooms to rovers under the cap. A free rover takes the first mushroom of its share of a joint plan
        (_plan), so it looks ahead at what the other rovers will reach first once they finish what they hold, rather
        than grabbing the one nearest to itself (nearest-to-me: husky1 9, r2 4 - bench q6r2_lite_v3)."""
        for r in sorted(self.rovers, key=lambda r: len(self._open(r))):
            while len(self._open(r)) < self.cap:
                pend = [m for m in self.mushrooms if m['state'] == 'pending' and m.get('hold_until', 0.0) <= self._now()]
                if not pend:
                    break
                plan = self._plan(pend)
                if not plan[r]:  # every pending one is better left to another rover: wait, re-planned every tick
                    break
                m = plan[r][0]
                m['rover'], m['state'] = r, 'assigned'
                ahead = '; '.join(f"{o} " + ','.join(f"M{n['id']}" for n in plan[o]) for o in self.rovers if plan[o])
                self.get_logger().info(f"M{m['id']} -> {r} (was pending; plan {ahead or 'none'}).")
                self._send(m)

    def _plan(self, pend):
        """Look-ahead over every rover: each rover starts where and when it will be free (after its open mushrooms);
        repeatedly the (rover, mushroom) pair with the earliest arrival is fixed and that rover moves on from there.
        Returns rover -> its planned mushrooms in order."""
        at = {o: self._free_at(o) for o in self.rovers}
        t = {o: self._busy_m(o) for o in self.rovers}
        plan = {o: [] for o in self.rovers}
        left = list(pend)
        while left:
            def arrive(om):
                o, m = om
                crowd = any(math.hypot(m['x'] - at[p][0], m['y'] - at[p][1]) < CROWD_R_M for p in self.rovers if p != o)
                return t[o] + self._dist(at[o], (m['x'], m['y'])) + (CROWD_COST_M if crowd else 0.0)
            # never back to the rover that dropped it (an 'or pend' fallback here undid every give-up handover)
            pairs = [(o, m) for o in self.rovers for m in left if m.get('from') != o or len(self.rovers) == 1]
            if not pairs:
                break
            o, m = min(pairs, key=arrive)
            t[o] += self._dist(at[o], (m['x'], m['y'])) + PICK_COST_M
            at[o] = (m['x'], m['y'])
            plan[o].append(m)
            left.remove(m)
        return plan

    def _unstick(self):
        """A rover that has got no closer to its mushroom in NO_PROGRESS_S drops it: to an idle rover if there is one,
        otherwise back into the joint plan (r2 stalled in Nav2 for the last 5 min of bench q6r2_lite_v2)."""
        now = self._now()
        for r in self.rovers:
            p = self.pos.get(r)
            if p is None:
                continue
            q = self._open(r)
            if not q:
                self.moved.pop(r, None)
                continue
            # progress = the crossing-aware distance to its mushroom shrinking: a fixed hold time (120 s) cancelled
            # long crossings halfway (v19/v20); standing still and circling (v11) both stop it shrinking
            d = self._dist(p, (q[0]['x'], q[0]['y']))
            best = self.moved.get(r)
            if best is None or best[0] != q[0]['id'] or d < best[1] - STUCK_M:
                self.moved[r] = (q[0]['id'], d, now)
                continue
            if now - best[2] < NO_PROGRESS_S:
                continue
            idle = [o for o in self.rovers if o != r and not self._open(o)]
            if not idle:
                if self.cap:  # nobody idle: back into the joint plan, not left with r (r2 held M1 all of v16)
                    for m in q:
                        self._cancel(r, m)
                        m['rover'], m['from'], m['state'] = None, r, 'pending'
                        self.get_logger().warn(f"{r} got no closer to M{m['id']} in {now - best[2]:.0f} s - "
                                               'back to the plan.')
                    self._dispatch()
                self.moved.pop(r, None)
                continue
            for m in self._open(r):
                o = min(idle, key=lambda o: self._dist((m['x'], m['y']), self.pos.get(o, (0.0, 0.0))))
                self._cancel(r, m)
                m['rover'], m['from'] = o, r
                self.get_logger().warn(f"{r} made no progress on M{m['id']} - handed to idle {o}.")
                self._send(m)
                idle.remove(o)
                if not idle:
                    break
            self.moved.pop(r, None)

    def _cancel(self, r, m):
        cancel = PointStamped()
        cancel.header.stamp = self.get_clock().now().to_msg()
        cancel.header.frame_id = 'map'
        cancel.point.x, cancel.point.y = m['x'], m['y']
        self.cancel_pubs[r].publish(cancel)

    def _free_at(self, r):
        q = self._open(r)
        return (q[-1]['x'], q[-1]['y']) if q else self.pos.get(r, (0.0, 0.0))

    def _busy_m(self, r):
        """Driving left before rover r is free: through its open mushrooms in order."""
        here, total = self.pos.get(r, (0.0, 0.0)), 0.0
        for m in self._open(r):
            total += self._dist(here, (m['x'], m['y'])) + PICK_COST_M
            here = (m['x'], m['y'])
        return total

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

    def _gave_up(self, rover, msg):
        """A rover that gave up on a mushroom also reports it on cage/done: it is still there, so it goes back to
        pending for another rover (bench v5: closed as done, uncollected until a quad found it again 30 min later)."""
        x, y = msg.point.x, msg.point.y
        near = [m for m in self.mushrooms if m['rover'] == rover and math.hypot(m['x'] - x, m['y'] - y) < DONE_M]
        if not near:
            return
        m = min(near, key=lambda m: math.hypot(m['x'] - x, m['y'] - y))
        m['state'], m['from'] = 'pending', rover
        m.setdefault('gave_up', []).append(rover)
        if set(m['gave_up']) >= set(self.rovers):  # every rover failed: wait for the quads' map to fill in first
            m['gave_up'], m['hold_until'] = [], self._now() + GAVE_UP_HOLD_S
        self.get_logger().warn(f"M{m['id']} given up by {rover} - back to pending for another rover.")
        if self.cap:
            self._dispatch()

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

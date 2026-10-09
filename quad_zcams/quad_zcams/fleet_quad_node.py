"""zcam_quad_node for one quad of several: it searches its own sector and claims the mushrooms it holds.

Sectors: with two quads, nested U loops from the spawn outward, one in each half (x > 0 and x < 0), the inner legs
1 m off the seam so the two never meet there; more quads get equal x strips flown as columns. Later passes stay in
the sector.

Claims (fleet/claims, frame_id = the quad's name, stamp = when its hold started): a quad starting a hold, and every
waypoint it reports, claims the point. Other quads never hold within DISPATCHED_M of a claim, and of two holds on the
same mushroom the later one gives way.

Launched by fleet_mission.launch.py with every parrot1/* topic remapped to the quad's own namespace.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped

from ammo_mission_control import quad_search_node as qs
from quad_zcams.zcam_quad_node import ZCamQuad

SEAM_M = 1.0  # U loops turn this far from the sector seam


class _Claiming:
    """Waypoint publisher that claims every point it reports."""

    def __init__(self, node, pub):
        self.node, self.pub = node, pub

    def publish(self, msg):
        self.pub.publish(msg)
        self.node._claim(msg.point.x, msg.point.y)

    @property
    def last(self):
        return self.pub.last


class FleetQuad(ZCamQuad):

    def __init__(self):
        super().__init__()
        self.name_ = self._fleet_param('fleet_name')
        self.claim_t = 0.0
        self.claim_pub = self.create_publisher(PointStamped, 'fleet/claims', 10)
        self.create_subscription(PointStamped, 'fleet/claims', self._claim_msg, 10)
        self.waypoint_pub = _Claiming(self, self.waypoint_pub)

    def _fleet_param(self, name):
        defaults = {'fleet_name': 'parrot1', 'sector': 0, 'sectors': 1}
        if not self.has_parameter(name):
            self.declare_parameter(name, defaults[name])
        return self.get_parameter(name).value

    # -- sector -----------------------------------------------------------------
    def _center_out(self, wps):
        k, n = int(self._fleet_param('sector')), int(self._fleet_param('sectors'))
        if n <= 1:
            return super()._center_out(wps)
        h, step = max(abs(v) for v in self.area[:4]), self.area[4]
        if n == 2:
            s = 1.0 if k == 0 else -1.0
            edge = h - 1.0
            loops = int(math.ceil(edge / step))
            out = []
            for i in range(loops):
                r = edge * (i + 1) / loops
                u = [(s * SEAM_M, -r), (s * r, -r), (s * r, r), (s * SEAM_M, r)]
                out += u if i % 2 == 0 else u[::-1]
            self.area = [min(0.0, s * h), max(0.0, s * h), -h, h, step]
        else:
            x0 = -h + 2.0 * h * k / n
            self.area = [x0, x0 + 2.0 * h / n, -h, h, step]
            out = [(y, x) for x, y in qs.build_lawnmower_waypoints(-h, h, x0 + step / 2, x0 + 2.0 * h / n, step)]
        self.get_logger().info(f'Sector {k + 1}/{n}: x {self.area[0]:.1f} to {self.area[1]:.1f} m, '
                               f'{len(out)} sweep corners.')
        return out

    # -- claims -----------------------------------------------------------------
    def _claim(self, x, y):
        m = PointStamped()
        m.header.frame_id = self.name_
        sec = self.claim_t if self.holding else self._now_s()
        m.header.stamp.sec, m.header.stamp.nanosec = int(sec), int((sec % 1.0) * 1e9)
        m.point.x, m.point.y = x, y
        self.claim_pub.publish(m)

    def _claim_msg(self, m):
        if m.header.frame_id == self.name_:
            return
        p = (m.point.x, m.point.y)
        if all(math.hypot(p[0] - x, p[1] - y) > qs.DISPATCHED_M for x, y in self.dispatched):
            self.dispatched.append(p)
        if not self.holding or math.hypot(p[0] - self.hold_target[0], p[1] - self.hold_target[1]) > 2.0 * qs.DISPATCHED_M:
            return
        theirs = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if (theirs, m.header.frame_id) < (self.claim_t, self.name_):
            self.holding, self.detour = False, None
            self.ignore_detections_until = self._now_s() + qs.RESUME_BLIND_S
            self.get_logger().info(f'{m.header.frame_id} already holds ({p[0]:.1f},{p[1]:.1f}) - leaving it, resuming '
                                   'search.')

    def _det_cb(self, msg):
        was = self.holding
        super()._det_cb(msg)
        if self.holding and not was:
            self.claim_t = self._now_s()
            self._claim(*self.hold_target)


def main(args=None):
    rclpy.init(args=args)
    node = FleetQuad()
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

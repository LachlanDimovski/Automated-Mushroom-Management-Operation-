"""Native (non-invasive) mushrooms the quads have seen, for the operator.

    ros2 run ammo_gui_node native_tracker

Each quad logs 'native mushroom N logged at (x, y) - left in place'. This node
merges those across the fleet and publishes them on `fleet/natives` (JSON list,
transient-local) plus a 'native' event on `ammo/mission/events`.
"""
import json
import math
import re

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

from ammo_gui_node import contract as C
from ammo_gui_node.mission_bridge import EVENT_QOS, LIVE_QOS, ROSOUT_QOS

NATIVES_TOPIC = 'fleet/natives'
RE_NATIVE = re.compile(r'native mushroom \d+ logged at \(([-\d.]+), ([-\d.]+)\)')
MERGE_M = 2.0  # one cap seen from two sides projects up to ~1.8 m apart


class NativeTracker(Node):

    def __init__(self):
        super().__init__('native_tracker')
        self.natives = []
        self.elapsed = 0.0
        self.pub = self.create_publisher(String, NATIVES_TOPIC, EVENT_QOS)
        self.event_pub = self.create_publisher(String, C.EVENTS_TOPIC, EVENT_QOS)
        self.create_subscription(Log, C.ROSOUT, self._on_log, ROSOUT_QOS)
        self.create_subscription(String, C.STATUS_TOPIC, self._on_status, LIVE_QOS)
        self.create_subscription(String, C.EVENTS_TOPIC, self._on_event, EVENT_QOS)

    def _on_status(self, msg):
        self.elapsed = json.loads(msg.data).get('elapsed', self.elapsed)

    def _on_event(self, msg):
        if json.loads(msg.data).get('kind') == C.EV_MISSION_START:
            self.natives = []
            self._publish()

    def _on_log(self, msg):
        m = RE_NATIVE.search(msg.msg or '')
        if not m:
            return
        x, y = float(m.group(1)), float(m.group(2))
        robot = msg.name.rsplit('_', 1)[-1]
        for n in self.natives:
            if math.hypot(x - n['x'], y - n['y']) < MERGE_M:
                n['sightings'] += 1
                if robot not in n['seen_by']:
                    n['seen_by'].append(robot)
                self._publish()
                return
        n = {'n': len(self.natives) + 1, 'x': round(x, 2), 'y': round(y, 2),
             'sightings': 1, 'seen_by': [robot]}
        self.natives.append(n)
        text = f"Native mushroom N{n['n']} found at ({x:.1f}, {y:.1f}) by {robot} - left in place"
        self.get_logger().info(text)
        self.event_pub.publish(String(data=json.dumps(
            {'t': self.elapsed, 'kind': C.EV_NATIVE, 'text': text, 'n': n['n'], 'x': n['x'], 'y': n['y']})))
        self._publish()

    def _publish(self):
        self.pub.publish(String(data=json.dumps(self.natives)))


def main(args=None):
    rclpy.init(args=args)
    node = NativeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

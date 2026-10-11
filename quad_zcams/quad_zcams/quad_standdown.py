"""Quads stand down once the search has nothing left to find, then stop costing the sim.

fleet/search_done (latched) goes true when every mushroom not yet collected is already on the coordinator's list
(fleet/status) and none is 'missed', or at ammo/mission_complete. The quads hear it as their mission complete: they fly
home and land (all but parrot1, which searches on in case a listed mushroom is a false positive). A landed quad is then
parked far outside the world and put back there every PARK_EVERY_S, as it falls (the quads found all 12 by ~100 sim s
and then held the 6-quad sim at 0.22x for the rovers' remaining 300-400 s - final_r1/r2).
Probes (4 quads + 1 rover): parking took the sim from 0.53x to 0.68x; stopping the quads' sensor bridges, the YOLO pool
or switching DART to Bullet collisions changed nothing. Removing the model aborts Gazebo Fortress in Ogre's teardown.
"""
import json
import os
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

PARK_EVERY_S = 20.0
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class QuadStanddown(Node):

    def __init__(self):
        super().__init__('quad_standdown')
        self.declare_parameter('world', '')
        self.world = os.path.basename(str(self.get_parameter('world').value)).replace('.sdf', '')
        self.done, self.parked = False, []
        self.pub = self.create_publisher(Bool, 'fleet/search_done', LATCHED)
        self.pub.publish(Bool(data=False))
        self.create_subscription(String, 'fleet/status', self._status, 10)
        self.create_timer(PARK_EVERY_S, self._park_all)
        self.create_subscription(Bool, 'ammo/mission_complete', lambda m: m.data and self._stand_down('mission complete'),
                                 LATCHED)

    def _status(self, msg):
        s = json.loads(msg.data)
        total = int(s.get('total', 0))
        if not self.done and total:
            ms = s.get('mushrooms', [])
            known = sum(m['state'] in ('pending', 'assigned') for m in ms)
            missed = any(m['state'] == 'missed' for m in ms)
            left = total - sum(s.get('collected', {}).values())
            if not missed and known >= left:
                self._stand_down(f'all {left} uncollected mushrooms are known')
        if self.done and self.world:
            for q in sorted(set(s.get('landed', [])) - set(self.parked)):
                self.parked.append(q)
                self._park(q)
                self.get_logger().info(f'{q} landed - parked outside the world.')

    def _park(self, q):
        subprocess.Popen(['ign', 'service', '-s', f'/world/{self.world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
                          '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req',
                          f'name: "{q}" position: {{x: {400 + 20 * self.parked.index(q)} y: 400 z: 200}}'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _park_all(self):
        for q in self.parked:
            self._park(q)

    def _stand_down(self, why):
        if not self.done:
            self.done = True
            self.pub.publish(Bool(data=True))
            self.get_logger().info(f'Search done ({why}) - quads going home.')


def main(args=None):
    rclpy.init(args=args)
    node = QuadStanddown()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

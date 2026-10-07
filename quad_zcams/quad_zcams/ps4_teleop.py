"""PS4 / DualShock 4 teleop for both robots, safe to run alongside the autonomous mission.

  R1 held  -> drive the rover: left stick up/down = forward/back, right stick left/right = turn.
              Goes out on husky1/cmd_vel_raw, so collision_guard still vetoes driving into things.
  L1 held  -> fly the quad: left stick = forward/back + strafe, right stick = yaw (left/right) + climb (up/down).
  Release  -> that robot is stopped and handed back to its mission node, which re-plans from where it was left.

While a deadman is held the node sends <robot>/teleop_active heartbeats; rover_capture_node and zcam_quad_node stand
down while they keep arriving (they lapse 0.5 s after the last one, so a crashed teleop hands control back).
With nothing held it publishes nothing, so it never fights the mission.

RUN
  ros2 launch quad_zcams zcam_mission.launch.py ...      (ps4:=true is the default; starts joy_node too)
  ros2 run quad_zcams ps4_teleop                         (standalone; starts joy_node unless -p start_joy:=false)

FIND YOUR MAPPING if a stick or button feels wrong (they differ USB vs Bluetooth vs driver)
  ros2 run quad_zcams ps4_teleop --ros-args -p inspect:=true
      Wiggle each stick / press each button, note the numbers, then e.g. -p rover_button:=5 -p axis_turn:=3
Defaults are the kernel DualShock layout joy_node reports: L1 4, R1 5, left stick axes 0/1, right stick axes 3/4.
"""
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

DEFAULTS = {
    'joy_topic': '/joy',
    'start_joy': True,        # launch joy_node as a child; false when a launch file runs it
    'inspect': False,         # print axes/buttons, publish nothing
    'rover_button': 5,        # R1
    'quad_button': 4,         # L1
    'axis_forward': 1,        # left stick up/down
    'axis_strafe': 0,         # left stick left/right
    'axis_yaw': 3,            # right stick left/right
    'axis_climb': 4,          # right stick up/down
    'axis_turn': 3,           # rover: right stick left/right
    'rover_speed': 0.6,       # m/s at full stick
    'rover_turn': 1.0,        # rad/s at full stick
    'speed_xy': 0.6,          # quad m/s at full stick
    'speed_climb': 0.5,
    'yaw_rate': 0.8,
    'deadzone': 0.08,
    'rate_hz': 20.0,
    'joy_timeout_s': 0.5,     # no /joy for this long (unplugged, joy_node died) -> stop and hand back
}
ROBOTS = {'rover': 'husky1/cmd_vel_raw', 'quad': 'parrot1/cmd_vel'}


def deadzone(v, d):
    if abs(v) < d:
        return 0.0
    return (v - d * (1.0 if v > 0 else -1.0)) / (1.0 - d)


class Ps4Teleop(Node):

    def __init__(self):
        super().__init__('ps4_teleop')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.joy, self.joy_stamp, self.active = None, 0.0, None
        self.cmd = {r: self.create_publisher(Twist, t, 10) for r, t in ROBOTS.items()}
        prefix = {'rover': 'husky1', 'quad': 'parrot1'}
        self.beat = {r: self.create_publisher(Bool, f'{prefix[r]}/teleop_active', 10) for r in ROBOTS}
        self.create_subscription(Joy, self.p['joy_topic'], self._joy_cb, 10)
        self.create_timer(1.0 / self.p['rate_hz'], self._tick)
        self.joy_proc = None
        if self.p['start_joy']:
            self.joy_proc = subprocess.Popen(['ros2', 'run', 'joy', 'joy_node', '--ros-args',
                                              '-p', 'autorepeat_rate:=30.0', '-p', 'deadzone:=0.02'])
            self.get_logger().info(f'started joy_node (pid {self.joy_proc.pid})')
        self.get_logger().info('INSPECT - publishing nothing' if self.p['inspect'] else
                               f"PS4 teleop ready: hold R1 (button {self.p['rover_button']}) for the rover, "
                               f"L1 (button {self.p['quad_button']}) for the quad")

    def _joy_cb(self, msg):
        self.joy, self.joy_stamp = msg, time.monotonic()
        if self.p['inspect']:
            self.get_logger().info(f"axes={['%+.2f' % a for a in msg.axes]}  buttons={list(msg.buttons)}",
                                   throttle_duration_sec=0.2)

    def _axis(self, i):
        if self.joy is None or not 0 <= i < len(self.joy.axes):
            return 0.0
        return deadzone(self.joy.axes[i], self.p['deadzone'])

    def _held(self, robot):
        i = self.p[f'{robot}_button']
        return self.joy is not None and 0 <= i < len(self.joy.buttons) and bool(self.joy.buttons[i])

    def _release(self, why):
        robot, self.active = self.active, None
        for _ in range(3):  # VelocityControl holds the last command forever
            self.cmd[robot].publish(Twist())
        self.beat[robot].publish(Bool(data=False))
        self.get_logger().info(f'{robot} handed back to the mission ({why})')

    def _tick(self):
        if self.p['inspect']:
            return
        if self.joy is None or time.monotonic() - self.joy_stamp > self.p['joy_timeout_s']:
            if self.active:
                self._release('no /joy - controller unplugged?')
            return
        if self.active and not self._held(self.active):
            self._release('deadman released')
        if self.active is None:  # one robot at a time: whichever button went down first keeps it
            self.active = next((r for r in ('rover', 'quad') if self._held(r)), None)
            if self.active:
                self.get_logger().warn(f'operator has the {self.active}')
        if self.active is None:
            return
        cmd = Twist()
        if self.active == 'rover':
            cmd.linear.x = -self._axis(self.p['axis_forward']) * self.p['rover_speed']
            cmd.angular.z = -self._axis(self.p['axis_turn']) * self.p['rover_turn']
        else:
            cmd.linear.x = -self._axis(self.p['axis_forward']) * self.p['speed_xy']
            cmd.linear.y = self._axis(self.p['axis_strafe']) * self.p['speed_xy']  # this axis already runs the right way
            cmd.linear.z = -self._axis(self.p['axis_climb']) * self.p['speed_climb']
            cmd.angular.z = -self._axis(self.p['axis_yaw']) * self.p['yaw_rate']
        self.beat[self.active].publish(Bool(data=True))
        self.cmd[self.active].publish(cmd)
        self.get_logger().info(f'{self.active}  x{cmd.linear.x:+.2f} y{cmd.linear.y:+.2f} z{cmd.linear.z:+.2f} '
                               f'yaw{cmd.angular.z:+.2f}', throttle_duration_sec=1.0)

    def shutdown(self):
        if self.active:
            self._release('shutting down')
        if self.joy_proc is not None:
            self.joy_proc.terminate()


def main():
    rclpy.init()
    node = Ps4Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

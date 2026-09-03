#!/usr/bin/env python3
"""PS4 / DualShock 4 test teleop for the parrot quadcopter.

Drives /parrot1/cmd_vel from a DualShock 4, and doubles as a joystick
inspector for finding your axis/button numbers (they differ USB vs Bluetooth
vs kernel driver).

SETUP
  Plug the controller in over USB (or pair over Bluetooth), then check it's
  seen:  ls /dev/input/js*        or   ls /dev/input/by-id | grep -i sony

RUN (with the sim already up)
  python3 ps4_drone_test.py
      Starts `joy joy_node` for you. Hold L1 and fly with the sticks.
      Release L1 -> stop/hover.  Ctrl-C -> stop + quit.

FIND YOUR MAPPING first if the defaults feel wrong
  python3 ps4_drone_test.py --ros-args -p inspect:=true
      Wiggle each stick / press each button, note the numbers, then:
  python3 ps4_drone_test.py --ros-args -p axis_yaw:=2 -p deadman_button:=4

TYPICAL DS4 MAP via joy_node/SDL2 (verify with inspect mode - it does vary)
  left stick  up/down    axis 1  -> forward / back      linear.x
  left stick  left/right  axis 0  -> strafe             linear.y
  right stick left/right  axis 3  -> yaw                angular.z
  right stick up/down     axis 4  -> climb / descend    linear.z
  L1                      button 4-> deadman (hold to enable)
Some drivers shift the right stick to axes 2/3 - that's what inspect is for.

NOTE ON FRAME: this sends raw body-style commands (stick forward -> linear.x+).
If "forward" turns out to mean "world +X regardless of heading", that's the
VelocityControl frame quirk - flag it and it needs yaw compensation.
"""
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

DEFAULTS = {
    'cmd_topic': '/parrot1/cmd_vel',
    'joy_topic': '/joy',
    'start_joy': True,        # launch joy_node as a child; false if you run your own
    'inspect': False,         # print axes/buttons, publish nothing
    'deadman_button': 4,      # hold to enable motion (L1)
    'axis_forward': 1,
    'axis_strafe': 0,
    'axis_yaw': 3,
    'axis_climb': 4,
    'speed_xy': 0.6,          # m/s at full stick
    'speed_climb': 0.5,       # m/s at full stick
    'yaw_rate': 0.8,          # rad/s at full stick
    'deadzone': 0.08,
    'rate_hz': 20.0,
}


def deadzone(v, d):
    if abs(v) < d:
        return 0.0
    return (v - d * (1.0 if v > 0 else -1.0)) / (1.0 - d)


class Ps4DroneTest(Node):

    def __init__(self):
        super().__init__('ps4_drone_test')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}

        self.joy = None
        self.joy_stamp = 0.0
        self.armed_prev = False

        self.pub = self.create_publisher(Twist, self.p['cmd_topic'], 10)
        self.create_subscription(Joy, self.p['joy_topic'], self._joy_cb, 10)
        self.create_timer(1.0 / self.p['rate_hz'], self._tick)

        self.joy_proc = None
        if self.p['start_joy']:
            self.joy_proc = subprocess.Popen([
                'ros2', 'run', 'joy', 'joy_node', '--ros-args',
                '-p', 'autorepeat_rate:=30.0', '-p', 'deadzone:=0.02'])
            self.get_logger().info(f'started joy_node (pid {self.joy_proc.pid})')

        mode = ('INSPECT - no cmd_vel' if self.p['inspect']
                else f"FLY - hold button {self.p['deadman_button']} (L1)")
        self.get_logger().info(f"ps4_drone_test -> {self.p['cmd_topic']}   [{mode}]")

    def _joy_cb(self, msg):
        self.joy = msg
        self.joy_stamp = time.monotonic()
        if self.p['inspect']:
            axes = ['%+.2f' % a for a in msg.axes]
            self.get_logger().info(
                f'axes={axes}  buttons={list(msg.buttons)}',
                throttle_duration_sec=0.2)

    def _axis(self, i):
        if self.joy is None or not (0 <= i < len(self.joy.axes)):
            return 0.0
        return deadzone(self.joy.axes[i], self.p['deadzone'])

    def _button(self, i):
        if self.joy is None or not (0 <= i < len(self.joy.buttons)):
            return 0
        return self.joy.buttons[i]

    def _tick(self):
        if self.p['inspect']:
            return

        if self.joy is None or time.monotonic() - self.joy_stamp > 1.0:
            self.pub.publish(Twist())
            self.get_logger().warn(
                'no /joy - controller connected? joy_node running?',
                throttle_duration_sec=3.0)
            return

        armed = bool(self._button(self.p['deadman_button']))
        cmd = Twist()
        if armed:
            cmd.linear.x = -self._axis(self.p['axis_forward']) * self.p['speed_xy']
            cmd.linear.y = -self._axis(self.p['axis_strafe']) * self.p['speed_xy']
            cmd.linear.z = -self._axis(self.p['axis_climb']) * self.p['speed_climb']
            cmd.angular.z = -self._axis(self.p['axis_yaw']) * self.p['yaw_rate']
        self.pub.publish(cmd)

        if armed:
            self.get_logger().info(
                'ARMED  x%+.2f  y%+.2f  z%+.2f  yaw%+.2f'
                % (cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z),
                throttle_duration_sec=0.5)
        elif self.armed_prev:
            self.get_logger().info('disarmed - hovering')
        self.armed_prev = armed

    def shutdown(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)
        if self.joy_proc is not None:
            self.joy_proc.terminate()


def main():
    rclpy.init()
    node = Ps4DroneTest()
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

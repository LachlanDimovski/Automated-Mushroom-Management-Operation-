#!/usr/bin/env python3
"""Stop the rover driving into a person the detector has identified.

Sits between perception and navigation: it watches the person detections from
hazard_detection.launch.py and, when one is in the rover's way, holds the
rover off and cancels whatever Nav2 is doing.

What counts as "in the way"
---------------------------
This used to be "the bounding box is at least 120 px tall", anywhere in the
frame. Height alone is a reasonable stand-in for range, but on its own it
cannot tell a person standing in front of the rover from one standing well
off to the side - and the rover's camera sees 120 degrees, so most of what
it can see is not on its path at all. A rover that stops for someone
standing six metres off its shoulder is a rover that never finishes a run.

So the box is turned into a position instead. With a pinhole camera of known
width and field of view:

    f       = (width / 2) / tan(hfov / 2)            focal length, pixels
    range   = person_height_m * f / box_height_px    how far away they are
    bearing = atan((cx - width / 2) / f)             where, left or right
    lateral = range * tan(bearing)                   ...in metres off-axis

and a person blocks the way when they are closer than `stop_range_m` and
within `corridor_half_m` of the rover's centreline. `corridor_half_m` is the
rover's own half-width plus a margin, so the corridor is the strip the rover
would actually drive through.

`stop_height_px` is kept as a backstop: a box that tall means someone is
close enough that where they are standing stops mattering.

Turning away works for free. As the rover rotates, the person's bearing grows
and their lateral offset leaves the corridor, so forward motion is released -
which is what makes "turn, then drive away" the way out of a hazard stop
rather than something that has to be coded as a manoeuvre.

Cancelling, and not getting stuck
---------------------------------
Cancelling the goal rather than fighting over cmd_vel is deliberate. Nav2's
velocity smoother publishes cmd_vel at ~20 Hz, so a second publisher on the
same topic produces a tug of war decided by whichever message landed last -
the robot judders instead of stopping, and nothing in the system knows it is
supposed to be halted. Cancelling goes through the interface Nav2 actually
exposes, so the behaviour tree unwinds properly.

But a cancel on its own can deadlock: the people in this sim are static, so
if the mission re-dispatches the same mushroom the rover drives at the same
person and gets cancelled again, forever. Two things prevent that. The cancel
will not repeat inside `recancel_after_s`, so a re-dispatched goal gets a
chance to route around rather than being killed on the first frame. And the
hard stop on forward motion lives in `collision_guard`, off `hazard_stop`,
which leaves turning and reversing alone - so the rover can always get itself
out, and a hand-driven rover is covered too, which a Nav2 cancel does nothing
for.

    ros2 run perception hazard_brake --ros-args -r __ns:=/husky1
"""
import json
import math

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray


def focal_px(width_px, hfov_rad):
    """Focal length in pixels for an image this wide with this field of view."""
    half = max(1e-6, math.tan(float(hfov_rad) / 2.0))
    return (float(width_px) / 2.0) / half


def range_from_height(box_h_px, person_height_m, f_px):
    """How far away a person of known height appears to be, in metres.

    Returns None for a box too short to measure: at a handful of pixels the
    estimate is dominated by the box's own quantisation, and a wrong distance
    is worse than no distance.
    """
    box_h_px = float(box_h_px)
    if box_h_px < 8.0:
        return None
    return float(person_height_m) * float(f_px) / box_h_px


def offsets(cx_px, width_px, f_px, range_m):
    """(bearing in radians, lateral offset in metres) of a box centre."""
    bearing = math.atan((float(cx_px) - float(width_px) / 2.0) / float(f_px))
    return bearing, float(range_m) * math.tan(bearing)


class HazardBrake(Node):
    def __init__(self):
        super().__init__('hazard_brake')

        self.declare_parameter('hazard_classes', ['person'])
        # husky.gazebo.xacro's rgbd_camera: 720 x 480 at a 2.0944 rad h-fov.
        self.declare_parameter('image_width_px', 720.0)
        self.declare_parameter('camera_hfov_rad', 2.0944)
        # A standing adult. The estimate scales linearly with this, so being
        # 10 cm out moves the reported range by about 6 per cent - fine for a
        # gate with a metre of margin in it.
        self.declare_parameter('person_height_m', 1.75)
        # The Husky is 0.99 m across the wheels. Half of that plus half a
        # metre of margin: the strip the rover would drive through, not a point.
        self.declare_parameter('corridor_half_m', 1.0)
        self.declare_parameter('stop_range_m', 3.0)
        # ...plus however far the rover will travel before the next frame.
        #
        # This is the one that matters on a slow machine. The detector on the
        # test box runs at about 0.2 Hz - a frame every five seconds - and the
        # rover cruises at 1.3 m/s, so between one frame and the next it covers
        # six and a half metres. A fixed three-metre gate is then not a gate at
        # all: the rover can be at eight metres in one frame and past the
        # person in the next, and the stop is simply skipped. (The lidar and
        # depth guards still stop it a metre short, so it does not hit anyone -
        # but it stops as an obstacle, not as a person, which is not the
        # behaviour being demonstrated.)
        #
        # So the gate grows with speed x the measured frame interval: at 2 Hz
        # and 1.3 m/s that adds 0.65 m and the gate is essentially the 3 m it
        # says; at 0.2 Hz it opens up to about 9 m, which is the honest range
        # at which that detector can still promise to stop in time.
        self.declare_parameter('lookahead', True)
        self.declare_parameter('lookahead_max_m', 8.0)   # never a whole world away
        # Backstop, independent of bearing: a box this tall is someone close
        # enough that which way they are standing no longer matters. It has to
        # mean a SHORTER range than `stop_range_m` or it swallows the corridor
        # test whole - at this camera's 207.8 px focal length, the old 120 px
        # worked out at 3.03 m, which is exactly the corridor range, so every
        # person anywhere in a 120-degree frame tripped the backstop and the
        # bearing was never consulted. 180 px is 2.0 m.
        self.declare_parameter('stop_height_px', 180.0)
        self.declare_parameter('clear_after_s', 2.0)
        # Clearing on a timer alone is wrong whenever the detector is slower
        # than the timer. On the test machine it runs at about 0.2 Hz - a frame
        # every five seconds - so a two-second clear let the rover drive at
        # somebody for three seconds with no information at all, then stop
        # again when the next frame landed. Stuttering towards a person is a
        # worse failure than stopping for one too long.
        #
        # So clearing needs positive evidence: the detector publishes an array
        # on every inference, empty or not, so a frame with nobody blocking in
        # it is a frame that says the way is clear. Two of those, AND the timer
        # elapsed, and the rover drives on. With a fast detector that is still
        # about two seconds; with a slow one it is as slow as the detector, and
        # it should be.
        self.declare_parameter('clear_frames', 2)
        # ...unless the detector has stopped publishing altogether, in which
        # case nothing will ever produce that evidence. Fail open rather than
        # leave a rover that cannot drive forward because a node died.
        self.declare_parameter('detector_stale_s', 8.0)
        self.declare_parameter('cancel_nav', True)
        self.declare_parameter('recancel_after_s', 8.0)

        g = self.get_parameter
        self.classes = set(g('hazard_classes').value)
        self.width = float(g('image_width_px').value)
        self.hfov = float(g('camera_hfov_rad').value)
        self.person_h = float(g('person_height_m').value)
        self.corridor = float(g('corridor_half_m').value)
        self.fixed_range = float(g('stop_range_m').value)
        self.stop_height = float(g('stop_height_px').value)
        self.clear_after = float(g('clear_after_s').value)
        self.clear_frames = int(g('clear_frames').value)
        self.detector_stale = float(g('detector_stale_s').value)
        self.cancel_nav = bool(g('cancel_nav').value)
        self.lookahead = bool(g('lookahead').value)
        self.lookahead_max = float(g('lookahead_max_m').value)
        self.recancel_after = float(g('recancel_after_s').value)
        self.f = focal_px(self.width, self.hfov)

        self.stopped = False
        self.last_seen = None      # last frame with somebody blocking in it
        self.last_frame = None     # last frame at all, blocking or not
        self.clear_count = 0       # consecutive frames with nobody blocking
        self.frame_gap = None      # seconds between frames, as measured
        self.last_cancel = None
        self.worst = {}
        self.stops = 0
        self.speed = 0.0           # rover forward speed, for the lookahead

        self.create_subscription(
            Detection2DArray, 'hazards', self._hazards_cb, 10)
        # Relative, so it resolves to <ns>/odometry. Only the forward speed is
        # used, and only to widen the gate - a missing odometry topic leaves
        # the gate at its fixed range rather than breaking anything.
        self.create_subscription(Odometry, 'odometry', self._odom_cb, 10)
        self.state_pub = self.create_publisher(Bool, 'hazard_stop', 10)
        # What the GUI shows the operator: which class, how confident, how far
        # away and how far off the centreline. A bare Bool leaves "why has the
        # rover stopped" to be guessed from a console line.
        self.info_pub = self.create_publisher(String, 'hazard_info', 10)
        # Relative name: resolves to /husky1/navigate_to_pose/_action/cancel_goal
        # under this node's namespace, matching the namespaced bt_navigator.
        self.cancel = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'watching for {sorted(self.classes)} on hazards: stop within '
            f'{self.fixed_range:.1f} m (plus speed x frame interval) and '
            f'{self.corridor:.2f} m of the '
            f'centreline, or any box over {self.stop_height:.0f} px tall '
            f'(f = {self.f:.1f} px from {self.width:.0f} px / '
            f'{math.degrees(self.hfov):.0f} deg)')

    # -- detections ---------------------------------------------------------
    def _odom_cb(self, msg):
        self.speed = float(msg.twist.twist.linear.x)

    def stop_range(self):
        """How far ahead a person stops the rover, right now.

        The fixed range plus whatever ground the rover will cover before the
        detector can speak again. Reversing does not count - the gate is about
        driving into somebody.
        """
        if not self.lookahead or not self.frame_gap:
            return self.fixed_range
        return self.fixed_range + min(
            self.lookahead_max, max(0.0, self.speed) * self.frame_gap)

    def _measure(self, det):
        """One detection as a dict, or None if it is not a hazard class."""
        if not det.results:
            return None
        hyp = det.results[0].hypothesis
        if hyp.class_id not in self.classes:
            return None
        h = float(det.bbox.size_y)
        rng = range_from_height(h, self.person_h, self.f)
        close = h >= self.stop_height
        if rng is None:
            # Unmeasurable box: only the backstop can speak for it.
            return {'class': hyp.class_id, 'score': float(hyp.score),
                    'height_px': h, 'range_m': None, 'bearing_deg': None,
                    'lateral_m': None, 'in_path': close, 'blocking': close}
        bearing, lateral = offsets(det.bbox.center.position.x, self.width,
                                   self.f, rng)
        in_path = abs(lateral) <= self.corridor
        return {'class': hyp.class_id, 'score': float(hyp.score),
                'height_px': round(h, 1), 'range_m': round(rng, 2),
                'bearing_deg': round(math.degrees(bearing), 1),
                'lateral_m': round(lateral, 2), 'in_path': in_path,
                'blocking': close or (in_path and rng <= self.stop_range())}

    def _hazards_cb(self, msg):
        now = self.get_clock().now()
        if self.last_frame is not None:
            gap = (now - self.last_frame).nanoseconds / 1e9
            # Smoothed, so one slow frame does not swing the clear time about.
            self.frame_gap = gap if self.frame_gap is None else (
                0.7 * self.frame_gap + 0.3 * gap)
        self.last_frame = now
        seen = [m for m in (self._measure(d) for d in msg.detections) if m]
        blocking = [m for m in seen if m['blocking']]
        # The nearest blocking person, or - when nobody blocks - the nearest
        # one seen, so the GUI can say "a person is in view, not in the way".
        pool = blocking or seen
        if pool:
            self.worst = min(pool, key=lambda m: m['range_m']
                             if m['range_m'] is not None else 0.0)
        elif not self.stopped:
            self.worst = {}
        if not blocking:
            # A frame that saw nobody in the way. This is the evidence the
            # clear is built on, which is why it is counted here and not in
            # the timer.
            self.clear_count += 1
            return
        self.clear_count = 0
        self.last_seen = now
        if not self.stopped:
            self._engage(self.worst)

    def _engage(self, what):
        self.stopped = True
        self.stops += 1
        where = ('%.1f m ahead, %.2f m off the centreline'
                 % (what['range_m'], what['lateral_m'])
                 if what.get('range_m') is not None
                 else '%.0f px tall' % what.get('height_px', 0.0))
        self.get_logger().warn(
            'HAZARD: %s (%.2f) %s - holding the rover'
            % (what.get('class', 'person'), what.get('score', 0.0), where))
        if not self.cancel_nav:
            return
        now = self.get_clock().now()
        if self.last_cancel is not None and \
                (now - self.last_cancel).nanoseconds / 1e9 < self.recancel_after:
            # Already cancelled recently. Cancelling again here is how a
            # static person turns into an endless dispatch/cancel loop; the
            # forward block in collision_guard is holding the rover anyway.
            self.get_logger().info(
                'not cancelling again within %.0f s - letting Nav2 try to '
                'route around' % self.recancel_after)
            return
        if self.cancel.service_is_ready():
            # An empty request cancels every active goal, which is what a
            # hazard stop should do - we do not care which goal it was.
            self.cancel.call_async(CancelGoal.Request())
            self.last_cancel = now
        else:
            self.get_logger().warn(
                'nav2 cancel service not available - is Nav2 running?')

    def _tick(self):
        now = self.get_clock().now()
        if self.stopped:
            quiet = (None if self.last_frame is None
                     else (now - self.last_frame).nanoseconds / 1e9)
            age = (0.0 if self.last_seen is None
                   else (now - self.last_seen).nanoseconds / 1e9)
            # At 0.2 Hz a normal gap between frames is five seconds, so a
            # fixed eight-second "the detector has died" timeout is a coin
            # toss. Three frame intervals is the honest version of that test.
            stale = max(self.detector_stale, 3.0 * (self.frame_gap or 0.0))
            if quiet is None or quiet > stale:
                self.stopped = False
                self.clear_count = 0
                self.get_logger().warn(
                    'the person detector has not published for %.1f s - '
                    'releasing the rover rather than holding it on a node '
                    'that may be dead (expected a frame every %.1f s)'
                    % (quiet or 0.0, self.frame_gap or 0.0))
            elif self.clear_count >= self.clear_frames and age > self.clear_after:
                self.stopped = False
                self.get_logger().info(
                    '%d frames with nobody in the way over %.1f s - the rover '
                    'may drive on' % (self.clear_count, age))
        self.state_pub.publish(Bool(data=self.stopped))
        info = dict(self.worst)
        info['stopped'] = self.stopped
        info['stops'] = self.stops
        # How fast the detector is actually running. At a frame every few
        # seconds the stop is as coarse as the detector, and that is worth
        # being able to read rather than guess at from the rover's behaviour.
        info['detector_hz'] = (None if not self.frame_gap
                               else round(1.0 / self.frame_gap, 2))
        info['clear_frames_seen'] = self.clear_count
        info['stop_range_m'] = round(self.stop_range(), 2)
        info['speed_ms'] = round(self.speed, 2)
        self.info_pub.publish(String(data=json.dumps(info)))


def main(args=None):
    rclpy.init(args=args)
    node = HazardBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

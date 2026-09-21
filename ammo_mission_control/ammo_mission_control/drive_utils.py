"""Shared steering math for the mission nodes (follow/mushroom/search), all
of which drive a robot toward a world-frame point using the same
angle-then-speed shape as skidsteer_controller's driveToWaypoint.
"""
import math


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def steer_angular(dx, dy, yaw, max_angular, angular_gain):
    """Angular velocity to turn toward (dx, dy) from current yaw, and the
    signed heading error it's based on (callers gate speed on how
    well-aimed they already are)."""
    target_angle = math.atan2(dy, dx)
    angle_diff = math.atan2(math.sin(target_angle - yaw), math.cos(target_angle - yaw))
    angular = max(-max_angular, min(max_angular, angle_diff * angular_gain))
    return angle_diff, angular

"""Ground projection of a quad-camera detection: pixel -> world (x, y).

Verified live: 0-3 cm error at 2.8-5.6 m range once the quad is level and
still (a moving/turning frame read 5-8 m off, which is why the bridge waits).
"""
import math

# Parrot camera geometry (parrot.gazebo.xacro/parrot.urdf.xacro): 720x480,
# 120 deg horizontal FOV, mounted with a fixed +45 deg pitch (rpy="0
# 0.785398 0" on camera_joint) - i.e. tilted down-and-forward from level,
# matching a search camera meant to see the ground ahead while flying.
IMG_W, IMG_H = 720, 480
CAM_HFOV_RAD = 2.0944
FOCAL_PX = (IMG_W / 2.0) / math.tan(CAM_HFOV_RAD / 2.0)
CAM_PITCH_RAD = 0.785398
# camera_joint sits 0.25 m ahead of and 0.25 m above parrot base_link; the
# bbox centre is on the cap (~0.12 m up), not the ground. Ignoring these read
# 0.43 m short live.
CAM_FWD_M, CAM_UP_M, TARGET_HEIGHT_M = 0.25, 0.25, 0.12
# Beyond this a mushroom is a few pixels and a one-row bbox error near the
# horizon swings the range by tens of metres (a single such frame sent both
# robots off the map live).
MAX_PROJECT_RANGE_M = 8.0


def project_to_ground(quad_x, quad_y, quad_yaw, quad_alt, px, py, ground_z=0.0):
    """World (x, y) where the camera ray through pixel (px, py) meets the
    mushroom-cap plane, or None if it never does. Full ray-cast through the
    camera's pitch, not separate yaw/pitch pixel angles: with the camera
    tilted 45 deg, a pixel's horizontal angle maps to a larger ground bearing
    (tan(a)/cos(pitch)), which read 0.4 m lateral error at 3 m live. Assumes
    level flight (roll/pitch ~ 0) and flat ground.
    """
    # camera pose: CAM_FWD_M ahead of and CAM_UP_M above base_link
    cx = quad_x + CAM_FWD_M * math.cos(quad_yaw)
    cy = quad_y + CAM_FWD_M * math.sin(quad_yaw)
    cz = quad_alt + CAM_UP_M
    # ray in camera frame (x forward, y left, z up), pinhole
    rx, ry, rz = FOCAL_PX, -(px - IMG_W / 2.0), -(py - IMG_H / 2.0)
    # pitch the camera frame down by CAM_PITCH_RAD (about +y), then yaw into world
    cp, sp = math.cos(CAM_PITCH_RAD), math.sin(CAM_PITCH_RAD)
    bx, by, bz = rx * cp + rz * sp, ry, -rx * sp + rz * cp
    cy_, sy_ = math.cos(quad_yaw), math.sin(quad_yaw)
    wx, wy, wz = bx * cy_ - by * sy_, bx * sy_ + by * cy_, bz
    if wz >= -1e-6:
        return None  # ray at/above the horizon
    t = (ground_z + TARGET_HEIGHT_M - cz) / wz
    if t <= 0.0 or math.hypot(t * wx, t * wy) > MAX_PROJECT_RANGE_M:
        return None
    return cx + t * wx, cy + t * wy

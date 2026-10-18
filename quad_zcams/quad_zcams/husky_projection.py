"""Ground projection for the Husky's camera: detection pixel -> world (x, y).

The quad has its own version in ammo_mission_control/ground_projection.py. The
maths is the same pinhole ray-cast, but every mount constant differs and that
file hardcodes the Parrot's, so this is a separate implementation rather than a
parameterisation - the quad's path is on the critical path for the demo and is
not worth disturbing for this.

    Husky                      Parrot
    0.45 m ahead of base_link  0.25 m
    0.25 m above base_link     0.25 m
    25 deg down                45 deg down
    sits on the ground         flies, altitude from odometry

Assumes the rover is level (roll/pitch ~ 0, true on the flat demo worlds) and
the ground under the mushroom is at the same height as the ground under the
rover.
"""
import math

# --- camera intrinsics, from urdf_husky/husky.gazebo.xacro -------------------
IMG_W, IMG_H = 720, 480
CAM_HFOV_RAD = 2.0944                                    # 120 deg
# Pinhole focal length in pixels: half the image (360 px) subtends half the
# field of view (60 deg) from the lens, so tan(60) = 360 / f.
FOCAL_PX = (IMG_W / 2.0) / math.tan(CAM_HFOV_RAD / 2.0)  # 207.85

# --- camera mount, from urdf_husky/husky.urdf.xacro camera_joint -------------
CAM_FWD_M = 0.45          # ahead of base_link
CAM_UP_M = 0.25           # above base_link
CAM_PITCH_RAD = 0.4363    # 25 deg down; the xacro's camera_pitch arg

# base_link itself rides above the ground on the wheels. Odometry reports
# z = 0.0 (it is 2D), so this has to come from the model: it matches
# collision_guard's base_height_m and the pose Gazebo reports for husky1.
# Leaving it out puts every projection consistently short.
BASE_HEIGHT_M = 0.1323

# The bounding box centres on the cap, not on the ground the mushroom stands
# on, so the ray is intersected with the cap's height rather than z = 0.
TARGET_HEIGHT_M = 0.12

# Past this a mushroom is a handful of pixels and one row of bbox error swings
# the range by metres. The rover only ever drives to points the quad found, so
# there is no reason to trust a long-range estimate of its own.
MAX_RANGE_M = 6.0


def bearing_of(px):
    """Horizontal angle to a detection, radians, positive to the left.

    Useful on its own: this is the signal a visual servo steers on, and it
    needs no range, no ground plane and no pose - just the pixel.
    """
    return math.atan2((IMG_W / 2.0) - px, FOCAL_PX)


def camera_ray(rover_x, rover_y, rover_yaw, px, py):
    """Camera centre and a world-frame direction through pixel (px, py).

    The ray is not normalised; only its direction matters to the caller.
    """
    # Camera position in the world. The rover is on the ground, so its height
    # is fixed rather than read from odometry.
    cx = rover_x + CAM_FWD_M * math.cos(rover_yaw)
    cy = rover_y + CAM_FWD_M * math.sin(rover_yaw)
    cz = BASE_HEIGHT_M + CAM_UP_M

    # Ray in the camera's own frame: x forward, y left, z up. Image coordinates
    # run right and down, so both pixel terms are negated.
    rx = FOCAL_PX
    ry = -(px - IMG_W / 2.0)
    rz = -(py - IMG_H / 2.0)

    # Pitch the camera frame down about +y. Check: (1,0,0) -> (cos p, 0, -sin p),
    # i.e. straight ahead tilts downward, which is what a downward pitch means.
    cp, sp = math.cos(CAM_PITCH_RAD), math.sin(CAM_PITCH_RAD)
    bx, by, bz = rx * cp + rz * sp, ry, -rx * sp + rz * cp

    # Yaw into the world frame.
    cy_, sy_ = math.cos(rover_yaw), math.sin(rover_yaw)
    return (cx, cy, cz), (bx * cy_ - by * sy_, bx * sy_ + by * cy_, bz)


def project(rover_x, rover_y, rover_yaw, px, py):
    """World (x, y) where the ray through pixel (px, py) meets the cap plane.

    Returns None when the ray never lands: at or above the horizon, behind the
    camera, or beyond MAX_RANGE_M.
    """
    c, w = camera_ray(rover_x, rover_y, rover_yaw, px, py)
    if w[2] >= -1e-6:
        return None                                  # at or above the horizon
    t = (TARGET_HEIGHT_M - c[2]) / w[2]
    if t <= 0.0:
        return None                                  # behind the camera
    gx, gy = c[0] + t * w[0], c[1] + t * w[1]
    if math.hypot(gx - rover_x, gy - rover_y) > MAX_RANGE_M:
        return None
    return gx, gy

"""What the quad has seen at the rover's height, for the rover to plan with.

The quad's forward depth camera (45 deg down) sees the ground and whatever stands on it 0.5-4.5 m ahead. Every frame's
pixels are placed in the world: a point BAND above the local ground is in the rover's way (collision_guard's depth
band), a point on it is open ground. The local ground is each frame's own lowest point within GROUND_WIN cells, but
no more than RISE_MAX_M above the ground under the quad (under a wide crown no ground shows, and its lowest leaves
would pass for it): the terrain rises and falls by about a metre over the dense world, and measuring heights from the
ground under the quad marked rising ground ahead as obstacles (only 52% of obstacle cells were real). Counted per cell and published
as an OccupancyGrid (100 obstacle, 0 open, -1 not seen).
"""
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage

from ammo_mission_control.ground_projection import CAM_FWD_M, CAM_PITCH_RAD, CAM_UP_M, FOCAL_PX

RES = 0.25
HALF = 20.0
BAND = (0.25, 1.2)        # above the ground: the rover's guard stops for 0.15-1.2 m, less a margin for slope
GROUND_TOL_M = 0.12       # this close to the local ground: open ground
MAX_RANGE_M = 4.5         # a degree of unmeasured tilt is 8 cm of height error here
GROUND_WIN = 7            # cells (1.75 m): the local ground is the lowest point seen this close...
RISE_MAX_M = 0.4          # ...and at most this far above the ground under the quad
STEP_PX = 6               # every 6th pixel each way
MIN_HITS = 2              # seen in at least this many frames...
HIT_SHARE = 0.3           # ...and in this share of the frames that saw the cell at all


class QuadMap:

    def __init__(self, half=HALF):
        self.half = half
        self.n = int(round(2 * half / RES))
        self.hit = np.zeros((self.n, self.n), np.int32)
        self.open = np.zeros((self.n, self.n), np.int32)
        self._rays = None

    def add_depth(self, depth, x, y, yaw, z, agl):
        """One depth frame (metres along the camera axis) taken at (x, y, yaw), base_link at height z (world), agl
        above the ground below."""
        if self._rays is None or self._rays[0] != depth.shape:
            h, w = depth.shape
            v, u = np.mgrid[STEP_PX // 2:h:STEP_PX, STEP_PX // 2:w:STEP_PX]
            rx, ry, rz = FOCAL_PX, -(u - w / 2.0), -(v - h / 2.0)
            cp, sp = math.cos(CAM_PITCH_RAD), math.sin(CAM_PITCH_RAD)
            self._rays = (depth.shape, v, u, rx * cp + rz * sp, ry, -rx * sp + rz * cp)
        _, v, u, bx, by, bz = self._rays
        d = depth[v, u]
        ok = np.isfinite(d) & (d > 0.2)
        t = np.where(ok, d, 0.0) / FOCAL_PX
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + CAM_FWD_M * c + t * (bx * c - by * s)
        py = y + CAM_FWD_M * s + t * (bx * s + by * c)
        pz = z + CAM_UP_M + t * bz
        ok &= np.hypot(px - x, py - y) < MAX_RANGE_M
        i = np.floor((px + self.half) / RES).astype(int)
        j = np.floor((py + self.half) / RES).astype(int)
        ok &= (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        i, j, pz = i[ok], j[ok], pz[ok]
        low = np.full((self.n, self.n), np.inf)
        np.minimum.at(low, (i, j), pz)
        ground = np.minimum(ndimage.minimum_filter(low, size=GROUND_WIN), z - agl + RISE_MAX_M)
        h = pz - ground[i, j]
        hit = (h > BAND[0]) & (h < BAND[1])
        opn = h < GROUND_TOL_M
        # once per cell per frame
        for mask, acc in ((hit, self.hit), (opn, self.open)):
            cells = np.unique(i[mask] * self.n + j[mask])
            acc.ravel()[cells] += 1

    def grid(self):
        """int8 cells: 100 obstacle, 0 open, -1 not seen."""
        seen = self.hit + self.open
        occ = (self.hit >= MIN_HITS) & (self.hit >= HIT_SHARE * seen)
        out = np.full((self.n, self.n), -1, np.int8)
        out[(self.open >= MIN_HITS) & ~occ] = 0
        out[occ] = 100
        return out

    def message(self, stamp, frame='husky1_map'):
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = stamp, frame
        m.info.resolution = RES
        m.info.width = m.info.height = self.n
        m.info.origin.position.x = m.info.origin.position.y = -self.half
        m.info.origin.orientation.w = 1.0
        m.data = self.grid().T.ravel().tolist()  # OccupancyGrid is row-major in y
        return m


def from_message(m):
    """(occ, known) bool arrays indexed [i (x), j (y)] from a QuadMap message."""
    a = np.array(m.data, np.int8).reshape(m.info.height, m.info.width).T
    return a == 100, a >= 0

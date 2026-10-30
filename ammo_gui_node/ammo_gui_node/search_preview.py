"""The path the quadcopter will fly, worked out before the run starts.

The setup page offers two search patterns and, until now, described them in
words. Words are a poor way to choose between "square loops outward" and
"lawnmower rows": the thing you actually want to know is where the quad
spends its first minute, because that is where the first mushrooms get found
and therefore where the rover is sent first.

So this builds the same corner list the quad's own node builds, and the
preview draws it. It is deliberately a copy of their arithmetic rather than
an impression of it - if their pattern changes, this should be changed with
it, and the test beside it compares the two directly.

Their versions, for reference:

  - `quad_search_node.build_lawnmower_waypoints` - rows at `spacing` apart,
    alternating direction, so the path is a zig-zag with square corners.
  - `zcam_quad_node._center_out` - closed squares at growing radii, the
    outermost one metre inside the edge of the sweep. Not a spiral: each
    loop is closed before the next, larger one begins.

Both are given the search area the launch file derives from the world,
`[-half, half, -half, half, row_spacing]`, which is why `half` here is the
world's playable half-size - the same number the map draws its edge box from.
"""

import math

# Their `_center_out` keeps the outermost loop this far inside the edge of the
# sweep: a loop on the edge itself flies past the outermost mushrooms at the
# very side of the camera's view, where the detector is weakest.
EDGE_INSET_M = 1.0

# How `zcam_mission.launch.py` sizes the sweep: the playable half-size pulled
# in by SWEEP_MARGIN_M, or DEFAULT_HALF_M on a world with no meta file at all
# (`simple_trees`). Copied from lines 77-82 of that launch file, so the
# preview is the area they will actually search rather than the whole map.
SWEEP_MARGIN_M = 4.5
DEFAULT_HALF_M = 6.5


def search_half(meta):
    """The half-size of the square the quad will sweep, from a world's meta."""
    size = ((meta or {}).get('world') or {}).get('playable_size_m')
    try:
        return float(size) / 2.0 - SWEEP_MARGIN_M
    except (TypeError, ValueError):
        return DEFAULT_HALF_M


def lawnmower(x_min, x_max, y_min, y_max, spacing):
    """Rows `spacing` apart, alternating direction. A zig-zag with corners."""
    if spacing <= 0:
        return []
    out = []
    y, i = y_min, 0
    while y <= y_max + 1e-6:
        out += [(x_min, y), (x_max, y)] if i % 2 == 0 else [(x_max, y), (x_min, y)]
        y, i = y + spacing, i + 1
    return out


def square_loops(half, spacing):
    """Closed squares at growing radii, from the spawn outward."""
    edge = half - EDGE_INSET_M
    if edge <= 0 or spacing <= 0:
        return []
    n = int(math.ceil(edge / spacing))
    out = []
    for k in range(n):
        r = edge * (k + 1) / n
        out += [(r, -r), (r, r), (-r, r), (-r, -r), (r, -r)]
    return out


def search_path(pattern, half, spacing):
    """[(x, y)] the quad will fly for this pattern, or [] if it cannot be drawn.

    `pattern` is the setup page's own key: 'center_out' or 'rows'. Anything
    else returns nothing rather than guessing, so a pattern added later shows
    no preview instead of a wrong one.
    """
    try:
        half = float(half)
        spacing = float(spacing)
    except (TypeError, ValueError):
        return []
    if half <= 0:
        return []
    if pattern == 'center_out':
        return square_loops(half, spacing)
    if pattern == 'rows':
        return lawnmower(-half, half, -half, half, spacing)
    return []


def path_length_m(points):
    """How far the quad flies to complete one pass. Worth showing: the two
    patterns cover the same ground and are not the same length."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total

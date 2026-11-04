"""Where a point on the ground lands in the overhead camera's picture.

The overhead camera is the one `mission_launcher` spawns: a pinhole camera
at (0, 0, H) over the middle of the world, pitched a quarter turn about Y so
it looks straight down. Because it is straight down, flat and centred, the
projection is a scale and two sign flips rather than a full camera model -
no distortion, no pose lookup, no TF.

Working it through, since the signs are the part that is easy to get wrong.
Ignition's camera frame is +X along the optical axis, +Y left, +Z up, and
the image runs right along -Y and down along -Z. A pitch of +pi/2 about the
world Y axis maps the camera frame onto the world like this:

    x_c (optical) -> -Z_w     the camera looks down
    y_c           -> +Y_w
    z_c           -> +X_w

So a ground point (x, y, 0) seen from (0, 0, H) is H deep, y across and x
up, and

    u = cu - f * y / H
    v = cv - f * x / H          with f = (px / 2) / tan(hfov / 2)

which puts world +X up the picture and world +Y to the left.

If that turns out to be mirrored on a real frame - the two flips below are
the only thing that could be wrong, and a screenshot settles it in seconds -
flip `U_SIGN` or `V_SIGN`. Everything that draws an overlay goes through
`project`, so one sign here fixes every caller. The robots are drawn as well
as the mushrooms for exactly this reason: a robot is visible in the picture,
so if its circle sits on it, the mushroom circles can be believed.
"""
import math

from ammo_gui_node import theme

U_SIGN = -1.0   # world +Y goes left in the image
V_SIGN = -1.0   # world +X goes up in the image


def focal_px(px, hfov):
    """Focal length in pixels for a `px`-wide image with this field of view."""
    half = max(1e-6, math.tan(float(hfov) / 2.0))
    return (float(px) / 2.0) / half


def ground_half_width(height, hfov):
    """Half the width of the ground the camera can see, in metres."""
    return float(height) * math.tan(float(hfov) / 2.0)


def project(x, y, height, px, py=None, hfov=1.5):
    """World (x, y) on the ground -> (u, v) pixels in a px-wide image.

    Returns None when the point is off the picture, so a caller can simply
    skip it rather than drawing a circle against the frame edge.
    """
    height = float(height)
    if height <= 0.0:
        return None
    py = float(py if py is not None else px)
    f = focal_px(px, hfov)
    u = px / 2.0 + U_SIGN * f * float(y) / height
    v = py / 2.0 + V_SIGN * f * float(x) / height
    if not (0.0 <= u <= px and 0.0 <= v <= py):
        return None
    return u, v


def metres_to_px(metres, height, px, hfov=1.5):
    """A length on the ground, in pixels - for sizing a circle."""
    height = float(height)
    if height <= 0.0:
        return 0.0
    return focal_px(px, hfov) * float(metres) / height


# How a mushroom is drawn on the overhead view, by what the mission knows of
# it. Red is the default because a mushroom nobody has found yet is the thing
# the operator is looking for; the rest are the states it can reach.
MARK_COLOURS = {
    'found': theme.AMBER,          # sighted, not collected yet
    'collected': theme.GREEN,
    'abandoned': theme.GREY,       # given up on
    'missed': theme.PURPLE,        # gripped, nothing there
}
MARK_UNFOUND = theme.ACCENT        # still out there
QUAD_COLOUR = theme.TEAL
ROVER_COLOUR = theme.BLUE
# People. Not in MARK_COLOURS because that dict is keyed by a mushroom's
# status, and a person has none - they are not something the mission is
# trying to do anything with.
PERSON_COLOUR = theme.ORCHID
PERSON_R_M = 0.8                   # shoulder width, near enough, for sizing
MARK_NEAR_M = 1.5                  # how close a report has to be to be this one

# What the note under the picture says, so the colours do not need explaining.
MARK_KEY = ('circles: mushrooms - red out there, amber seen, green collected   '
            'boxes: teal quad, blue skidsteer   '
            'pink head: person')


def screen_angle(yaw):
    """A world heading as a clockwise screen angle on the overhead picture.

    Not the same as on the map, and the difference is why a robot symbol
    drawn with the map's rule points the wrong way here. This camera puts
    +y to the left and +x upward (see `project` above), so a world step
    (dx, dy) moves the picture by (-dy, -dx) - and a heading of `yaw`,
    which is (cos yaw, sin yaw) in the world, becomes (-sin yaw, -cos yaw)
    on screen. Qt measures rotation clockwise with v downward, so that is
    atan2(-cos yaw, -sin yaw), which simplifies to 270 degrees - yaw.
    """
    return (270.0 - math.degrees(yaw)) % 360.0


def overhead_marks(placed, mushrooms, robots, height=None, px=None, hfov=None,
                   people=None):
    """Marks for the overhead camera: every mushroom, and every robot.

    `placed` is where the GUI put the mushrooms, `mushrooms` is what the
    mission has reported so far (so a found or collected one is drawn
    differently), and `robots` is {key: (x, y)}.

    The robots are not decoration - they are the check. A robot is visible in
    the picture, so if its box sits on it, the mushroom circles are in the
    right place too. They are drawn as squares and the mushrooms as circles,
    so the two are never told apart by colour alone.

    `people` is where the GUI stood the hazards. They get a third shape of
    their own - neither circle nor square - so the picture never asks anyone
    to tell a person from a mushroom by colour.

    `height`, `px` and `hfov` describe the camera. The callers in the two
    windows pass the ones for the world in question; the defaults are the
    camera `mission_launcher` spawns over a 60 m world.
    """
    height = float(height if height is not None else 34.3)
    px = int(px if px is not None else 640)
    hfov = float(hfov if hfov is not None else 1.5)
    marks = []
    found = []
    for m in (mushrooms or []):
        try:
            found.append((float(m.get('x', 0.0)), float(m.get('y', 0.0)),
                          m.get('status')))
        except (TypeError, ValueError):
            continue
    for i, point in enumerate(placed or [], 1):
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        at = project(x, y, height, px, hfov=hfov)
        if at is None:
            continue
        near = min((math.hypot(x - fx, y - fy), st) for fx, fy, st in found) if found else None
        state = near[1] if near and near[0] < MARK_NEAR_M else None
        marks.append({'u': at[0], 'v': at[1], 'at': (x, y), 'mushroom': True,
                      'kind': 'mushroom', 'shape': 'circle', 'state': state,
                      'r': max(7.0, metres_to_px(0.9, height, px, hfov)),
                      'colour': MARK_COLOURS.get(state, MARK_UNFOUND),
                      'label': f'M{i}'})
    for i, point in enumerate(people or [], 1):
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        at = project(x, y, height, px, hfov=hfov)
        if at is None:
            continue
        marks.append({'u': at[0], 'v': at[1], 'at': (x, y), 'mushroom': False,
                      'kind': 'person', 'shape': 'person',
                      'r': max(7.0, metres_to_px(PERSON_R_M, height, px, hfov)),
                      'colour': PERSON_COLOUR, 'label': f'H{i}'})
    for key, pos in (robots or {}).items():
        try:
            x, y = float(pos[0]), float(pos[1])
        except (TypeError, ValueError, IndexError):
            continue
        # (x, y) or (x, y, yaw) - the heading is only needed by the symbol
        # drawing, so callers that have it pass it and the rest do not.
        try:
            yaw = float(pos[2])
        except (TypeError, ValueError, IndexError):
            yaw = 0.0
        at = project(x, y, height, px, hfov=hfov)
        if at is None:
            continue
        quad = str(key).startswith('quad')
        label = ('Q' + str(key)[4:]) if quad else ('S' + str(key)[5:])
        marks.append({'u': at[0], 'v': at[1], 'at': (x, y), 'mushroom': False,
                      'kind': 'quad' if quad else 'rover', 'yaw': yaw,
                      'angle_deg': screen_angle(yaw), 'shape': 'box',
                      'r': max(9.0, metres_to_px(1.2, height, px, hfov)),
                      'colour': QUAD_COLOUR if quad else ROVER_COLOUR,
                      'label': label})
    return marks

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

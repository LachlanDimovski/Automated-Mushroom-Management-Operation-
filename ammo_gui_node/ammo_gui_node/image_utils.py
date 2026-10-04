"""sensor_msgs/Image -> QImage, with numpy and nothing else.

`cv_bridge` would do this in one line, but it drags OpenCV into this package
and the whole point of building on Qt was to not depend on cv2 for display.
A ROS Image is just a byte buffer plus a width, a height, a row stride and
an encoding name, so the conversion is a reshape and at most a channel swap.

Every returned QImage is `.copy()`d so it owns its pixels. Without that the
QImage would keep pointing at a numpy buffer that Python is free to collect
the moment the callback returns, which shows up later as a torn or garbage
frame rather than a clean crash.
"""
import numpy as np
from PySide6.QtGui import QImage


def _rows(msg, dtype, channels):
    """The message buffer as (height, width, channels), stride removed.

    `step` is the number of bytes per row and is not always width*channels -
    some publishers pad rows - so index by step first and trim afterwards.
    """
    itemsize = np.dtype(dtype).itemsize
    flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    expected = msg.step * msg.height
    if flat.size < expected:
        return None
    rows = flat[:expected].reshape(msg.height, msg.step)
    usable = msg.width * channels * itemsize
    rows = rows[:, :usable]
    out = rows.reshape(msg.height, msg.width, channels * itemsize)
    if itemsize == 1:
        return out
    return out.view(dtype).reshape(msg.height, msg.width, channels)


def _to_rgb888(rgb):
    """A contiguous HxWx3 uint8 array -> QImage that owns its data."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w = rgb.shape[:2]
    return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()


def _normalise(a, lo=None, hi=None):
    """Float or 16-bit data -> 0..255, ignoring NaN and infinities."""
    a = np.nan_to_num(np.asarray(a, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    lo = float(a.min()) if lo is None else lo
    hi = float(a.max()) if hi is None else hi
    if hi - lo < 1e-6:
        return np.zeros(a.shape, dtype=np.uint8)
    return (np.clip((a - lo) / (hi - lo), 0.0, 1.0) * 255.0).astype(np.uint8)


def qimage_from_ros(msg):
    """Convert a sensor_msgs/Image. Returns None for anything unrecognised.

    Returning None rather than raising is deliberate: this runs in a ROS
    callback on every frame, and one odd encoding should blank a camera
    panel, not take the window down.
    """
    try:
        enc = (msg.encoding or '').lower()

        if enc in ('rgb8', 'rgb'):
            return _to_rgb888(_rows(msg, np.uint8, 3))

        if enc in ('bgr8', 'bgr'):
            bgr = _rows(msg, np.uint8, 3)
            return None if bgr is None else _to_rgb888(bgr[:, :, ::-1])

        if enc in ('rgba8', 'rgba'):
            rgba = _rows(msg, np.uint8, 4)
            return None if rgba is None else _to_rgb888(rgba[:, :, :3])

        if enc in ('bgra8', 'bgra'):
            bgra = _rows(msg, np.uint8, 4)
            return None if bgra is None else _to_rgb888(bgra[:, :, 2::-1])

        if enc in ('mono8', '8uc1'):
            g = _rows(msg, np.uint8, 1)
            return None if g is None else _to_rgb888(np.repeat(g, 3, axis=2))

        if enc in ('mono16', '16uc1'):
            g = _rows(msg, np.uint16, 1)
            if g is None:
                return None
            g8 = _normalise(g[:, :, 0])
            return _to_rgb888(np.dstack([g8, g8, g8]))

        if enc in ('32fc1', '64fc1'):
            # Depth. Near is dark, far is bright, tinted blue - the same
            # reading the dashboard's depth views use.
            d = _rows(msg, np.float32 if enc == '32fc1' else np.float64, 1)
            if d is None:
                return None
            v = _normalise(d[:, :, 0], lo=0.0, hi=8.0)
            return _to_rgb888(np.dstack([(v * 0.30).astype(np.uint8),
                                         (v * 0.60).astype(np.uint8), v]))
    except Exception:
        return None
    return None


def qimage_from_compressed(msg):
    """sensor_msgs/CompressedImage -> QImage, via Qt's own JPEG/PNG readers."""
    try:
        img = QImage()
        if img.loadFromData(bytes(msg.data)):
            return img
    except Exception:
        pass
    return None

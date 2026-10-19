"""Attribute bags that stand in for ROS messages in the headless tests.

A generated message class is just a struct with defaults, so a hand-rolled
one is close enough for the GUI: every field the code reads exists, nested
fields are built on construction, and keyword arguments work the way rclpy's
do (`String(data='x')`).
"""


def bag(_cls, **defaults):
    """Build a message-like class whose fields default sensibly.

    A default that is callable is called per instance, so nested messages
    and lists are not shared between objects.
    """

    class _Bag:
        _defaults = defaults

        def __init__(self, **kwargs):
            for key, value in self._defaults.items():
                setattr(self, key, value() if callable(value) else value)
            for key, value in kwargs.items():
                setattr(self, key, value)

        def __repr__(self):
            inner = ', '.join(f'{k}={getattr(self, k)!r}' for k in self._defaults)
            return f'{_cls}({inner})'

    _Bag.__name__ = _Bag.__qualname__ = _cls
    return _Bag


Vector3 = bag('Vector3', x=0.0, y=0.0, z=0.0)
Point = bag('Point', x=0.0, y=0.0, z=0.0)
Quaternion = bag('Quaternion', x=0.0, y=0.0, z=0.0, w=1.0)
Header = bag('Header', frame_id='', stamp=None)

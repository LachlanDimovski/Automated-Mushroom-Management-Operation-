"""Enough of rclpy to construct the GUI's nodes and drive them by hand.

Publishers record what they were given, subscriptions are callable by topic,
and timers are collected rather than fired, so a test can step the node
deliberately instead of waiting on a clock.
"""
_initialised = False


def init(args=None):
    global _initialised
    _initialised = True


def shutdown():
    global _initialised
    _initialised = False


def ok():
    return _initialised


def spin(node):
    raise RuntimeError('the stub rclpy never spins - drive the node directly')


def spin_once(node, timeout_sec=0.0):
    return None

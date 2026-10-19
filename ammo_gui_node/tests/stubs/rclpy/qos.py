class _Enum:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


class ReliabilityPolicy:
    BEST_EFFORT = _Enum('BEST_EFFORT')
    RELIABLE = _Enum('RELIABLE')


class DurabilityPolicy:
    VOLATILE = _Enum('VOLATILE')
    TRANSIENT_LOCAL = _Enum('TRANSIENT_LOCAL')


class HistoryPolicy:
    KEEP_LAST = _Enum('KEEP_LAST')
    KEEP_ALL = _Enum('KEEP_ALL')


class QoSProfile:
    def __init__(self, depth=10, reliability=None, durability=None, history=None):
        self.depth = depth
        self.reliability = reliability or ReliabilityPolicy.RELIABLE
        self.durability = durability or DurabilityPolicy.VOLATILE
        self.history = history or HistoryPolicy.KEEP_LAST

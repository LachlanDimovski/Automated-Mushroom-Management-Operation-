"""A Node that records instead of talking to a middleware."""
import time


class _Clock:
    class _Now:
        def __init__(self):
            self.nanoseconds = int(time.monotonic() * 1e9)

        def to_msg(self):
            return None

    def now(self):
        return self._Now()


class _Logger:
    def __init__(self, sink):
        self.sink = sink

    def info(self, text):
        self.sink.append(('info', str(text)))

    def warn(self, text):
        self.sink.append(('warn', str(text)))

    def error(self, text):
        self.sink.append(('error', str(text)))

    def debug(self, text):
        self.sink.append(('debug', str(text)))


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)

    @property
    def last(self):
        return self.sent[-1] if self.sent else None


class Timer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback


class _Param:
    def __init__(self, value):
        self.value = value


class Node:
    def __init__(self, name):
        self._name = name
        self.publishers_by_topic = {}
        self.subs = {}
        self.timers = []
        self.params = {}
        self.log_lines = []
        # How many publishers the graph claims each topic has. Tests set it;
        # the real Node answers from the ROS graph.
        self.graph_publishers = {}
        self._logger = _Logger(self.log_lines)
        self._clock = _Clock()

    # -- the rclpy surface the GUI uses -------------------------------------
    def create_publisher(self, msg_type, topic, qos=10):
        pub = Publisher(topic)
        self.publishers_by_topic[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos=10):
        self.subs.setdefault(topic, []).append(callback)
        return callback

    def create_timer(self, period, callback):
        timer = Timer(period, callback)
        self.timers.append(timer)
        return timer

    def create_client(self, srv_type, name):
        return _Client(name)

    def declare_parameter(self, name, default=None):
        self.params[name] = _Param(default)
        return self.params[name]

    def get_parameter(self, name):
        return self.params.get(name, _Param(None))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def count_publishers(self, topic):
        return int(self.graph_publishers.get(topic, 0))

    def destroy_node(self):
        pass

    # -- test helpers -------------------------------------------------------
    def feed(self, topic, msg):
        """Deliver a message to every subscriber of a topic."""
        for callback in self.subs.get(topic, []):
            callback(msg)

    def fire_timers(self):
        for timer in list(self.timers):
            timer.callback()

    def sent(self, topic):
        pub = self.publishers_by_topic.get(topic)
        return pub.sent if pub else []


class _Client:
    def __init__(self, name):
        self.name = name
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls.append(request)
        return None

    def wait_for_service(self, timeout_sec=0.0):
        return True

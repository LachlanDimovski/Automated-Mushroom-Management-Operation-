class ActionClient:
    def __init__(self, node, action_type, name):
        self.node = node
        self.name = name
        self.goals = []
        self.available = True

    def wait_for_server(self, timeout_sec=0.0):
        return self.available

    def server_is_ready(self):
        return self.available

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _Future()


class _Future:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        return None

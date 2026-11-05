"""Is the simulation actually ready for a mission to start?

"Prepared" has to mean something stronger than "the launch command did not
fail". Gazebo takes half a minute, Nav2 comes up after it, and each YOLO
instance loads its weights after that - so a mission started the moment the
launch returns spends its first minute with a rover that cannot plan and
cameras that show nothing, and the mission clock counts all of it.

So the GUI waits for a list of things it can see for itself, and only calls
a run prepared when every one of them has arrived. The list is deliberately
made of observations rather than guesses: a frame off a camera topic, a
publisher on a detector's topic, Nav2's action server answering, the rover's
map frame appearing in TF.

This module is only the list and the bookkeeping, with no ROS in it, so the
tests can drive it with a dictionary. `ros_bridge.probe_readiness()` is what
actually looks.
"""

# key, what it says on the checklist, what it is really waiting for
CHECKS = (
    ('sim', 'Simulation', 'Gazebo running and publishing its clock'),
    ('nodes', 'Mission nodes', 'the skidsteer and quadcopter controllers started'),
    ('rover_odom', 'Skidsteer telemetry', 'odometry arriving from the skidsteer'),
    ('quad_odom', 'Quadcopter telemetry', 'odometry arriving from the quadcopter'),
    ('rover_cam', 'Skidsteer camera', 'frames arriving on the skidsteer camera'),
    ('quad_cam', 'Quadcopter camera', 'frames arriving on the quadcopter camera'),
    ('rover_yolo', 'Skidsteer detector', 'YOLO loaded and publishing detections'),
    ('quad_yolo', 'Quadcopter detector', 'YOLO loaded and publishing detections'),
    ('nav2', 'Nav2', 'the navigate_to_pose action server answering'),
    ('map_frame', 'Rover map frame', 'husky1_map on /husky1/tf - where goals are sent'),
)

CHECK_KEYS = tuple(key for key, _label, _detail in CHECKS)

# Checks that stop a launch from being possible at all, rather than just
# making it worse. Everything else can be waived by the operator.
REQUIRED = ('sim', 'nodes')

# Groups where at least one member has to be there. A run with no camera at
# all cannot detect a single mushroom, so "launch anyway" on one is not a
# degraded run, it is a guaranteed empty one - which is what made a Launch
# button that did nothing useful look like a Launch button that was broken.
REQUIRED_ANY = (
    (('rover_cam', 'quad_cam'), 'a camera'),
)

# When to stop waiting. Measured from the last thing that arrived, not from
# the start: a river world takes a lot longer to come up than a forest one,
# but in both cases pieces arrive steadily until the last one does. Forty-five
# seconds with nothing new is a stall, whatever the world. The absolute cap is
# a backstop for the case where the very first thing never arrives.
STALL_S = 45.0
# ...except on a world that takes a long time to load. The river worlds are a
# heightmap, water and several hundred models, and under software rendering
# the gap between one robot's camera arriving and the other's can be more
# than forty-five seconds on its own - the quadcopter spawns three seconds
# after the skidsteer and its camera sensor only starts once the world has
# finished loading. Giving up at forty-five there was the GUI being impatient
# rather than anything being wrong, and the result looked identical: a
# quadcopter camera tile that never came up and a LAUNCH ANYWAY button.
HEAVY_STALL_S = 150.0
PATIENCE_S = 300.0
HEAVY_PATIENCE_S = 480.0


class Readiness:
    """What has arrived so far, and what that means for the Launch button."""

    def __init__(self, checks=CHECKS, heavy=False):
        self.set_heavy(heavy)
        self.checks = tuple(checks)
        self.state = {key: False for key, _l, _d in self.checks}
        self.first_seen = {}
        self.elapsed = 0.0
        self.last_change = 0.0

    def set_heavy(self, heavy):
        """How long to wait before calling it. A river world gets longer."""
        self.heavy = bool(heavy)
        self.stall_s = HEAVY_STALL_S if self.heavy else STALL_S
        self.patience_s = HEAVY_PATIENCE_S if self.heavy else PATIENCE_S

    def reset(self):
        for key in self.state:
            self.state[key] = False
        self.first_seen.clear()
        self.elapsed = 0.0
        self.last_change = 0.0

    def update(self, probe, elapsed=None):
        """Fold one observation in. Checks latch: arrived once is arrived.

        Latching is on purpose. A camera at 5 Hz and a detector at 2 Hz will
        both miss the odd poll, and a checklist that flickered back to a cross
        would read as a fault when nothing is wrong.
        """
        if elapsed is not None:
            self.elapsed = float(elapsed)
        for key in self.state:
            if not self.state[key] and bool(probe.get(key)):
                self.state[key] = True
                self.first_seen[key] = self.elapsed
                self.last_change = self.elapsed
        return self.rows()

    def rows(self):
        """[(key, label, detail, ok)] in checklist order."""
        return [(key, label, detail, self.state[key])
                for key, label, detail in self.checks]

    def ready(self):
        return all(self.state.values())

    def can_launch(self):
        """Enough is up that a mission would not be a waste of time."""
        if not all(self.state[key] for key in REQUIRED if key in self.state):
            return False
        for keys, _label in REQUIRED_ANY:
            present = [k for k in keys if k in self.state]
            if present and not any(self.state[k] for k in present):
                return False
        return True

    def blocking(self):
        """What is stopping a launch outright, in checklist words."""
        out = [label for key, label, _d in self.checks
               if key in REQUIRED and not self.state[key]]
        for keys, label in REQUIRED_ANY:
            present = [k for k in keys if k in self.state]
            if present and not any(self.state[k] for k in present):
                out.append(label)
        return out

    def missing(self):
        return [label for key, label, _detail in self.checks if not self.state[key]]

    def done_count(self):
        return sum(1 for ok in self.state.values() if ok)

    def stalled_for(self):
        return max(0.0, self.elapsed - self.last_change)

    def impatient(self):
        """Long enough that whatever is missing is probably not coming."""
        if self.ready():
            return False
        return self.stalled_for() >= self.stall_s or self.elapsed >= self.patience_s

    def summary(self):
        if self.ready():
            return 'Everything is up.'
        missing = self.missing()
        if self.impatient():
            return ('Nothing new in %.0f s. Still missing: ' % self.stalled_for()
                    + ', '.join(m.lower() for m in missing)
                    + ('. This is a heavy world, so it was given %.0f s - if '
                       'a camera is what is missing, preparing again usually '
                       'gets it.' % self.stall_s if self.heavy else ''))
        if not self.can_launch():
            blocking = self.blocking()
            if blocking:
                return ('Cannot launch yet - still waiting for '
                        + ', '.join(b.lower() for b in blocking))
            return f'Waiting for {missing[0].lower()}...'
        if len(missing) == 1:
            return f'Waiting for the {missing[0].lower()}.'
        return f'Waiting for {len(missing)} things: ' + ', '.join(m.lower() for m in missing)

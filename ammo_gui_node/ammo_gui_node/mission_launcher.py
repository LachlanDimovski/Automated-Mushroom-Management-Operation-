"""Starts/stops the ammo_mission_control launch file for whichever mode was
picked on the home page, so "Begin Mission" is one button rather than a
separate terminal command.

Deliberately scoped to the mission-control layer only, not the whole sim:
world choice, husky/parrot enable flags, SLAM/Nav2/RViz, YOLO - the heavy
41068_ignition_bringup launch - stay a separate step done from a terminal
first, same as the cheat sheet already describes. Folding that in too would
mean this GUI owning Gazebo's startup time and every one of those launch
args on its home page, for a part of the workflow that isn't really
"operator" territory - it's the one-per-session setup step, not something
you reach for per mission.

    autonomous -> ammo_mission_control autonomous.launch.py
    manual     -> ammo_mission_control mushroom_mission.launch.py
                  (auto_trigger stays on by default in that file, so the
                  husky still dispatches itself to mushrooms the quad
                  spots - "manual" here means the operator flies the quad
                  by hand, via this GUI's keyboard teleop, not that mushroom
                  marking itself is manual too. See teleop_widget.py.)

Started with `ros2 launch`, in its own process group, so stop() can send a
single Ctrl-C-equivalent (SIGINT) to that whole group - the same clean-stop
signal `ros2 launch` itself handles when you Ctrl-C it in a terminal, which
lets each node's own shutdown logic run (e.g. ps4_drone_test.py's shutdown()
publishing a few zero-Twists before exiting) instead of hard-killing them.
"""
import os
import signal
import subprocess
import time

LAUNCH_FILES = {
    'autonomous': ('ammo_mission_control', 'autonomous.launch.py'),
    'manual': ('ammo_mission_control', 'mushroom_mission.launch.py'),
}

STOP_TIMEOUT_S = 5.0


class MissionLauncher:

    def __init__(self):
        self._proc = None
        self._mode = None

    @property
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, mode: str):
        if mode not in LAUNCH_FILES:
            raise ValueError(f'unknown mission mode {mode!r}')
        if self.is_running:
            self.stop()

        package, launch_file = LAUNCH_FILES[mode]
        # start_new_session=True (its own process group) so stop() can
        # signal the whole tree ros2 launch spawns, not just the ros2
        # launch process itself.
        self._proc = subprocess.Popen(
            ['ros2', 'launch', package, launch_file],
            start_new_session=True,
        )
        self._mode = mode

    def stop(self):
        if not self.is_running:
            self._proc = None
            return

        pgid = os.getpgid(self._proc.pid)
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            self._proc = None
            return

        deadline = time.monotonic() + STOP_TIMEOUT_S
        while self.is_running and time.monotonic() < deadline:
            time.sleep(0.1)

        if self.is_running:
            # Didn't exit cleanly in time - escalate rather than leave
            # ros2 launch (and everything under it) running forever.
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        self._proc = None
        self._mode = None
